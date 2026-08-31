/**
 * @file main.cpp
 * @brief Synapto-Reflex: Two-Thread Decoupled Hard Real-Time & OPC UA Server Engine.
 *
 * Implements a strict two-thread dual-domain architecture:
 * 1. Thread 1 (Hard RT): 100Hz PREEMPT_RT deterministic loop (SCHED_FIFO, priority 98).
 *    Executes PackML state machine, stochastic motor physics modeling, and zero-syscall execution.
 *    Pushes telemetry frames into a 64-byte cache-aligned SPSC lock-free ring buffer.
 * 2. Thread 2 (Non-RT Network Worker): Drains the ring buffer, updates open62541 OPC UA
 *    address space, serves IEC 62541 TCP requests (port 4840), and broadcasts sequenced UDP packets
 *    to the MuJoCo physics twin (port 4242).
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <random>
#include <cstring>
#include <csignal>
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#if defined(__linux__)
#include <sys/mman.h>
#include <sched.h>
#endif

#include <open62541/server.h>
#include <open62541/server_config_default.h>

#include "packml_sm.hpp"
#include "lockfree_queue.hpp"

// --- Telemetry Data Structures ---
struct TelemetryFrame {
    uint64_t seq{0};
    uint64_t timestamp_us{0};
    PackMLState state{PackMLState::STOPPED};
    float motor_current{0.0f};
    float target_velocity{0.0f};
};

// Global Lock-Free SPSC Queue and Inter-Thread Atomics
static SPSCQueue<TelemetryFrame, 1024> g_telemetry_queue;
static std::atomic<PackMLCommand> g_pending_command{PackMLCommand::NONE};
static std::atomic<bool> g_running{true};

static void signal_handler(int sig) {
    (void)sig;
    g_running.store(false, std::memory_order_release);
}

// Utility: get monotonic time in microseconds
static inline uint64_t get_monotonic_us() noexcept {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec / 1000);
}

/**
 * @brief Thread 1: Hard Real-Time Deterministic Control & Physics Loop (100Hz)
 */
void real_time_control_loop() {
#if defined(__linux__)
    // Lock all current and future process memory to prevent page faults in RT path
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "[RT Warning] mlockall failed (requires CAP_SYS_NICE/root). Proceeding with best-effort RT.\n";
    }

    // Set SCHED_FIFO real-time scheduling priority 98
    struct sched_param param{};
    param.sched_priority = 98;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "[RT Warning] sched_setscheduler SCHED_FIFO failed (requires CAP_SYS_NICE/root). Proceeding.\n";
    }
#endif

    std::cout << "[Synapto-Reflex] [RT Thread] Started 100Hz deterministic loop.\n";

    PackMLStateMachine sm(PackMLState::STOPPED, 10); // 10 ticks = 100ms acting duration
    std::mt19937 rng(42); // Deterministic PRNG seed for reproducible physics
    std::normal_distribution<float> dist_nominal(12.5f, 0.4f);
    std::normal_distribution<float> dist_anomaly(19.5f, 1.2f);
    std::normal_distribution<float> dist_inrush(14.0f, 0.5f);

    constexpr long TICK_PERIOD_NS = 10'000'000L; // 10ms -> 100Hz
    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    uint64_t seq_num = 0;
    uint32_t execute_ticks = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        // Compute next absolute tick boundary
        next_tick.tv_nsec += TICK_PERIOD_NS;
        while (next_tick.tv_nsec >= 1'000'000'000L) {
            next_tick.tv_nsec -= 1'000'000'000L;
            next_tick.tv_sec += 1;
        }

        // Sleep until next exact period boundary
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);

        // 1. Process pending commands from Non-RT thread
        PackMLCommand cmd = g_pending_command.exchange(PackMLCommand::NONE, std::memory_order_acq_rel);
        if (cmd != PackMLCommand::NONE) {
            sm.processCommand(cmd);
        }

        // 2. Step internal acting states (e.g. STARTING -> EXECUTE, STOPPING -> STOPPED)
        sm.step();
        const PackMLState current_state = sm.getState();

        // 3. Compute deterministic physics & sensor telemetry
        float current_amps = 0.0f;
        float target_vel = 0.0f;

        switch (current_state) {
            case PackMLState::EXECUTE:
                execute_ticks++;
                target_vel = 1.5f;
                // Periodically inject simulated mechanical wear/current spike every 1500 ticks (15s)
                if ((execute_ticks % 1500) > 1350) {
                    current_amps = dist_anomaly(rng);
                } else {
                    current_amps = dist_nominal(rng);
                }
                break;

            case PackMLState::STARTING:
            case PackMLState::RESETTING:
                execute_ticks = 0;
                target_vel = 0.5f;
                current_amps = dist_inrush(rng);
                break;

            case PackMLState::HOLDING:
            case PackMLState::HELD:
                target_vel = 0.0f;
                current_amps = 2.0f; // Holding brake current
                break;

            case PackMLState::STOPPING:
            case PackMLState::STOPPED:
                execute_ticks = 0;
                target_vel = 0.0f;
                current_amps = 0.05f; // Quiescent idle draw
                break;

            case PackMLState::ABORTING:
            case PackMLState::ABORTED:
                execute_ticks = 0;
                target_vel = 0.0f;
                current_amps = 0.0f; // Power fully isolated
                break;

            default:
                target_vel = 0.0f;
                current_amps = 0.1f;
                break;
        }

        // 4. Populate and push telemetry frame to Lock-Free SPSC Queue
        TelemetryFrame frame;
        frame.seq = ++seq_num;
        frame.timestamp_us = get_monotonic_us();
        frame.state = current_state;
        frame.motor_current = current_amps;
        frame.target_velocity = target_vel;

        g_telemetry_queue.push(frame);
    }

    std::cout << "[Synapto-Reflex] [RT Thread] Exited cleanly.\n";
}

/**
 * @brief Thread 2: Non-RT Network Worker (OPC UA Server & UDP Twin Broadcaster)
 */
void network_worker_loop() {
    // 1. Initialize open62541 OPC UA Server
    UA_Server *server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    // Register custom semantic namespace URI: urn:synapto:control:v1
    UA_UInt16 ns_synapto = UA_Server_addNamespace(server, "urn:synapto:control:v1");

    // Configure Address Space: SynaptoMachine Object
    UA_NodeId machineNode = UA_NODEID_STRING(ns_synapto, const_cast<char*>("SynaptoMachine"));
    UA_ObjectAttributes oAttr = UA_ObjectAttributes_default;
    oAttr.displayName = UA_LOCALIZEDTEXT(const_cast<char*>("en-US"), const_cast<char*>("SynaptoMachine"));
    UA_Server_addObjectNode(server, machineNode, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), UA_QUALIFIEDNAME(ns_synapto, const_cast<char*>("SynaptoMachine")),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), oAttr, nullptr, nullptr);

    // Variable Node: StateCurrent (Read-Only)
    UA_VariableAttributes sAttr = UA_VariableAttributes_default;
    int32_t init_state = static_cast<int32_t>(PackMLState::STOPPED);
    UA_Variant_setScalar(&sAttr.value, &init_state, &UA_TYPES[UA_TYPES_INT32]);
    sAttr.displayName = UA_LOCALIZEDTEXT(const_cast<char*>("en-US"), const_cast<char*>("StateCurrent"));
    sAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_NodeId stateNode = UA_NODEID_STRING(ns_synapto, const_cast<char*>("PackML.StateCurrent"));
    UA_Server_addVariableNode(server, stateNode, machineNode, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(ns_synapto, const_cast<char*>("StateCurrent")), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              sAttr, nullptr, nullptr);

    // Variable Node: MotorCurrent (Read-Only)
    UA_VariableAttributes tAttr = UA_VariableAttributes_default;
    float init_curr = 0.0f;
    UA_Variant_setScalar(&tAttr.value, &init_curr, &UA_TYPES[UA_TYPES_FLOAT]);
    tAttr.displayName = UA_LOCALIZEDTEXT(const_cast<char*>("en-US"), const_cast<char*>("MotorCurrent"));
    tAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_NodeId telemetryNode = UA_NODEID_STRING(ns_synapto, const_cast<char*>("Telemetry.MotorCurrent"));
    UA_Server_addVariableNode(server, telemetryNode, machineNode, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(ns_synapto, const_cast<char*>("MotorCurrent")), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              tAttr, nullptr, nullptr);

    // Variable Node: Command (Read/Write)
    UA_VariableAttributes cAttr = UA_VariableAttributes_default;
    int32_t init_cmd = 0;
    UA_Variant_setScalar(&cAttr.value, &init_cmd, &UA_TYPES[UA_TYPES_INT32]);
    cAttr.displayName = UA_LOCALIZEDTEXT(const_cast<char*>("en-US"), const_cast<char*>("Command"));
    cAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_NodeId cmdNode = UA_NODEID_STRING(ns_synapto, const_cast<char*>("PackML.Command"));
    UA_Server_addVariableNode(server, cmdNode, machineNode, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(ns_synapto, const_cast<char*>("Command")), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              cAttr, nullptr, nullptr);

    // 2. Initialize non-blocking UDP socket for MuJoCo Twin
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock >= 0) {
        int flags = fcntl(udp_sock, F_GETFL, 0);
        fcntl(udp_sock, F_SETFL, flags | O_NONBLOCK);
    }

    struct sockaddr_in twin_addr{};
    twin_addr.sin_family = AF_INET;
    twin_addr.sin_port = htons(4242);
    const char* twin_host = std::getenv("TWIN_HOST") ? std::getenv("TWIN_HOST") : "127.0.0.1";
    inet_pton(AF_INET, twin_host, &twin_addr.sin_addr);

    std::cout << "[Synapto-Reflex] [Network Worker] OPC UA Server listening on opc.tcp://0.0.0.0:4840 (ns=" << ns_synapto << ": urn:synapto:control:v1)\n";
    std::cout << "[Synapto-Reflex] [Network Worker] Telemetry UDP dispatching to " << twin_host << ":4242 (v1 schema)\n";

    TelemetryFrame latest_frame{};

    while (g_running.load(std::memory_order_relaxed)) {
        // A. Drain SPSC Queue
        bool new_data_received = false;
        TelemetryFrame popped_frame;
        while (g_telemetry_queue.pop(popped_frame)) {
            latest_frame = popped_frame;
            new_data_received = true;
        }

        // B. Publish latest state and telemetry to OPC UA Address Space
        if (new_data_received) {
            int32_t s_val = static_cast<int32_t>(latest_frame.state);
            UA_Variant v_state;
            UA_Variant_init(&v_state);
            UA_Variant_setScalar(&v_state, &s_val, &UA_TYPES[UA_TYPES_INT32]);
            UA_Server_writeValue(server, stateNode, v_state);

            float t_val = latest_frame.motor_current;
            UA_Variant v_tel;
            UA_Variant_init(&v_tel);
            UA_Variant_setScalar(&v_tel, &t_val, &UA_TYPES[UA_TYPES_FLOAT]);
            UA_Server_writeValue(server, telemetryNode, v_tel);

            // C. Dispatch versioned, ordered JSON UDP frame to MuJoCo Twin
            if (udp_sock >= 0) {
                char json_buf[256];
                int len = std::snprintf(json_buf, sizeof(json_buf),
                    "{\"v\":1,\"seq\":%llu,\"ts_us\":%llu,\"state\":%d,\"current\":%.2f,\"target_vel\":%.2f}",
                    static_cast<unsigned long long>(latest_frame.seq),
                    static_cast<unsigned long long>(latest_frame.timestamp_us),
                    static_cast<int>(latest_frame.state),
                    latest_frame.motor_current,
                    latest_frame.target_velocity
                );
                if (len > 0) {
                    sendto(udp_sock, json_buf, len, 0, reinterpret_cast<struct sockaddr*>(&twin_addr), sizeof(twin_addr));
                }
            }
        }

        // D. Check for incoming OPC UA Commands from AI Core / SCADA
        UA_Variant outVariant;
        UA_Variant_init(&outVariant);
        UA_Server_readValue(server, cmdNode, &outVariant);
        if (outVariant.data) {
            int32_t incoming_cmd_val = *reinterpret_cast<int32_t*>(outVariant.data);
            if (incoming_cmd_val != 0) {
                auto cmd = static_cast<PackMLCommand>(incoming_cmd_val);
                g_pending_command.store(cmd, std::memory_order_release);

                // Reset command node to NONE (0)
                int32_t zero = 0;
                UA_Variant vZero;
                UA_Variant_init(&vZero);
                UA_Variant_setScalar(&vZero, &zero, &UA_TYPES[UA_TYPES_INT32]);
                UA_Server_writeValue(server, cmdNode, vZero);
            }
        }
        UA_Variant_clear(&outVariant);

        // E. Run open62541 iteration
        UA_Server_run_iterate(server, false);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (udp_sock >= 0) {
        close(udp_sock);
    }
    UA_Server_delete(server);
    std::cout << "[Synapto-Reflex] [Network Worker] Exited cleanly.\n";
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================================\n";
    std::cout << "⚡ Synapto-Reflex Deterministic Edge Engine (C++20)\n";
    std::cout << "========================================================\n";

    // Launch Hard Real-Time Thread and Non-RT Network Worker
    std::thread rt_thread(real_time_control_loop);
    std::thread net_thread(network_worker_loop);

    rt_thread.join();
    net_thread.join();

    return 0;
}
