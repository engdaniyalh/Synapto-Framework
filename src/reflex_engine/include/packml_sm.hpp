/**
 * @file packml_sm.hpp
 * @brief ISA-TR88.00.02 (PackML) State Machine Definitions for Synapto-Reflex.
 *
 * Implements standard industrial PackML state machine semantics with clear
 * separation between Waiting States (steady) and Acting States (transitional).
 * Utilizes atomic operations to guarantee safe cross-thread state reading by
 * the open62541 OPC UA server and telemetry worker threads.
 */

#pragma once

#include <cstdint>
#include <string>
#include <atomic>
#include <string_view>

/**
 * @enum PackMLState
 * @brief Standardized PackML state identifiers conforming to ISA-TR88.00.02.
 */
enum class PackMLState : int32_t {
    CLEARING     = 1,
    STOPPED      = 2,
    STARTING     = 3,
    IDLE         = 4,
    SUSPENDED    = 5,
    EXECUTE      = 6,
    STOPPING     = 7,
    ABORTING     = 8,
    ABORTED      = 9,
    HOLDING      = 10,
    HELD         = 11,
    UNHOLDING    = 12,
    SUSPENDING   = 13,
    UNSUSPENDING = 14,
    RESETTING    = 15,
    COMPLETING   = 16,
    COMPLETE     = 17
};

/**
 * @enum PackMLCommand
 * @brief Standardized PackML state transition commands.
 */
enum class PackMLCommand : int32_t {
    NONE    = 0,
    RESET   = 1,
    START   = 2,
    STOP    = 3,
    HOLD    = 4,
    UNHOLD  = 5,
    SUSPEND = 6,
    UNSUSPEND = 7,
    ABORT   = 8,
    CLEAR   = 9
};

/**
 * @brief Convert PackMLState enum to human-readable string view.
 */
[[nodiscard]] constexpr std::string_view stateToString(PackMLState s) noexcept {
    switch (s) {
        case PackMLState::CLEARING:     return "CLEARING";
        case PackMLState::STOPPED:      return "STOPPED";
        case PackMLState::STARTING:     return "STARTING";
        case PackMLState::IDLE:         return "IDLE";
        case PackMLState::SUSPENDED:    return "SUSPENDED";
        case PackMLState::EXECUTE:      return "EXECUTE";
        case PackMLState::STOPPING:     return "STOPPING";
        case PackMLState::ABORTING:     return "ABORTING";
        case PackMLState::ABORTED:      return "ABORTED";
        case PackMLState::HOLDING:      return "HOLDING";
        case PackMLState::HELD:         return "HELD";
        case PackMLState::UNHOLDING:    return "UNHOLDING";
        case PackMLState::SUSPENDING:   return "SUSPENDING";
        case PackMLState::UNSUSPENDING: return "UNSUSPENDING";
        case PackMLState::RESETTING:    return "RESETTING";
        case PackMLState::COMPLETING:   return "COMPLETING";
        case PackMLState::COMPLETE:     return "COMPLETE";
        default:                        return "UNKNOWN";
    }
}

/**
 * @brief Helper checking if state is an acting (transitional) state.
 */
[[nodiscard]] constexpr bool isActingState(PackMLState s) noexcept {
    switch (s) {
        case PackMLState::CLEARING:
        case PackMLState::STARTING:
        case PackMLState::STOPPING:
        case PackMLState::ABORTING:
        case PackMLState::HOLDING:
        case PackMLState::UNHOLDING:
        case PackMLState::SUSPENDING:
        case PackMLState::UNSUSPENDING:
        case PackMLState::RESETTING:
        case PackMLState::COMPLETING:
            return true;
        default:
            return false;
    }
}

/**
 * @class PackMLStateMachine
 * @brief Manages industrial state transitions based on incoming commands and internal stepping.
 */
class PackMLStateMachine {
private:
    /** Thread-safe atomic storage of the current machine state. */
    std::atomic<PackMLState> current_state_{PackMLState::STOPPED};
    int32_t acting_ticks_{0};
    int32_t acting_duration_ticks_{10}; // Ticks needed to complete an acting transition (e.g., 100ms at 100Hz)

public:
    explicit PackMLStateMachine(PackMLState initial_state = PackMLState::STOPPED, int32_t acting_duration_ticks = 10) noexcept
        : current_state_(initial_state), acting_duration_ticks_(acting_duration_ticks) {}

    /**
     * @brief Processes an incoming PackML command and transitions to the appropriate state.
     *
     * In accordance with ISA-TR88.00.02:
     * - STOPPED + RESET -> RESETTING
     * - IDLE + START -> STARTING
     * - EXECUTE + STOP -> STOPPING
     * - EXECUTE + HOLD -> HOLDING
     * - HELD + UNHOLD -> UNHOLDING
     * - ABORTED + CLEAR -> CLEARING
     * - Any non-ABORTED state + ABORT -> ABORTING
     *
     * @param cmd The incoming command.
     * @return true if command triggered a valid transition, false otherwise.
     */
    bool processCommand(PackMLCommand cmd) noexcept {
        const PackMLState current = current_state_.load(std::memory_order_relaxed);

        if (cmd == PackMLCommand::NONE) {
            return false;
        }

        // Hardware ABORT overrides any non-ABORTED state immediately into ABORTING
        if (cmd == PackMLCommand::ABORT) {
            if (current != PackMLState::ABORTED && current != PackMLState::ABORTING) {
                current_state_.store(PackMLState::ABORTING, std::memory_order_release);
                acting_ticks_ = 0;
                return true;
            }
            return false;
        }

        switch (current) {
            case PackMLState::STOPPED:
                if (cmd == PackMLCommand::RESET) {
                    current_state_.store(PackMLState::RESETTING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                break;

            case PackMLState::IDLE:
                if (cmd == PackMLCommand::START) {
                    current_state_.store(PackMLState::STARTING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                if (cmd == PackMLCommand::STOP) {
                    current_state_.store(PackMLState::STOPPING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                break;

            case PackMLState::EXECUTE:
                if (cmd == PackMLCommand::STOP) {
                    current_state_.store(PackMLState::STOPPING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                if (cmd == PackMLCommand::HOLD) {
                    current_state_.store(PackMLState::HOLDING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                break;

            case PackMLState::HELD:
                if (cmd == PackMLCommand::UNHOLD) {
                    current_state_.store(PackMLState::UNHOLDING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                if (cmd == PackMLCommand::STOP) {
                    current_state_.store(PackMLState::STOPPING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                break;

            case PackMLState::ABORTED:
                if (cmd == PackMLCommand::CLEAR) {
                    current_state_.store(PackMLState::CLEARING, std::memory_order_release);
                    acting_ticks_ = 0;
                    return true;
                }
                break;

            default:
                // Cannot issue standard commands while currently executing an acting transition
                break;
        }
        return false;
    }

    /**
     * @brief Periodic state machine step method to advance acting/transitional states.
     *
     * Once an acting state completes its execution time/sequence, it transitions to
     * its target waiting state:
     * - RESETTING -> IDLE
     * - STARTING -> EXECUTE
     * - STOPPING -> STOPPED
     * - HOLDING -> HELD
     * - UNHOLDING -> EXECUTE
     * - ABORTING -> ABORTED
     * - CLEARING -> STOPPED
     */
    void step() noexcept {
        const PackMLState current = current_state_.load(std::memory_order_relaxed);
        if (!isActingState(current)) {
            return;
        }

        acting_ticks_++;
        if (acting_ticks_ >= acting_duration_ticks_) {
            acting_ticks_ = 0;
            switch (current) {
                case PackMLState::RESETTING:
                    current_state_.store(PackMLState::IDLE, std::memory_order_release);
                    break;
                case PackMLState::STARTING:
                    current_state_.store(PackMLState::EXECUTE, std::memory_order_release);
                    break;
                case PackMLState::STOPPING:
                    current_state_.store(PackMLState::STOPPED, std::memory_order_release);
                    break;
                case PackMLState::HOLDING:
                    current_state_.store(PackMLState::HELD, std::memory_order_release);
                    break;
                case PackMLState::UNHOLDING:
                    current_state_.store(PackMLState::EXECUTE, std::memory_order_release);
                    break;
                case PackMLState::ABORTING:
                    current_state_.store(PackMLState::ABORTED, std::memory_order_release);
                    break;
                case PackMLState::CLEARING:
                    current_state_.store(PackMLState::STOPPED, std::memory_order_release);
                    break;
                default:
                    break;
            }
        }
    }

    /**
     * @brief Direct state setter for testing or simulation forced overrides.
     */
    void forceState(PackMLState s) noexcept {
        current_state_.store(s, std::memory_order_release);
        acting_ticks_ = 0;
    }

    /**
     * @brief Retrieves the current thread-safe machine state.
     */
    [[nodiscard]] PackMLState getState() const noexcept {
        return current_state_.load(std::memory_order_acquire);
    }
};
