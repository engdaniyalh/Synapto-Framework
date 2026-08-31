"""
@module twin_viewer
@brief MuJoCo 3D Digital Twin Simulation & Real-Time Telemetry Viewer.

Ingests high-frequency UDP telemetry dispatched by the Synapto-Reflex engine,
enforces strict packet ordering via sequence number verification, and synchronizes
actuator controls and Andon signal illumination states in MuJoCo.

Supports both interactive rendering (GLFW viewer) and headless CI stepping mode.
"""

import sys
import os
import socket
import json
import threading
import time
import argparse
import mujoco

# PackML State Identifiers
STATE_CLEARING = 1
STATE_STOPPED = 2
STATE_STARTING = 3
STATE_IDLE = 4
STATE_SUSPENDED = 5
STATE_EXECUTE = 6
STATE_STOPPING = 7
STATE_ABORTING = 8
STATE_ABORTED = 9
STATE_HOLDING = 10
STATE_HELD = 11
STATE_UNHOLDING = 12
STATE_RESETTING = 15

# Global Telemetry State
g_lock = threading.Lock()
g_last_seq = 0
g_state = STATE_STOPPED
g_motor_current = 0.0
g_target_vel = 0.0
g_running = True


def udp_telemetry_listener(port: int = 4242):
    """
    @brief Background UDP listener receiving ordered telemetry packets.
    """
    global g_last_seq, g_state, g_motor_current, g_target_vel, g_running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)

    print(f"[Synapto-Twin] Listening for UDP telemetry on 0.0.0.0:{port}...")

    while g_running:
        try:
            data, _ = sock.recvfrom(1024)
            payload = json.loads(data.decode("utf-8"))

            seq = int(payload.get("seq", 0))

            with g_lock:
                # Monotonic sequence check: reject out-of-order stale packets
                if seq > g_last_seq:
                    g_last_seq = seq
                    g_state = int(payload.get("state", STATE_STOPPED))
                    g_motor_current = float(payload.get("current", 0.0))
                    g_target_vel = float(payload.get("target_vel", 0.0))

        except socket.timeout:
            continue
        except Exception as e:
            if g_running:
                pass

    sock.close()


def apply_visuals(model, data, geom_andon_id, actuator_motor_id, state: int, target_vel: float, motor_current: float):
    """
    @brief Updates MuJoCo physics actuators and Andon tower RGBA colors based on PackML state.
    """
    if state == STATE_EXECUTE:
        # Green = Active Execution
        model.geom_rgba[geom_andon_id] = [0.0, 1.0, 0.0, 1.0]
        data.ctrl[actuator_motor_id] = target_vel * 2.0
    elif state in (STATE_ABORTING, STATE_ABORTED):
        # Red = Emergency Abort
        model.geom_rgba[geom_andon_id] = [1.0, 0.0, 0.0, 1.0]
        data.ctrl[actuator_motor_id] = 0.0
    elif state in (STATE_HOLDING, STATE_HELD):
        # Amber = Suspended / Held
        model.geom_rgba[geom_andon_id] = [1.0, 0.7, 0.0, 1.0]
        data.ctrl[actuator_motor_id] = 0.0
    elif state in (STATE_STARTING, STATE_RESETTING, STATE_UNHOLDING, STATE_CLEARING):
        # Blue = State Transitioning / Inrush
        model.geom_rgba[geom_andon_id] = [0.2, 0.5, 1.0, 1.0]
        data.ctrl[actuator_motor_id] = target_vel
    else:
        # White = Stopped / Idle Quiescent
        model.geom_rgba[geom_andon_id] = [0.9, 0.9, 0.9, 1.0]
        data.ctrl[actuator_motor_id] = 0.0


def main():
    global g_running

    parser = argparse.ArgumentParser(description="Synapto MuJoCo Digital Twin Viewer")
    parser.add_argument("--headless", action="store_true", help="Run in headless simulation mode without GUI")
    parser.add_argument("--port", type=int, default=4242, help="UDP telemetry port")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration in seconds (0 = infinite)")
    args = parser.parse_args()

    is_headless = args.headless or (os.getenv("HEADLESS_MODE", "false").lower() == "true")

    model_path = os.path.join(os.path.dirname(__file__), '../model/machine.xml')
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    geom_andon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "andon_light")
    actuator_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_vel")

    # Start UDP listener thread
    listener_thread = threading.Thread(target=udp_telemetry_listener, args=(args.port,), daemon=True)
    listener_thread.start()

    start_time = time.time()

    if is_headless:
        print("[Synapto-Twin] Headless physics mode active. Stepping simulation...")
        try:
            while g_running:
                with g_lock:
                    state, target_vel, current = g_state, g_target_vel, g_motor_current

                apply_visuals(model, data, geom_andon_id, actuator_motor_id, state, target_vel, current)
                mujoco.mj_step(model, data)
                time.sleep(model.opt.timestep)

                if args.duration > 0 and (time.time() - start_time) >= args.duration:
                    break
        except KeyboardInterrupt:
            pass
    else:
        import mujoco.viewer
        print("[Synapto-Twin] Launching interactive MuJoCo 3D Viewer...")
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running() and g_running:
                    step_start = time.time()
                    with g_lock:
                        state, target_vel, current = g_state, g_target_vel, g_motor_current

                    apply_visuals(model, data, geom_andon_id, actuator_motor_id, state, target_vel, current)
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

                    if args.duration > 0 and (time.time() - start_time) >= args.duration:
                        break
        except Exception as e:
            print(f"[Synapto-Twin] Interactive viewer fallback: {e}")

    g_running = False
    listener_thread.join(timeout=1.0)
    print("[Synapto-Twin] Digital Twin stopped.")


if __name__ == "__main__":
    main()
