"""
@module main
@brief Synapto-AI Core: Asynchronous OPC UA Closed-Loop Control & Inference Daemon.

Connects to the Reflex open62541 OPC UA server via asyncua, subscribes to
real-time telemetry and PackML state changes, runs ONNX Runtime anomaly scoring,
and executes deterministic closed-loop safety commands (ABORT).
"""

import os
import asyncio
import logging
from collections import deque
import numpy as np
from asyncua import Client, ua
from src.ai_core.config import config
from src.ai_core.onnx_inference import MotorAnomalyDetector

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [AI Core] %(message)s'
)


class TelemetryHandler:
    """
    @class TelemetryHandler
    @brief Handles asynchronous OPC UA data change notifications for sensor streams.
    """

    def __init__(self, detector: MotorAnomalyDetector, cmd_node, state_node):
        self.detector = detector
        self.cmd_node = cmd_node
        self.state_node = state_node
        self.window = deque(maxlen=detector.cfg.sequence_length)
        self.current_state = 2  # Default to STOPPED (2)

    async def on_state_change(self, new_state: int):
        self.current_state = new_state
        logging.info(f"Machine State updated to: {self.current_state}")

    async def datachange_notification(self, node, val, data):
        """
        @brief Invoked upon every OPC UA variable subscription change.
        """
        current_reading = float(val)
        self.window.append(current_reading)

        if len(self.window) == self.detector.cfg.sequence_length:
            seq = np.array(list(self.window), dtype=np.float32).reshape(1, self.detector.cfg.sequence_length, 1)
            residual, is_anomaly = self.detector.evaluate_frame(
                sequence=seq,
                latest_value=current_reading,
                state=self.current_state
            )

            if is_anomaly:
                logging.warning(
                    f"🚨 [ANOMALY DETECTED] State={self.current_state}, Current={current_reading:.2f}A, "
                    f"Residual={residual:.3f} >= Threshold. Issuing PackML ABORT!"
                )
                # PackML ABORT Command = 8
                abort_variant = ua.DataValue(ua.Variant(8, ua.VariantType.Int32))
                await self.cmd_node.write_value(abort_variant)
                self.window.clear()


async def run_ai_core():
    endpoint = os.getenv("OPCUA_ENDPOINT", config.opc_ua_endpoint)
    logging.info(f"Initializing AI Core inference engine connecting to {endpoint}...")

    detector = MotorAnomalyDetector(config)

    # Retry connection to Reflex OPC UA server until ready
    client = Client(url=endpoint)
    connected = False
    for attempt in range(1, 30):
        try:
            await client.connect()
            connected = True
            logging.info(f"Connected to Synapto-Reflex OPC UA Server at {endpoint}")
            break
        except Exception as e:
            logging.info(f"Waiting for Reflex server to be ready (attempt {attempt}/30)... ({e})")
            await asyncio.sleep(1.0)

    if not connected:
        logging.error("Failed to connect to OPC UA Server after maximum retries. Exiting.")
        return

    async with client:
        # Retrieve OPC UA Node IDs
        telemetry_node = client.get_node("ns=1;s=Telemetry.MotorCurrent")
        cmd_node = client.get_node("ns=1;s=PackML.Command")
        state_node = client.get_node("ns=1;s=PackML.StateCurrent")

        handler = TelemetryHandler(detector, cmd_node, state_node)

        # Subscribe to high-frequency Telemetry data changes
        sub = await client.create_subscription(10, handler)
        await sub.subscribe_data_change(telemetry_node)

        logging.info("Active telemetry subscription established. Monitoring in closed loop.")

        # Supervisory polling loop for machine lifecycle
        while True:
            try:
                state_val = await state_node.read_value()
                if state_val != handler.current_state:
                    await handler.on_state_change(int(state_val))

                # Autonomous loop for continuous demonstration:
                # If STOPPED (2), issue RESET (1) -> IDLE (4) -> START (2) -> EXECUTE (6)
                if state_val == 2:  # STOPPED
                    logging.info("Machine in STOPPED state. Initializing RESET sequence...")
                    await asyncio.sleep(2.0)
                    await cmd_node.write_value(ua.DataValue(ua.Variant(1, ua.VariantType.Int32)))  # RESET
                elif state_val == 4:  # IDLE
                    logging.info("Machine IDLE and ready. Commanding START to EXECUTE...")
                    await asyncio.sleep(1.0)
                    await cmd_node.write_value(ua.DataValue(ua.Variant(2, ua.VariantType.Int32)))  # START
                elif state_val == 9:  # ABORTED
                    logging.info("Machine is ABORTED after anomaly. Cooling down before CLEAR...")
                    await asyncio.sleep(4.0)
                    await cmd_node.write_value(ua.DataValue(ua.Variant(9, ua.VariantType.Int32)))  # CLEAR

            except Exception as e:
                logging.warning(f"OPC UA State poll encounter: {e}")

            await asyncio.sleep(1.0)


def main():
    try:
        asyncio.run(run_ai_core())
    except (KeyboardInterrupt, SystemExit):
        logging.info("AI Core daemon stopped by user.")


if __name__ == "__main__":
    main()
