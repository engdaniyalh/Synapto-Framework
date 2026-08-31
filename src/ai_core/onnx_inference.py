"""
@module onnx_inference
@brief State-Aware ONNX Runtime evaluation layer for edge predictive maintenance.

Responsible for receiving telemetry frames, scoring anomalies against historical
distribution configurations (Z-Score), and executing the ONNX model to generate
predictive residual deviation scores.

Incorporates ISA-TR88.00.02 State-Aware Anomaly Gating:
Transitional states (e.g. STARTING, RESETTING) produce expected inrush currents
and are gated to eliminate false-positive emergency ABORT commands. Anomaly triggers
are strictly enforced during steady EXECUTE (6) states.
"""

import os
import numpy as np
import onnxruntime as ort
from typing import Tuple
from src.ai_core.config import SystemConfig
from src.ai_core.export_model import export_to_onnx


class MotorAnomalyDetector:
    """
    @class MotorAnomalyDetector
    @brief Encapsulates state-gated ONNX edge evaluation and statistical inference.
    """

    # ISA-TR88.00.02 PackML State Constant
    PACKML_STATE_EXECUTE = 6

    def __init__(self, config: SystemConfig):
        """
        @brief Initializes the ONNX Runtime session using system configurations.
        @param config SystemConfig containing model paths and thresholds.
        """
        self.cfg = config

        # Ensure ONNX model binary exists before creating session
        if not os.path.exists(self.cfg.onnx_model_path):
            print(f"[AI Core] ONNX model not found at {self.cfg.onnx_model_path}. Exporting baseline...")
            export_to_onnx(self.cfg.onnx_model_path)

        # In production environments (e.g. NVIDIA Jetson Orin), 'TensorrtExecutionProvider'
        # or 'CUDAExecutionProvider' can be added to the providers array.
        self.session = ort.InferenceSession(
            self.cfg.onnx_model_path,
            providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name

    def evaluate_frame(
        self,
        sequence: np.ndarray,
        latest_value: float,
        state: int = PACKML_STATE_EXECUTE
    ) -> Tuple[float, bool]:
        """
        @brief Executes state-gated anomaly scoring on a given telemetry frame.

        Evaluates statistical deviation (Z-score) and predictive health
        by analyzing the residual difference against the ONNX model prediction.

        @param sequence Sliding window of historical float telemetry (shape 1x10x1 or 10)
        @param latest_value The immediate float scalar reading for the current frame.
        @param state PackML state integer (default: 6 for EXECUTE).
        @return Tuple containing (residual_score_float, boolean_flag_is_anomaly).
        """
        # Ensure correct dimensionality (1, seq_len, 1) for ONNX model execution
        if sequence.ndim != 3 or sequence.shape != (1, self.cfg.sequence_length, 1):
            sequence = np.asarray(sequence, dtype=np.float32).reshape(1, self.cfg.sequence_length, 1)

        onnx_outputs = self.session.run(None, {self.input_name: sequence})
        predicted_current = float(onnx_outputs[0][0][0])
        residual = abs(predicted_current - latest_value)

        # State-Aware Anomaly Gating:
        # Transitional states (STARTING=3, RESETTING=15, STOPPING=7, HOLDING=10)
        # naturally produce transient current spikes/inrush. Anomaly abort triggers
        # are strictly active only in the steady EXECUTE (6) state.
        if state != self.PACKML_STATE_EXECUTE:
            return residual, False

        z_score = abs(latest_value - self.cfg.nominal_current_mean) / self.cfg.nominal_current_std
        is_anomaly = bool(
            (z_score >= self.cfg.anomaly_z_threshold) or
            (residual >= self.cfg.residual_threshold)
        )

        return residual, is_anomaly
