"""
@file test_inference.py
@brief PyTest suite for Synapto AI Core: Parity, State-Gated Anomaly Detection & Config.
"""

import os
import pytest
import numpy as np
import torch
import onnxruntime as ort

from src.ai_core.config import SystemConfig
from src.ai_core.export_model import MotorLSTM, export_to_onnx, train_baseline_model
from src.ai_core.onnx_inference import MotorAnomalyDetector


@pytest.fixture(scope="module")
def exported_model_setup(tmp_path_factory):
    """
    @brief Trains and exports a deterministic ONNX model for testing.
    """
    model_dir = tmp_path_factory.mktemp("models")
    model_path = str(model_dir / "test_motor_lstm.onnx")

    torch.manual_seed(42)
    model = MotorLSTM()
    model.eval()

    export_to_onnx(model_path, model=model)
    return model, model_path


@pytest.fixture
def test_config(exported_model_setup):
    _, model_path = exported_model_setup
    return SystemConfig(
        ONNX_MODEL_PATH=model_path,
        nominal_current_mean=12.5,
        nominal_current_std=0.45,
        ANOMALY_Z_THRESHOLD=3.0,
        residual_threshold=2.0,
        WINDOW_SIZE=10
    )


def test_pytorch_onnx_parity(exported_model_setup):
    """
    @test Verifies numerical parity between PyTorch eager execution and ONNX Runtime.
    Acceptance Criteria: max |y_torch - y_onnx| < 1e-5
    """
    torch_model, onnx_path = exported_model_setup
    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    np.random.seed(123)
    dummy_np = np.random.randn(1, 10, 1).astype(np.float32)
    dummy_torch = torch.from_numpy(dummy_np)

    with torch.no_grad():
        torch_out = torch_model(dummy_torch).numpy()

    onnx_out = session.run(None, {input_name: dummy_np})[0]

    max_diff = np.max(np.abs(torch_out - onnx_out))
    print(f"[Parity Test] Max numerical diff: {max_diff}")
    assert max_diff < 1e-5


def test_nominal_telemetry_in_execute_state(test_config):
    """
    @test Verifies nominal current (12.5A) in EXECUTE state (6) flags no anomaly.
    """
    detector = MotorAnomalyDetector(test_config)
    sequence = np.full((1, 10, 1), 12.5, dtype=np.float32)

    residual, is_anomaly = detector.evaluate_frame(
        sequence=sequence,
        latest_value=12.55,
        state=6  # EXECUTE
    )

    assert not is_anomaly
    assert isinstance(residual, float)


def test_state_aware_gating_ignores_inrush_in_starting(test_config):
    """
    @test Verifies that motor inrush current spikes (15.5A) during STARTING (3)
    are gated and do NOT trigger false-positive anomalies.
    """
    detector = MotorAnomalyDetector(test_config)
    sequence = np.full((1, 10, 1), 12.5, dtype=np.float32)

    # 15.5A is > 6 standard deviations from mean (12.5A), which would normally trigger an anomaly
    residual, is_anomaly = detector.evaluate_frame(
        sequence=sequence,
        latest_value=15.5,
        state=3  # STARTING (Transitional state)
    )

    assert not is_anomaly, "Inrush spike in STARTING state must be gated and not flag as anomaly"


def test_anomaly_triggers_in_execute_state(test_config):
    """
    @test Verifies that high deviation spikes (19.5A) during EXECUTE (6) trigger an anomaly.
    """
    detector = MotorAnomalyDetector(test_config)
    sequence = np.full((1, 10, 1), 12.5, dtype=np.float32)

    # 19.5A during steady EXECUTE indicates physical stall/bearing failure
    residual, is_anomaly = detector.evaluate_frame(
        sequence=sequence,
        latest_value=19.5,
        state=6  # EXECUTE
    )

    assert is_anomaly, "Current spike in EXECUTE state must trigger anomaly"
    assert residual > 0.0


def test_pydantic_system_config():
    """
    @test Validates SystemConfig initialization and custom field loading.
    """
    cfg = SystemConfig(
        OPC_UA_ENDPOINT="opc.tcp://10.0.0.5:4840",
        nominal_current_mean=15.0,
        ANOMALY_Z_THRESHOLD=4.0
    )
    assert cfg.opc_ua_endpoint == "opc.tcp://10.0.0.5:4840"
    assert cfg.nominal_current_mean == 15.0
    assert cfg.anomaly_z_threshold == 4.0
