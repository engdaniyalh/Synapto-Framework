"""
@module config
@brief Configuration management for the Synapto-Framework AI Core.

Leverages Pydantic Settings to load and validate system configurations from
environment variables, external config files, and fallback defaults. Essential
for dynamic tuning without modifying source code.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field


class SystemConfig(BaseSettings):
    """
    @class SystemConfig
    @brief Validated data structure for AI environment and pipeline settings.
    """

    opc_ua_endpoint: str = Field(
        default="opc.tcp://127.0.0.1:4840",
        validation_alias="OPC_UA_ENDPOINT",
        description="Endpoint URL for the open62541 OPC UA server."
    )
    onnx_model_path: str = Field(
        default="models/motor_lstm.onnx",
        validation_alias="ONNX_MODEL_PATH",
        description="Local filepath to the exported ONNX model binary."
    )

    nominal_current_mean: float = Field(
        default=12.5,
        description="Baseline expected operational motor current (Amps)."
    )
    nominal_current_std: float = Field(
        default=0.45,
        description="Expected standard deviation for motor current fluctuations."
    )
    anomaly_z_threshold: float = Field(
        default=3.0,
        validation_alias="ANOMALY_Z_THRESHOLD",
        description="Z-score threshold which flags data as a significant anomaly."
    )
    residual_threshold: float = Field(
        default=2.0,
        description="Residual difference threshold between ONNX model prediction and actual telemetry."
    )

    sequence_length: int = Field(
        default=10,
        validation_alias="WINDOW_SIZE",
        description="Number of sequential samples required to execute a single inference frame."
    )

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore"
    )


# Singleton configuration instance loaded upon import
config = SystemConfig()
