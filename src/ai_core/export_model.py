"""
@module export_model
@brief Utility script to train and export PyTorch models into optimized ONNX graphs.

Pre-compiles the deep learning PyTorch models into the ONNX format, allowing
for lightweight, performant, framework-agnostic edge execution via ONNX Runtime.
"""

import os
import torch
import torch.nn as nn
import numpy as np


class MotorLSTM(nn.Module):
    """
    @class MotorLSTM
    @brief Time-series deep learning model for industrial motor anomaly detection.
    """

    def __init__(self, input_dim: int = 1, hidden_dim: int = 64, num_layers: int = 2):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )
        # Initialize final linear layer bias to nominal 12.5A for immediate stable baseline
        nn.init.constant_(self.fc[-1].bias, 12.5)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out, _ = self.lstm(x)
        return self.fc(out[:, -1, :])


def train_baseline_model(epochs: int = 3, seq_len: int = 10) -> MotorLSTM:
    """
    @brief Trains a baseline LSTM on synthetic nominal motor telemetry (12.5A).
    """
    torch.manual_seed(42)
    np.random.seed(42)

    model = MotorLSTM()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
    loss_fn = nn.MSELoss()

    # Generate synthetic nominal telemetry (12.5A with 0.4A gaussian noise)
    data = np.random.normal(12.5, 0.4, 1000).astype(np.float32)
    X, y = [], []
    for i in range(len(data) - seq_len):
        X.append(data[i:i + seq_len])
        y.append(data[i + seq_len])

    tensor_x = torch.tensor(np.array(X), dtype=torch.float32).unsqueeze(-1)
    tensor_y = torch.tensor(np.array(y), dtype=torch.float32).unsqueeze(-1)

    dataset = torch.utils.data.TensorDataset(tensor_x, tensor_y)
    loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=True)

    model.train()
    for _ in range(epochs):
        for batch_x, batch_y in loader:
            optimizer.zero_grad()
            pred = model(batch_x)
            loss = loss_fn(pred, batch_y)
            loss.backward()
            optimizer.step()

    model.eval()
    return model


def export_to_onnx(output_path: str = "models/motor_lstm.onnx", model: MotorLSTM = None):
    """
    @brief Compiles and writes the ONNX binary for edge deployment.

    @param output_path Destination path for the exported .onnx file.
    @param model Optional pre-trained MotorLSTM instance. If None, trains baseline.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    if model is None:
        model = train_baseline_model()
    model.eval()

    # Generate dummy input to trace the dynamic computation graph
    dummy_input = torch.randn(1, 10, 1, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=["input_sequence"],
        output_names=["predicted_current"],
        dynamic_axes={
            "input_sequence": {0: "batch_size"},
            "predicted_current": {0: "batch_size"}
        }
    )
    print(f"[Synapto-MLOps] Successfully exported ONNX model to {output_path}")


if __name__ == "__main__":
    export_to_onnx()
