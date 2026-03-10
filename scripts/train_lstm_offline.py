#!/usr/bin/env python3
"""
train_lstm_offline.py
---------------------
Script huấn luyện ngoại tuyến (Offline Training) mô hình LSTM
sử dụng tập dữ liệu thu thập từ môi trường mô phỏng Gazebo + PX4 SITL.

Quy trình:
  1. Đọc dataset CSV (các chuỗi kế hoạch khớp + nhãn torque tương ứng)
  2. Xây dựng DataLoader với chuỗi trượt (Sliding Window)
  3. Huấn luyện LSTM với Adam + MSE Loss
  4. Lưu trọng số tốt nhất dạng .pth

Dataset CSV format:
  q1, q2, q3, q4, q5, q6, tau_x, tau_y, tau_z
  (mỗi dòng là 1 time step)
"""

import os
import argparse
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
from torch.optim.lr_scheduler import CosineAnnealingLR
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Tuple


# ============================================================
#  Kiến trúc LSTM (tương đồng với node inference)
# ============================================================
class LSTMTrajectoryPredictor(nn.Module):

    def __init__(self,
                 input_features: int = 6,
                 hidden_dim: int = 64,
                 num_layers: int = 2,
                 output_dim: int = 3,
                 dropout: float = 0.1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        self.lstm = nn.LSTM(
            input_features, hidden_dim, num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
        self.layer_norm = nn.LayerNorm(hidden_dim)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 32),
            nn.Tanh(),
            nn.Linear(32, output_dim)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out, _ = self.lstm(x)
        return self.fc(self.layer_norm(out[:, -1, :]))


# ============================================================
#  Dataset tùy chỉnh với Sliding Window
# ============================================================
class UAMTrajectoryDataset(Dataset):
    """
    Tạo cặp (chuỗi_khớp, torque_mục_tiêu) từ dữ liệu chuỗi thời gian.

    Với mỗi time step t:
      X[t] = [q(t - seq_len + 1), ..., q(t)]  → chuỗi đầu vào
      y[t] = tau(t + lookahead)                → nhãn mô-men xoắn tương lai
    """

    def __init__(self,
                 data: np.ndarray,
                 seq_len: int = 10,
                 lookahead: int = 1):
        """
        data     : numpy array [N, 9] = [q1..q6, tau_x, tau_y, tau_z]
        seq_len  : độ dài chuỗi đầu vào
        lookahead: bước nhìn trước (bao nhiêu step tương lai để dự đoán)
        """
        self.seq_len   = seq_len
        self.lookahead = lookahead

        joints = data[:, :6].astype(np.float32)  # [N, 6]
        torques= data[:, 6:].astype(np.float32)  # [N, 3]

        # Chuẩn hóa khớp về [-1, 1] (góc trong radians, thường ∈ [-π, π])
        self.joint_mean = joints.mean(axis=0)
        self.joint_std  = joints.std(axis=0) + 1e-8
        joints = (joints - self.joint_mean) / self.joint_std

        # Chuẩn hóa torque
        self.torque_mean = torques.mean(axis=0)
        self.torque_std  = torques.std(axis=0) + 1e-8
        torques = (torques - self.torque_mean) / self.torque_std

        self.joints  = torch.tensor(joints)
        self.torques = torch.tensor(torques)
        self.n_samples = len(joints) - seq_len - lookahead + 1

    def __len__(self) -> int:
        return self.n_samples

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        x = self.joints[idx : idx + self.seq_len]
        y = self.torques[idx + self.seq_len - 1 + self.lookahead]
        return x, y


# ============================================================
#  Hàm sinh dữ liệu giả lập (khi chưa có file CSV thực tế)
# ============================================================
def generate_synthetic_dataset(n_samples: int = 50000,
                                save_path: str = 'data/gazebo_dataset.csv'):
    """
    Tạo dataset mô phỏng đơn giản: cánh tay 6 khớp dao động ngẫu nhiên,
    torque được tính gần đúng bằng mô hình động lực học tuyến tính giản lược.
    """
    os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else '.', exist_ok=True)

    t       = np.linspace(0, n_samples * 0.01, n_samples)  # 10ms step
    freqs   = [0.3, 0.5, 0.7, 0.4, 0.6, 0.2]
    amps    = [0.8, 1.2, 0.6, 0.9, 0.5, 1.0]

    q = np.zeros((n_samples, 6))
    for i in range(6):
        q[:, i] = amps[i] * np.sin(2 * np.pi * freqs[i] * t + np.random.uniform(0, np.pi))

    dq  = np.gradient(q, axis=0) / 0.01
    ddq = np.gradient(dq, axis=0) / 0.01

    # Mô-men xoắn giả lập: tau ≈ M(q)*ddq + C(q,dq)*dq (giản lược)
    m_link = [0.3, 0.4, 0.3, 0.2, 0.15, 0.1]   # Khối lượng từng link [kg]
    l_link = [0.15, 0.13, 0.10, 0.08, 0.06, 0.05] # Chiều dài link [m]

    tau_x = sum(m_link[i] * l_link[i]**2 * ddq[:, i] for i in range(3))
    tau_y = sum(m_link[i] * l_link[i]**2 * ddq[:, i] * np.cos(q[:, i]) for i in range(3, 6))
    tau_z = 0.05 * dq[:, 0] * np.cos(q[:, 0]) + 0.03 * dq[:, 2]

    # Thêm nhiễu đo lường thực tế
    noise_scale = 0.02
    tau_x += np.random.normal(0, noise_scale, n_samples)
    tau_y += np.random.normal(0, noise_scale, n_samples)
    tau_z += np.random.normal(0, noise_scale * 0.5, n_samples)

    df = pd.DataFrame({
        'q1': q[:,0], 'q2': q[:,1], 'q3': q[:,2],
        'q4': q[:,3], 'q5': q[:,4], 'q6': q[:,5],
        'tau_x': tau_x, 'tau_y': tau_y, 'tau_z': tau_z
    })
    df.to_csv(save_path, index=False)
    print(f"Dataset tổng hợp đã lưu: {save_path} ({n_samples} mẫu)")
    return save_path


# ============================================================
#  Vòng lặp huấn luyện
# ============================================================
def train_model(args) -> None:

    # ── Thiết bị ──
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Thiết bị huấn luyện: {device}")

    # ── Đọc / tạo dataset ──
    if not os.path.exists(args.data_path):
        print(f"Không tìm thấy dataset. Đang tạo dataset tổng hợp...")
        args.data_path = generate_synthetic_dataset(
            n_samples=args.synthetic_samples,
            save_path=args.data_path
        )

    df   = pd.read_csv(args.data_path)
    data = df.values
    print(f"Đã tải dataset: {len(data)} mẫu | cột: {list(df.columns)}")

    # ── Khởi tạo Dataset & DataLoader ──
    full_dataset = UAMTrajectoryDataset(data, seq_len=args.seq_len, lookahead=args.lookahead)

    val_size   = int(0.15 * len(full_dataset))
    test_size  = int(0.10 * len(full_dataset))
    train_size = len(full_dataset) - val_size - test_size

    train_ds, val_ds, test_ds = random_split(
        full_dataset, [train_size, val_size, test_size],
        generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(train_ds, batch_size=args.batch_size,
                               shuffle=True,  num_workers=2, pin_memory=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size,
                               shuffle=False, num_workers=2)
    test_loader  = DataLoader(test_ds,  batch_size=args.batch_size,
                               shuffle=False, num_workers=2)

    print(f"Train: {train_size} | Val: {val_size} | Test: {test_size}")

    # ── Mô hình ──
    model = LSTMTrajectoryPredictor(
        input_features=6,
        hidden_dim=args.hidden_dim,
        num_layers=args.num_layers,
        output_dim=3,
        dropout=args.dropout
    ).to(device)

    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"Số tham số LSTM: {n_params:,}")

    # ── Hàm mất mát + Optimizer ──
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(),
                                  lr=args.lr, weight_decay=1e-5)
    scheduler = CosineAnnealingLR(optimizer, T_max=args.epochs, eta_min=1e-6)

    # ── Vòng lặp huấn luyện ──
    best_val_loss = float('inf')
    history = {'train': [], 'val': []}

    os.makedirs(os.path.dirname(args.model_out)
                if os.path.dirname(args.model_out) else '.', exist_ok=True)

    for epoch in range(1, args.epochs + 1):

        # --- Training phase ---
        model.train()
        train_losses = []
        for batch_x, batch_y in train_loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)
            optimizer.zero_grad()
            pred   = model(batch_x)
            loss   = criterion(pred, batch_y)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            train_losses.append(loss.item())

        # --- Validation phase ---
        model.eval()
        val_losses = []
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                batch_x, batch_y = batch_x.to(device), batch_y.to(device)
                pred = model(batch_x)
                val_losses.append(criterion(pred, batch_y).item())

        train_loss = np.mean(train_losses)
        val_loss   = np.mean(val_losses)
        scheduler.step()

        history['train'].append(train_loss)
        history['val'].append(val_loss)

        # Lưu mô hình tốt nhất
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), args.model_out)

        if epoch % 10 == 0 or epoch == 1:
            print(f"Epoch [{epoch:4d}/{args.epochs}] "
                  f"Train Loss: {train_loss:.6f} | "
                  f"Val Loss: {val_loss:.6f} | "
                  f"LR: {scheduler.get_last_lr()[0]:.2e}"
                  + (" ✓ BEST" if val_loss == best_val_loss else ""))

    # ── Test cuối cùng ──
    model.load_state_dict(torch.load(args.model_out, map_location=device))
    model.eval()
    test_preds, test_labels = [], []
    with torch.no_grad():
        for batch_x, batch_y in test_loader:
            pred = model(batch_x.to(device)).cpu().numpy()
            test_preds.append(pred)
            test_labels.append(batch_y.numpy())

    test_preds  = np.concatenate(test_preds)
    test_labels = np.concatenate(test_labels)
    test_mse    = np.mean((test_preds - test_labels) ** 2)
    test_rmse   = np.sqrt(test_mse)
    print(f"\n=== Kết quả Test ===")
    print(f"MSE  : {test_mse:.6f}")
    print(f"RMSE : {test_rmse:.6f}")
    print(f"Model đã lưu: {args.model_out}")

    # ── Vẽ đồ thị Loss ──
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.plot(history['train'], label='Train Loss')
    plt.plot(history['val'],   label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.title('LSTM Training History')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    n_plot = min(500, len(test_preds))
    plt.plot(test_labels[:n_plot, 0], label='τx thực tế', alpha=0.7)
    plt.plot(test_preds[:n_plot, 0],  label='τx dự đoán', alpha=0.7)
    plt.plot(test_labels[:n_plot, 1], label='τy thực tế', alpha=0.7)
    plt.plot(test_preds[:n_plot, 1],  label='τy dự đoán', alpha=0.7)
    plt.xlabel('Time step')
    plt.ylabel('Torque (chuẩn hóa)')
    plt.title('Dự đoán vs Thực tế')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plot_path = args.model_out.replace('.pth', '_training_curve.png')
    plt.savefig(plot_path, dpi=150)
    print(f"Đồ thị lưu tại: {plot_path}")


# ============================================================
#  Entry point
# ============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description='Huấn luyện LSTM dự báo mô-men xoắn cho UAM'
    )
    parser.add_argument('--data_path',         default='data/gazebo_dataset.csv')
    parser.add_argument('--model_out',         default='../models/lstm_uam_weights.pth')
    parser.add_argument('--epochs',            type=int,   default=100)
    parser.add_argument('--batch_size',        type=int,   default=64)
    parser.add_argument('--lr',                type=float, default=1e-3)
    parser.add_argument('--seq_len',           type=int,   default=10)
    parser.add_argument('--lookahead',         type=int,   default=1)
    parser.add_argument('--hidden_dim',        type=int,   default=64)
    parser.add_argument('--num_layers',        type=int,   default=2)
    parser.add_argument('--dropout',           type=float, default=0.1)
    parser.add_argument('--synthetic_samples', type=int,   default=80000)
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    train_model(args)
