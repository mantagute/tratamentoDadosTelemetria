"""
gerarDadosPerfeitos.py
======================
Gera uma sessão sintética "perfeita" para validar:
  1) plotagem do mapa/traçado
  2) tracking da posição no mapa base da primeira volta
  3) consistência do dead reckoning em cenário sem ruído

Saída:
  data/processed/sintetico-perfeito/
    - ACT_SPEED_A13.csv
    - ACT_SPEED_B13.csv
    - VENTOR_LINEAR_ACC_X.csv
    - VENTOR_ANGULAR_SPEED_Z.csv
    - REF_TRAJETORIA_X.csv
    - REF_TRAJETORIA_Y.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import pandas as pd


DIR_BASE = Path(__file__).resolve().parent.parent
DIR_PROCESSADO = DIR_BASE / "data" / "processed"

# Mesmas constantes do getTrajetoria.py para manter coerência perfeita.
RAIO_RODA_POLEGADAS = 10.0
METROS_POR_POLEGADA = 0.0254
RAIO_RODA_M = RAIO_RODA_POLEGADAS * METROS_POR_POLEGADA
CIRCUNFERENCIA_RODA_M = 2.0 * np.pi * RAIO_RODA_M
FATOR_REDUCAO_PLANETARIA = 11.72
RPM_MOTOR_PARA_MPS = CIRCUNFERENCIA_RODA_M / (60.0 * FATOR_REDUCAO_PLANETARIA)
MPS_PARA_RPM_MOTOR = 1.0 / RPM_MOTOR_PARA_MPS


def montar_df(nome: str, t: np.ndarray, valores: np.ndarray, unidade: str, can_id: str, prioridade: int) -> pd.DataFrame:
    linhas = []
    for ti, vi in zip(t, valores):
        linhas.append(
            {
                "names": nome,
                "timestamp": float(round(ti, 6)),
                "id_can": can_id,
                "prioridade": int(prioridade),
                "dado": f"{vi:.4f} {unidade}",
            }
        )
    return pd.DataFrame(linhas)


def main() -> None:
    parser = argparse.ArgumentParser(description="Gera dados sintéticos perfeitos para validar mapa e tracking.")
    parser.add_argument("--saida", default="sintetico-perfeito", help="Nome da pasta em data/processed/")
    parser.add_argument("--fs", type=float, default=50.0, help="Frequência de amostragem (Hz). Padrão: 50.")
    parser.add_argument("--voltas", type=int, default=4, help="Quantidade de voltas sintéticas. Padrão: 4.")
    parser.add_argument("--lap-sec", type=float, default=40.0, help="Período de volta (s). Padrão: 40.")
    parser.add_argument("--raio-m", type=float, default=25.0, help="Raio da pista circular (m). Padrão: 25.")
    parser.add_argument(
        "--vel-media",
        type=float,
        default=None,
        help="Velocidade média (m/s). Padrão: derivada de lap-sec e raio (coerência física).",
    )
    parser.add_argument("--amp-vel", type=float, default=0.0, help="Amplitude da oscilação de velocidade (m/s).")
    parser.add_argument("--repouso-sec", type=float, default=2.0, help="Repouso no início/fim (s).")
    args = parser.parse_args()

    dt = 1.0 / args.fs
    t_mov = np.arange(0.0, args.voltas * args.lap_sec, dt)
    omega_lap = 2.0 * np.pi / args.lap_sec

    # Garante coerência física entre período de volta e raio quando não houver override.
    vel_media = args.vel_media
    if vel_media is None:
        vel_media = (2.0 * np.pi * args.raio_m) / args.lap_sec

    # Velocidade periódica por volta para facilitar detecção de período.
    vel = vel_media + args.amp_vel * np.sin(omega_lap * t_mov)
    vel = np.clip(vel, 0.5, None)
    acc_x = args.amp_vel * omega_lap * np.cos(omega_lap * t_mov)
    yaw = vel / args.raio_m  # cinemática circular ideal

    # Heading e trajetória de referência ideais.
    theta = np.zeros_like(t_mov)
    x = np.zeros_like(t_mov)
    y = np.zeros_like(t_mov)
    for i in range(1, len(t_mov)):
        theta[i] = theta[i - 1] + 0.5 * (yaw[i] + yaw[i - 1]) * dt
        vx_i = vel[i] * np.cos(theta[i])
        vy_i = vel[i] * np.sin(theta[i])
        vx_prev = vel[i - 1] * np.cos(theta[i - 1])
        vy_prev = vel[i - 1] * np.sin(theta[i - 1])
        x[i] = x[i - 1] + 0.5 * (vx_i + vx_prev) * dt
        y[i] = y[i - 1] + 0.5 * (vy_i + vy_prev) * dt

    # Inclui repouso inicial/final para validar bias por bordas, se necessário.
    n_rep = int(round(args.repouso_sec * args.fs))
    t0 = 946_684_800.0  # época fixa e reproduzível

    t_pre = t0 + np.arange(n_rep) * dt
    t_core = t_pre[-1] + dt + t_mov
    t_pos = t_core[-1] + dt + np.arange(n_rep) * dt
    t = np.concatenate([t_pre, t_core, t_pos])

    zeros = np.zeros(n_rep)
    vel_full = np.concatenate([zeros, vel, zeros])
    acc_full = np.concatenate([zeros, acc_x, zeros])
    yaw_full = np.concatenate([zeros, yaw, zeros])
    x_full = np.concatenate([np.full(n_rep, x[0]), x, np.full(n_rep, x[-1])])
    y_full = np.concatenate([np.full(n_rep, y[0]), y, np.full(n_rep, y[-1])])

    rpm = vel_full * MPS_PARA_RPM_MOTOR

    pasta_saida = DIR_PROCESSADO / args.saida
    pasta_saida.mkdir(parents=True, exist_ok=True)

    df_a13 = montar_df("ACT_SPEED_A13", t, rpm, "rpm", "0x18FF01F7", 1)
    df_b13 = montar_df("ACT_SPEED_B13", t, rpm, "rpm", "0x18FF02F7", 1)
    df_acc = montar_df("VENTOR_LINEAR_ACC_X", t, acc_full, "m/s²", "0x00000001", 1)
    df_yaw = montar_df("VENTOR_ANGULAR_SPEED_Z", t, yaw_full, "rad/s", "0x00000002", 1)
    df_rx = montar_df("REF_TRAJETORIA_X", t, x_full, "m", "0x00000002", 1)
    df_ry = montar_df("REF_TRAJETORIA_Y", t, y_full, "m", "0x00000002", 1)

    df_a13.to_csv(pasta_saida / "ACT_SPEED_A13.csv", index=False)
    df_b13.to_csv(pasta_saida / "ACT_SPEED_B13.csv", index=False)
    df_acc.to_csv(pasta_saida / "VENTOR_LINEAR_ACC_X.csv", index=False)
    df_yaw.to_csv(pasta_saida / "VENTOR_ANGULAR_SPEED_Z.csv", index=False)
    df_rx.to_csv(pasta_saida / "REF_TRAJETORIA_X.csv", index=False)
    df_ry.to_csv(pasta_saida / "REF_TRAJETORIA_Y.csv", index=False)

    print("=" * 60)
    print("DADOS SINTÉTICOS PERFEITOS GERADOS")
    print("=" * 60)
    print(f"Pasta: {pasta_saida}")
    print(f"Amostras: {len(t)} | fs={args.fs:.1f}Hz | voltas={args.voltas} | lap={args.lap_sec:.1f}s")
    print(f"Raio: {args.raio_m:.1f}m | vel média: {vel_media:.2f}m/s | amp vel: {args.amp_vel:.2f}m/s")
    print("Arquivos: ACT_SPEED_A13/B13, VENTOR_LINEAR_ACC_X, VENTOR_ANGULAR_SPEED_Z, REF_TRAJETORIA_X/Y")


if __name__ == "__main__":
    main()
