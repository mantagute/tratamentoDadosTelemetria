"""
velo.py
-------
Recebe um CSV de aceleração no formato padrão do extrator e gera um CSV
de velocidade no mesmo formato, pronto para o plotador.py.

Uso:
  python3 src/velo.py <caminho/para/SINAL_ACC.csv>

  ex: python3 src/velo.py data/processed/candump-1999-12-31_230113/VENTOR_LINEAR_ACC_Y.csv

Saída (mesma pasta do CSV de entrada):
  <SINAL_VEL>.csv      — velocidade integrada (m/s)

Convenção de nomes:
  VENTOR_LINEAR_ACC_X  →  VENTOR_LINEAR_VEL_X
  VENTOR_LINEAR_ACC_Y  →  VENTOR_LINEAR_VEL_Y
  qualquer outro nome  →  <NOME>_VEL

Pipeline interno:
  1. Carrega CSV e extrai valores numéricos da coluna 'dado'
  2. Estima e remove bias (média dos 5% menores |acc|)
  3. Aplica filtro Butterworth passa-baixa 4ª ordem (cutoff 3 Hz)
  4. Integração trapezoidal → velocidade
  5. Remoção de drift linear residual na velocidade
  6. Salva CSV no formato padrão (names, timestamp, id_can, prioridade, dado)
"""

import sys
import re
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import butter, filtfilt

# ── Helpers ───────────────────────────────────────────────────────────────────

def extrair_numero(texto: str) -> float | None:
    match = re.search(r"-?\d+\.?\d*", str(texto))
    return float(match.group()) if match else None


def extrair_unidade(texto: str) -> str:
    """Extrai a unidade do campo 'dado', ex: '-0.29 m/s²' → 'm/s²'."""
    parts = str(texto).strip().split()
    return parts[1] if len(parts) > 1 else ""


def derivar_nome_vel(sig_acc: str) -> str:
    """
    A partir do nome do sinal de aceleração, deriva o nome de velocidade.
    ex: VENTOR_LINEAR_ACC_Y → VENTOR_LINEAR_VEL_Y
    """
    upper = sig_acc.upper()
    if "_ACC_" in upper:
        suffix = upper.split("_ACC_")[-1]
        return f"VENTOR_LINEAR_VEL_{suffix}"
    return f"{upper}_VEL"


def to_row(name: str, ts: float, can_id: str,
           prio: int, val: float, unit: str) -> dict:
    return {
        "names":      name,
        "timestamp":  round(ts, 6),
        "id_can":     can_id,
        "prioridade": prio,
        "dado":       f"{val:.4f} {unit}",
    }

# ── Pipeline ──────────────────────────────────────────────────────────────────

def processar(csv_path: Path) -> None:
    print(f"\n[velo] {csv_path.name}")

    # 1. Carregar
    df = pd.read_csv(csv_path)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["acc"]       = df["dado"].apply(extrair_numero)
    df = df.dropna(subset=["timestamp", "acc"]).sort_values("timestamp").reset_index(drop=True)

    if len(df) < 10:
        print("  [skip] amostras insuficientes.")
        return

    # metadados do sinal de origem para propagar nos CSVs de saída
    sig_acc  = str(df["names"].iloc[0])
    can_id   = str(df["id_can"].iloc[0])
    prio     = int(df["prioridade"].iloc[0])
    unit_acc = extrair_unidade(df["dado"].iloc[0])

    sig_vel = derivar_nome_vel(sig_acc)

    # 2. Bias
    abs_acc   = df["acc"].abs()
    threshold = np.percentile(abs_acc, 5)
    bias      = df.loc[abs_acc < threshold, "acc"].mean()
    df["acc_corr"] = df["acc"] - bias
    print(f"  Bias estimado : {bias:.6f} {unit_acc}")

    # 3. Filtro Butterworth passa-baixa
    dt     = df["timestamp"].diff().mean()
    fs     = 1.0 / dt
    cutoff = 3.0   # Hz — dinâmica longitudinal/lateral típica
    b, a   = butter(4, cutoff / (0.5 * fs), btype="low")
    df["acc_filt"] = filtfilt(b, a, df["acc_corr"])
    print(f"  Fs detectada  : {fs:.2f} Hz  |  cutoff filtro: {cutoff} Hz")

    # 4. Integração trapezoidal → velocidade
    vel = np.zeros(len(df))
    for i in range(1, len(df)):
        dt_i   = df["timestamp"][i] - df["timestamp"][i - 1]
        vel[i] = vel[i - 1] + 0.5 * (df["acc_filt"][i] + df["acc_filt"][i - 1]) * dt_i
    df["vel"] = vel

    # 5. Remoção de drift linear residual
    df["vel"] = df["vel"] - df["vel"].mean()
    df["vel"] = df["vel"] - df["vel"].iloc[0]
    drift     = np.linspace(0, df["vel"].iloc[-1], len(df))
    df["vel"] = df["vel"] - drift

    print(f"  Velocidade final   : {df['vel'].iloc[-1]:.4f} m/s")

    # 6. Salvar CSV no formato padrão
    out_dir = csv_path.parent
    ts_arr  = df["timestamp"].to_numpy()
    valores = df["vel"].to_numpy()

    rows = [to_row(sig_vel, ts_arr[i], can_id, prio, valores[i], "m/s")
            for i in range(len(df))]
    
    out_path = out_dir / f"{sig_vel}.csv"
    pd.DataFrame(rows).to_csv(out_path, index=False)
    print(f"  → {out_path.name}")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  INTEGRADOR DE VELOCIDADE")
    print("=" * 60)

    if len(sys.argv) < 2:
        print("\nUso: python3 velo.py <caminho/para/SINAL_ACC.csv> [...]")
        print("\nExemplo:")
        print("  python3 src/velo.py data/processed/candump-1999-12-31_230113/VENTOR_LINEAR_ACC_Y.csv")
        print("\nVários arquivos de uma vez:")
        print("  python3 src/velo.py data/processed/candump-xyz/VENTOR_LINEAR_ACC_*.csv")
        return

    for arg in sys.argv[1:]:
        p = Path(arg)
        lista_arquivos = [p] if p.exists() else sorted(Path().glob(arg))
        for fp in lista_arquivos:
            if not fp.exists():
                print(f"\n[ERRO] Arquivo não encontrado: {fp}")
                continue
            processar(fp)

    print("\nPronto.\n")


if __name__ == "__main__":
    main()