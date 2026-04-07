"""
plotador.py
-----------
Para cada pasta em data/processed/, plota cada sinal em um gráfico
individual e salva em data/processed/<arquivo>/plots/<SINAL>.png

Eixo X: tempo relativo (t - t0) em segundos, calculado a partir
        do primeiro timestamp de cada arquivo.

Sinais inválidos (arquivo .invalid gerado pelo extrator) recebem um
gráfico de "cartão vermelho" explicitando o motivo da rejeição, em vez
de serem silenciosamente omitidos.

Uso:
  python3 src/plotador.py                  # todos os arquivos
  python3 src/plotador.py session_0055     # filtrar por nome
"""

import sys
import textwrap
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path

# ── Caminhos ──────────────────────────────────────────────────────────────────

BASE_DIR = Path(__file__).resolve().parent.parent
PROC_DIR = BASE_DIR / "data" / "processed"

# ── Metadados visuais por sinal ───────────────────────────────────────────────

META = {
    "ACT_SPEED_A13":    ("#58a6ff", "Velocidade real — Motor A13"),
    "ACT_TORQUE_A13":   ("#3fb950", "Torque real — Motor A13"),
    "ACT_POWER_A13":    ("#ffa657", "Potência real — Motor A13"),
    "ACT_TEMP_A13":     ("#f78166", "Temperatura — Motor A13"),
    "ACT_SPEED_B13":    ("#79c0ff", "Velocidade real — Motor B13"),
    "ACT_TORQUE_B13":   ("#56d364", "Torque real — Motor B13"),
    "ACT_POWER_B13":    ("#ffb77a", "Potência real — Motor B13"),
    "ACT_TEMP_B13":     ("#ffa198", "Temperatura — Motor B13"),
    "SETP_TORQUE_13A":  ("#58a6ff", "Setpoint Torque — 13A"),
    "SETP_RPM_13A":     ("#3fb950", "Setpoint RPM — 13A"),
    "SETP_TORQUE_13B":  ("#ff7b72", "Setpoint Torque — 13B"),
    "SETP_RPM_13B":     ("#ffa657", "Setpoint RPM — 13B"),
    "SETP_TORQUE_A13":  ("#c9a0ff", "Setpoint Torque — Motor A13"),
    "SETP_TORQUE_B13":  ("#d4a0ff", "Setpoint Torque — Motor B13"),
}
DEFAULT_COLOR = "#8b949e"

# ── Estilo ────────────────────────────────────────────────────────────────────

DARK_BG   = "#0d1117"
PANEL_BG  = "#161b22"
GRID_COL  = "#21262d"
TEXT_COL  = "#e6edf3"
MUTED     = "#8b949e"
ERR_COLOR = "#da3633"
WARN_COL  = "#d29922"

plt.rcParams.update({
    "figure.facecolor": DARK_BG,
    "axes.facecolor":   PANEL_BG,
    "axes.edgecolor":   GRID_COL,
    "axes.labelcolor":  TEXT_COL,
    "axes.titlecolor":  TEXT_COL,
    "axes.grid":        True,
    "grid.color":       GRID_COL,
    "grid.linewidth":   0.5,
    "xtick.color":      MUTED,
    "ytick.color":      MUTED,
    "text.color":       TEXT_COL,
    "font.family":      "monospace",
})

# ── Plot: sinal inválido ──────────────────────────────────────────────────────

def plot_invalid(invalid_path: Path, plots_dir: Path) -> None:
    """Gera um cartão de erro para sinais marcados como inválidos pelo extrator."""
    sig     = invalid_path.stem
    _, label = META.get(sig, (DEFAULT_COLOR, sig))
    reason  = invalid_path.read_text(encoding="utf-8")

    fig, ax = plt.subplots(figsize=(12, 3.6))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_title(label, fontsize=10, fontweight="bold", pad=7, color=ERR_COLOR)

    ax.text(0.5, 0.72, "⚠  SINAL INVÁLIDO — NÃO PLOTADO",
            transform=ax.transAxes, fontsize=10, fontweight="bold",
            va="center", ha="center", color=ERR_COLOR)

    wrapped = textwrap.fill(reason, width=100)
    ax.text(0.5, 0.38, wrapped,
            transform=ax.transAxes, fontsize=7,
            va="center", ha="center", color=MUTED,
            bbox=dict(boxstyle="round,pad=0.5", facecolor=DARK_BG,
                      alpha=0.8, edgecolor=ERR_COLOR))

    fig.tight_layout(pad=1.4)
    out = plots_dir / f"{sig}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK_BG)
    plt.close(fig)
    print(f"    [INVÁLIDO] {sig:<22}  →  plots/{sig}.png  (cartão de erro gerado)")

# ── Plot: sinal válido ────────────────────────────────────────────────────────

def plot_signal(csv_path: Path, plots_dir: Path) -> None:
    sig = csv_path.stem
    df  = pd.read_csv(csv_path)

    if df.empty:
        print(f"    [skip] {sig} — sem dados")
        return

    df["valor"]   = df["dado"].str.extract(r"([-\d.]+)", expand=False).astype(float)
    df["unidade"] = df["dado"].str.extract(r"[-\d.]+ (\S+)", expand=False).fillna("")
    df = df.dropna(subset=["valor"])

    if len(df) < 2:
        print(f"    [skip] {sig} — amostras insuficientes")
        return

    t0   = df["timestamp"].iloc[0]
    t    = (df["timestamp"] - t0).to_numpy()   # ← conversão para numpy
    y    = df["valor"].to_numpy()               # ← conversão para numpy
    unit = df["unidade"].iloc[0]
    dur  = t[-1]
    freq = len(df) / dur if dur > 0 else 0

    color, label = META.get(sig, (DEFAULT_COLOR, sig))

    fig, ax = plt.subplots(figsize=(12, 3.6))
    ax.plot(t, y, color=color, linewidth=1.2, alpha=0.92)
    ax.fill_between(t, y, alpha=0.07, color=color)

    ax.set_title(label, fontsize=10, fontweight="bold", pad=7)
    ax.set_xlabel("Tempo relativo (s)", fontsize=8, labelpad=4)
    ax.set_ylabel(unit, fontsize=8)
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.0f}s"))
    ax.tick_params(labelsize=7)
    ax.spines[["top", "right"]].set_visible(False)

    stats = (f"n={len(df)}  |  {freq:.1f} Hz\n"
             f"min={y.min():.2f}  max={y.max():.2f}  μ={y.mean():.2f} {unit}")
    ax.text(0.99, 0.97, stats, transform=ax.transAxes, fontsize=6.5,
            va="top", ha="right", color=MUTED,
            bbox=dict(boxstyle="round,pad=0.4", facecolor=DARK_BG,
                      alpha=0.7, edgecolor=GRID_COL))

    fig.tight_layout(pad=1.4)
    out = plots_dir / f"{sig}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=DARK_BG)
    plt.close(fig)
    print(f"    {sig:<22}  {len(df)} pts  |  {dur:.1f}s  |  {freq:.1f}Hz  →  plots/{sig}.png")

# ── Processa pasta ────────────────────────────────────────────────────────────

def process_folder(folder: Path) -> None:
    csvs = sorted(folder.glob("*.csv"))
    invs = sorted(folder.glob("*.invalid"))

    if not csvs and not invs:
        return

    plots_dir = folder / "plots"
    plots_dir.mkdir(exist_ok=True)

    print(f"\n[{folder.name}]  {len(csvs)} sinal(is) válido(s)  |  {len(invs)} inválido(s)")

    for inv in invs:
        plot_invalid(inv, plots_dir)

    for csv in csvs:
        plot_signal(csv, plots_dir)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  PLOTADOR DE TELEMETRIA CAN")
    print("=" * 60)

    filtro = sys.argv[1].strip() if len(sys.argv) > 1 else None

    if not PROC_DIR.exists():
        print("\nNenhum dado encontrado. Execute o extrator.py primeiro.\n")
        return

    folders = sorted(
        d for d in PROC_DIR.iterdir()
        if d.is_dir() and (filtro is None or filtro in d.name)
    )

    if not folders:
        print("\nNenhum dado encontrado. Execute extrator.py primeiro.\n")
        return

    for folder in folders:
        process_folder(folder)

    print("\nPronto. Gráficos em data/processed/<arquivo>/plots/\n")
    print("Sinais inválidos aparecem como cartão de erro no mesmo diretório.\n")


if __name__ == "__main__":
    main()