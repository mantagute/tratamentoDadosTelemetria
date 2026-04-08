"""
plotador.py
===========
Visualizador de telemetria CAN — gera gráficos individuais por sinal.

VISÃO GERAL
-----------
Varre o diretório data/processed/ em busca de CSVs e arquivos .invalid
gerados pelos extratores e pelo integrador de velocidade. Para cada sinal
encontrado, gera um gráfico .png salvo em data/processed/<pasta>/plots/.

DOIS TIPOS DE GRÁFICO
---------------------
  1. Sinal válido (CSV)
     Plota a série temporal com tempo relativo no eixo X (t - t0), onde t0
     é o timestamp da primeira amostra. Exibe estatísticas de n, frequência,
     mínimo, máximo e média no canto superior direito.

  2. Sinal inválido (.invalid)
     Gera um "cartão de erro" — painel vermelho com o conteúdo do arquivo
     .invalid — para que o problema fique explícito no relatório visual,
     em vez de o sinal ser silenciosamente omitido.

ESTILO VISUAL
-------------
Tema escuro inspirado no GitHub Dark, com cores diferenciadas por tipo de sinal:
  - Azul claro    → velocidades e RPM
  - Verde         → torques
  - Laranja       → potência
  - Vermelho/rosa → temperatura
  - Amarelo       → pedal de acelerador

USO
---
  python3 src/plotador.py                    # plota todos os arquivos em processed/
  python3 src/plotador.py session_0055       # filtra por nome de pasta
  python3 src/plotador.py candump-1999-12-31 # filtra por nome de pasta
"""

import sys
import textwrap
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path


# ── Caminhos ──────────────────────────────────────────────────────────────────

DIR_BASE     = Path(__file__).resolve().parent.parent
DIR_PROCESSADO = DIR_BASE / "data" / "processed"


# ── Metadados visuais por sinal ───────────────────────────────────────────────
#
# Mapeamento: nome_sinal → (cor_hex, titulo_legivel, unidade_exibida)
#
# A unidade aqui é usada apenas como fallback; o plotador extrai a unidade
# real diretamente do campo 'dado' no CSV.

METADADOS_SINAIS = {
    # ── Inversores de tração — Motor A13 ─────────────────────────────────────
    "ACT_SPEED_A13":          ("#58a6ff", "Velocidade real — Motor A13",           "rpm"),
    "ACT_TORQUE_A13":         ("#3fb950", "Torque real — Motor A13",               "Nm"),
    "ACT_POWER_A13":          ("#ffa657", "Potência real — Motor A13",             "kW"),
    "ACT_TEMP_A13":           ("#f78166", "Temperatura — Motor A13",               "°C"),
    # ── Inversores de tração — Motor B13 ─────────────────────────────────────
    "ACT_SPEED_B13":          ("#79c0ff", "Velocidade real — Motor B13",           "rpm"),
    "ACT_TORQUE_B13":         ("#56d364", "Torque real — Motor B13",               "Nm"),
    "ACT_POWER_B13":          ("#ffb77a", "Potência real — Motor B13",             "kW"),
    "ACT_TEMP_B13":           ("#ffa198", "Temperatura — Motor B13",               "°C"),
    # ── Setpoints de torque (VCU → inversores) ────────────────────────────────
    "SETP_TORQUE_A13":        ("#c9a0ff", "Setpoint Torque — Motor A13",           "Nm"),
    "SETP_TORQUE_B13":        ("#d4a0ff", "Setpoint Torque — Motor B13",           "Nm"),
    # ── IMU — Aceleração linear (candump) ─────────────────────────────────────
    "VENTOR_LINEAR_ACC_X":    ("#58a6ff", "Aceleração Linear X (lateral)",         "m/s²"),
    "VENTOR_LINEAR_ACC_Y":    ("#3fb950", "Aceleração Linear Y (longitudinal)",    "m/s²"),
    "VENTOR_LINEAR_ACC_Z":    ("#ffa657", "Aceleração Linear Z (vertical)",        "m/s²"),
    # ── IMU — Velocidade angular (candump) ────────────────────────────────────
    "VENTOR_ANGULAR_SPEED_X": ("#f78166", "Velocidade Angular X",                  "rad/s"),
    "VENTOR_ANGULAR_SPEED_Y": ("#ff7b72", "Velocidade Angular Y",                  "rad/s"),
    "VENTOR_ANGULAR_SPEED_Z": ("#ffa657", "Velocidade Angular Z",                  "rad/s"),
    # ── Velocidade integrada (getVelocidade.py) ───────────────────────────────
    "VENTOR_LINEAR_VEL_X":    ("#79c0ff", "Velocidade Integrada X (lateral)",      "m/s"),
    "VENTOR_LINEAR_VEL_Y":    ("#56d364", "Velocidade Integrada Y (longitudinal)", "m/s"),
    # ── VCU — Pedal de acelerador (candump) ──────────────────────────────────
    "APS_PERC":               ("#e3b341", "Pedal de Acelerador (APS)",             "%"),
}

# Cor usada para sinais sem entrada no dicionário acima
COR_PADRAO = "#8b949e"


# ── Paleta de cores (tema GitHub Dark) ────────────────────────────────────────

COR_FUNDO_FIGURA  = "#0d1117"   # fundo geral da figura
COR_FUNDO_PAINEL  = "#161b22"   # fundo da área de plot
COR_GRADE         = "#21262d"   # linhas de grade e bordas dos eixos
COR_TEXTO         = "#e6edf3"   # texto dos eixos e títulos
COR_TEXTO_SUAVE   = "#8b949e"   # texto secundário (estatísticas, ticks)
COR_ERRO          = "#da3633"   # vermelho para cartões de sinal inválido

plt.rcParams.update({
    "figure.facecolor": COR_FUNDO_FIGURA,
    "axes.facecolor":   COR_FUNDO_PAINEL,
    "axes.edgecolor":   COR_GRADE,
    "axes.labelcolor":  COR_TEXTO,
    "axes.titlecolor":  COR_TEXTO,
    "axes.grid":        True,
    "grid.color":       COR_GRADE,
    "grid.linewidth":   0.5,
    "xtick.color":      COR_TEXTO_SUAVE,
    "ytick.color":      COR_TEXTO_SUAVE,
    "text.color":       COR_TEXTO,
    "font.family":      "monospace",
})


# ── Plotagem de sinal inválido ────────────────────────────────────────────────

def plotar_sinal_invalido(caminho_invalid: Path, diretorio_plots: Path) -> None:
    """
    Gera um "cartão de erro" para um sinal marcado como inválido pelo extrator.

    Em vez de simplesmente ignorar sinais inválidos, este gráfico torna o
    problema visível no relatório visual, exibindo o diagnóstico do arquivo
    .invalid e alertando o operador com destaque em vermelho.
    """
    nome_sinal = caminho_invalid.stem
    meta = METADADOS_SINAIS.get(nome_sinal)
    titulo = meta[1] if meta else nome_sinal
    conteudo_diagnostico = caminho_invalid.read_text(encoding="utf-8")

    fig, ax = plt.subplots(figsize=(12, 3.6))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_title(titulo, fontsize=10, fontweight="bold", pad=7, color=COR_ERRO)

    # Mensagem principal de alerta
    ax.text(
        0.5, 0.72,
        "⚠  SINAL INVÁLIDO — NÃO PLOTADO",
        transform=ax.transAxes,
        fontsize=10, fontweight="bold",
        va="center", ha="center",
        color=COR_ERRO,
    )

    # Diagnóstico detalhado em caixa de texto
    texto_formatado = textwrap.fill(conteudo_diagnostico, width=100)
    ax.text(
        0.5, 0.38,
        texto_formatado,
        transform=ax.transAxes,
        fontsize=7,
        va="center", ha="center",
        color=COR_TEXTO_SUAVE,
        bbox=dict(
            boxstyle="round,pad=0.5",
            facecolor=COR_FUNDO_FIGURA,
            alpha=0.8,
            edgecolor=COR_ERRO,
        ),
    )

    fig.tight_layout(pad=1.4)
    caminho_saida = diretorio_plots / f"{nome_sinal}.png"
    fig.savefig(caminho_saida, dpi=150, bbox_inches="tight", facecolor=COR_FUNDO_FIGURA)
    plt.close(fig)
    print(f"    [INVÁLIDO] {nome_sinal:<26}  →  plots/{nome_sinal}.png  (cartão de erro gerado)")


# ── Plotagem de sinal válido ──────────────────────────────────────────────────

def plotar_sinal(caminho_csv: Path, diretorio_plots: Path) -> None:
    """
    Gera o gráfico de série temporal para um sinal válido.

    Eixo X: tempo relativo em segundos (t - t0), onde t0 é o timestamp
    da primeira amostra. Isso facilita a comparação entre sessões com
    timestamps absolutos diferentes.

    Exibe no canto superior direito:
      - n (número de amostras)
      - frequência estimada (Hz)
      - mínimo, máximo e média do sinal
    """
    nome_sinal = caminho_csv.stem
    df = pd.read_csv(caminho_csv)

    if df.empty:
        print(f"    [skip] {nome_sinal} — CSV vazio")
        return

    # Extrai valor numérico e unidade do campo 'dado' (ex: '-0.29 m/s²')
    df["valor"]   = df["dado"].str.extract(r"([-\d.]+)", expand=False).astype(float)
    df["unidade"] = df["dado"].str.extract(r"[-\d.]+ (\S+)", expand=False).fillna("")
    df = df.dropna(subset=["valor"])

    if len(df) < 2:
        print(f"    [skip] {nome_sinal} — amostras insuficientes para plotar")
        return

    # Calcula tempo relativo a partir do primeiro timestamp
    t0             = df["timestamp"].iloc[0]
    tempo_relativo = (df["timestamp"] - t0).to_numpy()
    valores        = df["valor"].to_numpy()
    unidade        = df["unidade"].iloc[0]
    duracao_total  = tempo_relativo[-1]
    frequencia_hz  = len(df) / duracao_total if duracao_total > 0 else 0

    # Busca metadados visuais do sinal
    meta  = METADADOS_SINAIS.get(nome_sinal)
    cor   = meta[0] if meta else COR_PADRAO
    titulo = meta[1] if meta else nome_sinal

    fig, ax = plt.subplots(figsize=(12, 3.6))

    # Linha principal e área preenchida abaixo (alpha baixo para não poluir)
    ax.plot(tempo_relativo, valores, color=cor, linewidth=1.2, alpha=0.92)
    ax.fill_between(tempo_relativo, valores, alpha=0.07, color=cor)

    # Títulos e rótulos
    ax.set_title(titulo, fontsize=10, fontweight="bold", pad=7)
    ax.set_xlabel("Tempo relativo (s)", fontsize=8, labelpad=4)
    ax.set_ylabel(unidade, fontsize=8)

    # Formata eixo X com sufixo 's' (ex: "120s" em vez de "120")
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.0f}s"))
    ax.tick_params(labelsize=7)
    ax.spines[["top", "right"]].set_visible(False)

    # Caixa de estatísticas no canto superior direito
    estatisticas = (
        f"n={len(df)}  |  {frequencia_hz:.1f} Hz\n"
        f"min={valores.min():.2f}  max={valores.max():.2f}  μ={valores.mean():.2f} {unidade}"
    )
    ax.text(
        0.99, 0.97,
        estatisticas,
        transform=ax.transAxes,
        fontsize=6.5,
        va="top", ha="right",
        color=COR_TEXTO_SUAVE,
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor=COR_FUNDO_FIGURA,
            alpha=0.7,
            edgecolor=COR_GRADE,
        ),
    )

    fig.tight_layout(pad=1.4)
    caminho_saida = diretorio_plots / f"{nome_sinal}.png"
    fig.savefig(caminho_saida, dpi=150, bbox_inches="tight", facecolor=COR_FUNDO_FIGURA)
    plt.close(fig)
    print(f"    {nome_sinal:<26}  {len(df)} pts  |  {duracao_total:.1f}s  |  {frequencia_hz:.1f}Hz  →  plots/{nome_sinal}.png")


# ── Processamento por pasta ───────────────────────────────────────────────────

def processar_pasta(pasta: Path) -> None:
    """
    Processa todos os sinais (válidos e inválidos) de uma pasta de sessão.

    Cria o subdiretório plots/ se ainda não existir.
    Sinais inválidos são plotados primeiro (cartões de erro) para que
    apareçam no início da listagem de saída.
    """
    csvs   = sorted(pasta.glob("*.csv"))
    invalids = sorted(pasta.glob("*.invalid"))

    if not csvs and not invalids:
        return  # pasta vazia ou apenas subpastas (ex: plots/)

    diretorio_plots = pasta / "plots"
    diretorio_plots.mkdir(exist_ok=True)

    print(f"\n[{pasta.name}]  {len(csvs)} sinal(is) válido(s)  |  {len(invalids)} inválido(s)")

    # Sinais inválidos primeiro
    for caminho_invalid in invalids:
        plotar_sinal_invalido(caminho_invalid, diretorio_plots)

    # Sinais válidos
    for caminho_csv in csvs:
        plotar_sinal(caminho_csv, diretorio_plots)


# ── Ponto de entrada ──────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  PLOTADOR DE TELEMETRIA CAN")
    print("=" * 60)

    # Filtro opcional por nome de pasta (ex: 'session_0055' ou 'candump-1999')
    filtro_nome = sys.argv[1].strip() if len(sys.argv) > 1 else None

    if not DIR_PROCESSADO.exists():
        print("\n[ERRO] Diretório data/processed/ não encontrado.")
        print("       Execute os extratores antes do plotador.\n")
        return

    # Lista apenas diretórios (ignora arquivos na raiz de processed/)
    pastas_disponiveis = sorted(
        pasta for pasta in DIR_PROCESSADO.iterdir()
        if pasta.is_dir() and (filtro_nome is None or filtro_nome in pasta.name)
    )

    if not pastas_disponiveis:
        print("\n[ERRO] Nenhuma pasta de dados encontrada em data/processed/.")
        print("       Execute os extratores antes do plotador.\n")
        return

    for pasta in pastas_disponiveis:
        processar_pasta(pasta)

    print("\nPronto. Gráficos em data/processed/<arquivo>/plots/\n")
    print("Sinais inválidos aparecem como cartão de erro no mesmo diretório.\n")


if __name__ == "__main__":
    main()