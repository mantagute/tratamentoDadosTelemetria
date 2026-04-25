"""
getTrajetoria.py
================
Reconstrução de trajetória 2D por dead reckoning a partir de sinais IMU.

VISÃO GERAL
-----------
Combina a velocidade longitudinal integrada (VENTOR_LINEAR_VEL_Y) com o
heading acumulado a partir da velocidade angular do giroscópio
(VENTOR_ANGULAR_SPEED_Z) para reconstruir a posição x, y do veículo no
plano da pista ao longo do tempo.

PIPELINE DE PROCESSAMENTO
-------------------------
  1. CARGA E VALIDAÇÃO
     Carrega VENTOR_LINEAR_VEL_Y.csv e VENTOR_ANGULAR_SPEED_Z.csv do mesmo
     diretório de sessão. Ambos devem existir — o primeiro é gerado por
     getVelocidade.py, o segundo por extratorCandumpFiles.py.

  2. CORREÇÃO DE BIAS DO GIROSCÓPIO
     Estima o offset estático do giroscópio pela média das amostras com
     menor magnitude absoluta (percentil 5% — mesmo mecanismo do
     getVelocidade.py). Bias não corrigido acumula erro de heading
     linearmente e distorce toda a trajetória.

  3. FILTRO BUTTERWORTH PASSA-BAIXA
     Remove ruído de alta frequência do giroscópio antes de integrar.
     Parâmetros: 4ª ordem, cutoff 2 Hz, filtfilt (fase zero).
     Cutoff menor que o de ACC (3 Hz) porque o heading é mais sensível
     a ruído — qualquer componente espúrio se integra duas vezes
     (ω → θ → posição).

  4. INTERPOLAÇÃO TEMPORAL
     VENTOR_ANGULAR_SPEED_Z e VENTOR_LINEAR_VEL_Y têm timestamps
     independentes (gerados por fontes diferentes no barramento CAN).
     O giroscópio é interpolado linearmente nos timestamps da velocidade
     para criar uma grade temporal comum antes de operar os dois sinais
     juntos.

  5. INTEGRAÇÃO DO HEADING
     Integra VENTOR_ANGULAR_SPEED_Z → heading θ por método trapezoidal
     com timestamps reais:
         θ[i] = θ[i-1] + 0.5 × (ω[i] + ω[i-1]) × Δt[i]
     θ₀ = 0 (heading inicial arbitrário — norte local).

  6. DECOMPOSIÇÃO EM WORLD FRAME
     Projeta a velocidade escalar longitudinal nas componentes do world
     frame usando o heading acumulado:
         vx[i] = vel[i] × cos(θ[i])    (componente lateral)
         vy[i] = vel[i] × sin(θ[i])    (componente longitudinal)

  7. INTEGRAÇÃO DE POSIÇÃO
     Integra vx, vy → x, y por método trapezoidal com timestamps reais.

  8. MÉTRICAS DE QUALIDADE
     Calcula e exibe o erro de fechamento — distância euclidiana entre
     o ponto inicial e final da trajetória. Para sessões em pista fechada,
     esse erro é a principal métrica de qualidade do dead reckoning.

  9. SALVAMENTO
     Salva TRAJETORIA_X.csv e TRAJETORIA_Y.csv no mesmo diretório dos
     arquivos de entrada, no formato padrão da pipeline.

CONVENÇÃO DE SINAL DO YAW
--------------------------
O sentido positivo de VENTOR_ANGULAR_SPEED_Z (horário ou anti-horário
visto de cima) depende da orientação de montagem da IMU no chassi e ainda
não foi confirmado empiricamente. Se a trajetória gerada aparecer espelhada
em relação ao esperado (curvas à direita resultando em curvatura à esquerda
no mapa), negue o sinal usando a flag --negar-yaw:

    python3 src/getTrajetoria.py data/processed/candump-xyz/ --negar-yaw

Após confirmar o sentido, registre a convenção no TRAJECTORY.md e remova
a necessidade da flag atualizando o sinal diretamente em SINAIS_CANDUMP.

USO
---
  # Um diretório:
  python3 src/getTrajetoria.py data/processed/candump-1999-12-31/

  # Múltiplos diretórios:
  python3 src/getTrajetoria.py data/processed/candump-*/

  # Com negação do yaw (se a trajetória sair espelhada):
  python3 src/getTrajetoria.py data/processed/candump-xyz/ --negar-yaw

SAÍDA
-----
  data/processed/<pasta>/TRAJETORIA_X.csv
  data/processed/<pasta>/TRAJETORIA_Y.csv
"""

import sys
import re
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import butter, filtfilt
from scipy.interpolate import interp1d


# ── Parâmetros do filtro do giroscópio ───────────────────────────────────────

# Ordem e cutoff menores que os de ACC porque o heading é duplamente sensível
# a ruído: qualquer componente espúrio se integra duas vezes (ω → θ → posição).
FILTRO_ORDEM  = 4
FILTRO_CUTOFF = 2.0   # Hz


# ── Helpers de parsing do campo 'dado' ───────────────────────────────────────

def extrair_valor(texto: str) -> float | None:
    """Extrai o valor numérico (com sinal) do campo 'dado'. Ex: '-0.29 m/s²' → -0.29"""
    match = re.search(r"-?\d+\.?\d*", str(texto))
    return float(match.group()) if match else None


def montar_linha_csv(
    nome_sinal: str,
    timestamp: float,
    can_id: str,
    prioridade: int,
    valor: float,
    unidade: str,
) -> dict:
    """Retorna um dicionário no formato padrão do CSV de saída da pipeline."""
    return {
        "names":      nome_sinal,
        "timestamp":  round(timestamp, 6),
        "id_can":     can_id,
        "prioridade": prioridade,
        "dado":       f"{valor:.4f} {unidade}",
    }


# ── Carregamento de CSVs ──────────────────────────────────────────────────────

def carregar_sinal(caminho: Path) -> pd.DataFrame | None:
    """
    Carrega um CSV de sinal da pipeline e extrai timestamps e valores numéricos.

    Retorna DataFrame com colunas [timestamp, valor] ordenado por timestamp,
    ou None se o arquivo não existir ou tiver dados insuficientes.
    """
    if not caminho.exists():
        print(f"  [ERRO] Arquivo não encontrado: {caminho.name}")
        return None

    df = pd.read_csv(caminho)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["valor"]     = df["dado"].apply(extrair_valor)
    df = df.dropna(subset=["timestamp", "valor"]).sort_values("timestamp").reset_index(drop=True)

    if len(df) < 10:
        print(f"  [ERRO] {caminho.name} tem amostras insuficientes ({len(df)} < 10).")
        return None

    return df


# ── Correção de bias ──────────────────────────────────────────────────────────

def corrigir_bias(valores: np.ndarray, percentil: float = 5.0) -> tuple[np.ndarray, float]:
    """
    Estima e remove o offset estático de um sinal de sensor.

    O bias é a média das amostras com magnitude abaixo do percentil 5%
    (momentos de quasi-repouso). Em repouso, o sinal real deve ser ~0;
    qualquer resíduo é erro do sensor.

    Retorna o array corrigido e o bias estimado.
    """
    magnitude      = np.abs(valores)
    limiar         = np.percentile(magnitude, percentil)
    amostras_repouso = valores[magnitude <= limiar]
    bias           = amostras_repouso.mean() if len(amostras_repouso) > 0 else 0.0
    return valores - bias, bias


# ── Filtro passa-baixa ────────────────────────────────────────────────────────

def aplicar_filtro_butterworth(
    valores: np.ndarray,
    taxa_amostragem: float,
    cutoff: float = FILTRO_CUTOFF,
    ordem: int = FILTRO_ORDEM,
) -> np.ndarray:
    """
    Aplica filtro Butterworth passa-baixa de fase zero (filtfilt).

    Usa frequência de Nyquist derivada da taxa de amostragem real do sinal.
    """
    nyquist = 0.5 * taxa_amostragem
    coef_b, coef_a = butter(ordem, cutoff / nyquist, btype="low")
    return filtfilt(coef_b, coef_a, valores)


# ── Integração trapezoidal ────────────────────────────────────────────────────

def integrar_trapezio(valores: np.ndarray, timestamps: np.ndarray, valor_inicial: float = 0.0) -> np.ndarray:
    """
    Integra um sinal por método trapezoidal usando timestamps reais.

    Robusto a taxas de amostragem irregulares e lacunas no log.
    Fórmula: integral[i] = integral[i-1] + 0.5 × (val[i] + val[i-1]) × Δt[i]
    """
    resultado    = np.zeros(len(valores))
    resultado[0] = valor_inicial
    for i in range(1, len(valores)):
        delta_t      = timestamps[i] - timestamps[i - 1]
        media        = 0.5 * (valores[i] + valores[i - 1])
        resultado[i] = resultado[i - 1] + media * delta_t
    return resultado


# ── Pipeline principal ────────────────────────────────────────────────────────

def processar_diretorio(diretorio: Path, negar_yaw: bool = False) -> None:
    """
    Executa a reconstrução de trajetória 2D para um diretório de sessão.

    Espera encontrar VENTOR_LINEAR_VEL_Y.csv e VENTOR_ANGULAR_SPEED_Z.csv
    no diretório informado.
    """
    print(f"\n{'─' * 60}")
    print(f"  [trajetoria] {diretorio.name}")
    print(f"{'─' * 60}")

    # ── Etapa 1: Carregar sinais ──────────────────────────────────────────────
    df_vel = carregar_sinal(diretorio / "VENTOR_LINEAR_VEL_Y.csv")
    df_yaw = carregar_sinal(diretorio / "VENTOR_ANGULAR_SPEED_Z.csv")

    if df_vel is None or df_yaw is None:
        print("  [skip] Sinais necessários ausentes. Execute getVelocidade.py e extratorCandumpFiles.py antes.")
        return

    t_vel = df_vel["timestamp"].to_numpy()
    vel   = df_vel["valor"].to_numpy()

    t_yaw = df_yaw["timestamp"].to_numpy()
    yaw   = df_yaw["valor"].to_numpy()

    # Lê metadados do sinal de yaw para propagar no CSV de saída
    can_id_yaw    = str(df_yaw["id_can"].iloc[0])
    prioridade_yaw = int(df_yaw["prioridade"].iloc[0])

    print(f"  VEL   : {len(vel)} pts  |  Δt={t_vel[-1]-t_vel[0]:.1f}s")
    print(f"  YAW   : {len(yaw)} pts  |  Δt={t_yaw[-1]-t_yaw[0]:.1f}s")

    if negar_yaw:
        yaw = -yaw
        print("  [flag] --negar-yaw ativo: sinal do giroscópio invertido.")

    # ── Etapa 2: Correção de bias do giroscópio ───────────────────────────────
    yaw_corrigido, bias_yaw = corrigir_bias(yaw)
    print(f"  Bias yaw estimado : {bias_yaw:.6f} rad/s")

    # ── Etapa 3: Filtro passa-baixa no giroscópio ─────────────────────────────
    intervalo_medio_yaw = np.diff(t_yaw).mean()
    fs_yaw              = 1.0 / intervalo_medio_yaw
    yaw_filtrado        = aplicar_filtro_butterworth(yaw_corrigido, fs_yaw)
    print(f"  Fs yaw detectada  : {fs_yaw:.2f} Hz  |  Cutoff filtro: {FILTRO_CUTOFF} Hz")

    # ── Etapa 4: Interpolação temporal ───────────────────────────────────────
    # O giroscópio e a velocidade têm timestamps independentes. Interpola o
    # giroscópio nos timestamps da velocidade para criar uma grade comum.
    # Usa only_range para não extrapolar: limita ao intervalo de sobreposição.
    t_inicio_comum = max(t_vel[0],  t_yaw[0])
    t_fim_comum    = min(t_vel[-1], t_yaw[-1])

    mascara_vel = (t_vel >= t_inicio_comum) & (t_vel <= t_fim_comum)
    if mascara_vel.sum() < 10:
        print("  [ERRO] Sobreposição temporal entre VEL e YAW é insuficiente (< 10 amostras).")
        print("         Verifique se os arquivos pertencem à mesma sessão.")
        return

    t_vel_comum = t_vel[mascara_vel]
    vel_comum   = vel[mascara_vel]

    # Interpolação linear do giroscópio filtrado nos timestamps da velocidade
    interp_yaw     = interp1d(t_yaw, yaw_filtrado, kind="linear", bounds_error=False, fill_value=0.0)
    yaw_interpolado = interp_yaw(t_vel_comum)

    n_amostras = len(t_vel_comum)
    duracao    = t_vel_comum[-1] - t_vel_comum[0]
    print(f"  Grade comum       : {n_amostras} pts  |  {duracao:.1f}s  |  sobreposição [{t_inicio_comum:.2f}, {t_fim_comum:.2f}]")

    # ── Etapa 5: Integração do heading ────────────────────────────────────────
    # θ₀ = 0 (heading inicial arbitrário — norte local do sistema de coords)
    theta = integrar_trapezio(yaw_interpolado, t_vel_comum, valor_inicial=0.0)

    theta_min_deg = np.degrees(theta.min())
    theta_max_deg = np.degrees(theta.max())
    print(f"  Heading range     : [{theta_min_deg:.1f}°, {theta_max_deg:.1f}°]  (rotação total: {np.degrees(theta[-1]):.1f}°)")

    # ── Etapa 6: Decomposição em world frame ──────────────────────────────────
    vx = vel_comum * np.cos(theta)
    vy = vel_comum * np.sin(theta)

    # ── Etapa 7: Integração de posição ───────────────────────────────────────
    x = integrar_trapezio(vx, t_vel_comum, valor_inicial=0.0)
    y = integrar_trapezio(vy, t_vel_comum, valor_inicial=0.0)

    # ── Etapa 8: Métricas de qualidade ───────────────────────────────────────
    erro_fechamento = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()
    print(f"  Extensão trajetória : X=[{x.min():.1f}, {x.max():.1f}]m  Y=[{y.min():.1f}, {y.max():.1f}]m")
    print(f"  Erro de fechamento  : {erro_fechamento:.2f}m  (sobre extensão aprox. {max(x_range, y_range):.0f}m)")

    # ── Etapa 9: Salvar CSVs de trajetória ────────────────────────────────────
    # Usa o can_id do sinal de yaw como referência (origem da posição é a IMU)
    linhas_x = [
        montar_linha_csv("TRAJETORIA_X", t_vel_comum[i], can_id_yaw, prioridade_yaw, x[i], "m")
        for i in range(n_amostras)
    ]
    linhas_y = [
        montar_linha_csv("TRAJETORIA_Y", t_vel_comum[i], can_id_yaw, prioridade_yaw, y[i], "m")
        for i in range(n_amostras)
    ]

    caminho_x = diretorio / "TRAJETORIA_X.csv"
    caminho_y = diretorio / "TRAJETORIA_Y.csv"

    pd.DataFrame(linhas_x).to_csv(caminho_x, index=False)
    pd.DataFrame(linhas_y).to_csv(caminho_y, index=False)

    print(f"  → {caminho_x.name}")
    print(f"  → {caminho_y.name}")


# ── Ponto de entrada ──────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  RECONSTRUÇÃO DE TRAJETÓRIA — DEAD RECKONING")
    print("=" * 60)

    # Parse de argumentos: separa flags de caminhos
    args       = sys.argv[1:]
    negar_yaw  = "--negar-yaw" in args
    caminhos   = [a for a in args if not a.startswith("--")]

    if not caminhos:
        print("\nUso: python3 getTrajetoria.py <diretório_de_sessão> [--negar-yaw]")
        print("\nExemplos:")
        print("  python3 src/getTrajetoria.py data/processed/candump-1999-12-31/")
        print("  python3 src/getTrajetoria.py data/processed/candump-*/")
        print("  python3 src/getTrajetoria.py data/processed/candump-xyz/ --negar-yaw")
        print("\nEspera encontrar no diretório:")
        print("  VENTOR_LINEAR_VEL_Y.csv    (saída de getVelocidade.py)")
        print("  VENTOR_ANGULAR_SPEED_Z.csv (saída de extratorCandumpFiles.py)")
        print("\n--negar-yaw: inverte o sinal do giroscópio (use se a trajetória sair espelhada).")
        return

    # Expande globs e resolve caminhos
    diretorios_processados = []
    for argumento in caminhos:
        caminho = Path(argumento)
        if caminho.is_dir():
            diretorios_processados.append(caminho)
        else:
            # Tenta glob relativo ao cwd
            expandidos = sorted(Path().glob(argumento))
            for p in expandidos:
                if p.is_dir():
                    diretorios_processados.append(p)

    if not diretorios_processados:
        print("\n[ERRO] Nenhum diretório válido encontrado nos argumentos fornecidos.")
        return

    for diretorio in diretorios_processados:
        processar_diretorio(diretorio, negar_yaw=negar_yaw)

    print("\n" + "=" * 60)
    print("  TRAJETÓRIA CONCLUÍDA")
    print("=" * 60)
    print("  CSVs gerados: TRAJETORIA_X.csv, TRAJETORIA_Y.csv")
    print("  Execute o plotador para visualizar os resultados.")
    print("  Verifique o erro de fechamento — quanto menor, melhor a qualidade.\n")


if __name__ == "__main__":
    main()