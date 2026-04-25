"""
extratorCandumpFiles.py
=======================
Extrator de telemetria CAN a partir de arquivos no formato candump.

VISÃO GERAL
-----------
O candump é uma ferramenta do pacote can-utils do Linux que registra o
tráfego CAN diretamente do kernel. Cada linha do arquivo representa uma
mensagem capturada no barramento, no seguinte formato:

    (timestamp_unix) interface CANID#HEXDATA
    (0946688473.192390) can0 00000001#E3FF000005000000

Este script lê esses arquivos linha a linha, identifica as mensagens de
interesse pelo CAN ID, decodifica os bytes do payload em valores físicos
e salva um CSV por sinal no diretório data/processed/.

SINAIS SUPORTADOS
-----------------
    Mensagem                  | CAN ID     | Sinal                  | Bytes | Tipo   | Escala | Unidade
    acceleration_vector_x_y_1 | 0x00000001 | VENTOR_LINEAR_ACC_X    | 0–1   | int16  | × 0.01 | m/s²
    acceleration_vector_x_y_1 | 0x00000001 | VENTOR_ANGULAR_SPEED_X | 2–3   | int16  | × 0.01 | rad/s
    acceleration_vector_x_y_1 | 0x00000001 | VENTOR_LINEAR_ACC_Y    | 4–5   | int16  | × 0.01 | m/s²
    acceleration_vector_x_y_1 | 0x00000001 | VENTOR_ANGULAR_SPEED_Y | 6–7   | int16  | × 0.01 | rad/s
    acceleration_vector_z_2   | 0x00000002 | VENTOR_LINEAR_ACC_Z    | 0–1   | int16  | × 0.01 | m/s²
    acceleration_vector_z_2   | 0x00000002 | VENTOR_ANGULAR_SPEED_Z | 2–3   | int16  | × 0.01 | rad/s
    VCU_DATA_OUT              | 0x18FF1515 | APS_PERC               | 2–3   | uint16 | × 0.01 | %

NOTA SOBRE APS_PERC (pedal de acelerador)
-----------------------------------------
O sinal APS_PERC ocupa os bits 16–31 da mensagem VCU_DATA_OUT (bytes 2–3
em notação little-endian). O fator de escala 0.01 corresponde ao divisor
100 da especificação original: 10000 raw → 100.00%.

NOTA SOBRE VENTOR_ANGULAR_SPEED_Z (yaw rate)
--------------------------------------------
A convenção de sinal NÃO está confirmada: positivo pode ser horário ou
anti-horário visto de cima, dependendo da orientação de montagem da IMU
no chassi. Validar com uma curva de referência (ex: curva à direita) e
registrar a convenção antes de usar em getTrajetoria.py.

SAÍDA
-----
    data/processed/<nome_do_arquivo_log>/<SINAL>.csv
    data/processed/<nome_do_arquivo_log>/<SINAL>.invalid  (se suspeito)

FORMATO DO CSV DE SAÍDA
-----------------------
    names, timestamp, id_can, prioridade, dado
    VENTOR_LINEAR_ACC_X, 946688473.19, 0x00000001, 1, -0.29 m/s²

USO
---
    python3 src/extratorCandumpFiles.py

    Coloque os arquivos .log em data/raw/candumpFiles/ antes de executar.
"""

import re
import struct
import pandas as pd
import numpy as np
from pathlib import Path


# ── Parâmetros globais ────────────────────────────────────────────────────────

# Arquivos candump costumam ser menores que session CSVs; limiar mais permissivo
TAMANHO_MINIMO_KB = 1

# Se ≥ 20% das amostras de um sinal forem rejeitadas pela validação física,
# o sinal inteiro é considerado suspeito e não gera CSV
LIMIAR_INVALIDADE = 0.20


# ── Resolução de caminhos ─────────────────────────────────────────────────────

def _resolver_diretorio_base() -> Path:
    """
    Localiza o diretório raiz do projeto de forma robusta, independente
    de onde o script foi invocado.

    Testa candidatos em ordem crescente de distância e retorna o primeiro
    que contém a pasta data/. Se nenhum for encontrado, usa o cwd.
    """
    diretorio_script = Path(__file__).resolve().parent
    candidatos = [
        diretorio_script,
        diretorio_script.parent,
        Path.cwd(),
        Path.cwd().parent,
    ]
    for candidato in candidatos:
        if (candidato / "data").exists():
            return candidato
    return Path.cwd()


DIR_BASE    = _resolver_diretorio_base()
DIR_CANDUMP = DIR_BASE / "data" / "raw" / "candumpFiles"
DIR_SAIDA   = DIR_BASE / "data" / "processed"


# ── Mapa de sinais ────────────────────────────────────────────────────────────
#
# Estrutura de cada entrada:
#   nome_sinal → (
#       can_id_int,        ID CAN como inteiro
#       byte_inicio,       offset inicial no payload (bytes)
#       byte_comprimento,  número de bytes a ler (1 ou 2)
#       com_sinal,         True = int com sinal, False = unsigned
#       multiplicador,     multiplica o valor bruto
#       offset,            soma após a multiplicação
#       unidade,           string da unidade física
#       prioridade,        campo de prioridade no CSV de saída
#       limite_minimo,     valor físico mínimo aceitável
#       limite_maximo,     valor físico máximo aceitável
#       delta_max_frame,   variação máxima aceitável entre dois frames (None = sem filtro)
#   )
#
# Fórmula de decodificação: valor_fisico = (raw * multiplicador) + offset
#
# Diferença em relação ao extrator de sessão: aqui usa-se multiplicador
# (em vez de divisor), pois os sinais IMU são especificados com fator ×.

SINAIS_CANDUMP = {
    #                             can_id      ini compr sgn   mult  off  unit      prio  min    max    delta

    # ── Mensagem 0x00000001: acceleration_vector_x_y_1 ───────────────────────
    # Layout: [ACC_X(0-1)] [ANG_SPEED_X(2-3)] [ACC_Y(4-5)] [ANG_SPEED_Y(6-7)]
    #"VENTOR_LINEAR_ACC_X":    (0x00000001,  0,  2, True,  0.01, 0.0, "m/s²",  1, -20.0, 20.0,  None),
    # NOTA: angular_speed_x mede rotação em torno do eixo X (rolamento/roll).
    # Sentido positivo: não confirmado. Validar com manobra de referência
    # (inclinar o veículo para a direita e verificar o sinal resultante).
    "VENTOR_ANGULAR_SPEED_X": (0x00000001,  2,  2, True,  0.01, 0.0, "rad/s", 1, -20.0, 20.0,  None),
    #"VENTOR_LINEAR_ACC_Y":    (0x00000001,  4,  2, True,  0.01, 0.0, "m/s²",  1, -20.0, 20.0,  None),
    # NOTA: angular_speed_y mede rotação em torno do eixo Y (arfagem/pitch).
    # Sentido positivo: não confirmado.
    "VENTOR_ANGULAR_SPEED_Y": (0x00000001,  6,  2, True,  0.01, 0.0, "rad/s", 1, -20.0, 20.0,  None),

    # ── Mensagem 0x00000002: acceleration_vector_z_2 ─────────────────────────
    #"VENTOR_LINEAR_ACC_Z":    (0x00000002,  0,  2, True,  0.01, 0.0, "m/s²",  1, -20.0, 20.0,  None),
    # NOTA CRÍTICA — convenção de sinal do yaw NÃO confirmada.
    # Positivo pode ser horário ou anti-horário visto de cima dependendo da
    # montagem da IMU no chassi. Validar antes de usar em getTrajetoria.py:
    # fazer uma curva à direita e verificar se angular_speed_z é positiva ou
    # negativa. Registrar a convenção aqui e no TRAJECTORY.md após confirmação.
    "VENTOR_ANGULAR_SPEED_Z": (0x00000002,  2,  2, True,  0.01, 0.0, "rad/s", 1, -20.0, 20.0,  None),

    # ── VCU — Pedal de acelerador ─────────────────────────────────────────────
    #"APS_PERC":               (0x18FF1515,  2,  2, False, 0.01, 0.0, "%",     1,   0.0, 100.0, None),
}


# ── Expressão regular para parsing do formato candump ────────────────────────

# Captura os três grupos de cada linha válida do candump:
#   Grupo 1: timestamp com ponto decimal (ex: 0946688473.192390)
#   Grupo 2: CAN ID hexadecimal (ex: 00000001 ou 18FF1515)
#   Grupo 3: payload hexadecimal (ex: E3FF000005000000)
_REGEX_LINHA_CANDUMP = re.compile(
    r"^\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)\s*$"
)

# Mapeamento (comprimento_bytes, com_sinal) → formato struct little-endian
_FORMATOS_STRUCT = {
    (1, True):  "b",   # int8
    (1, False): "B",   # uint8
    (2, True):  "h",   # int16
    (2, False): "H",   # uint16
}


# ── Decodificação de bytes ────────────────────────────────────────────────────

def decodificar_payload(
    payload: bytes,
    byte_inicio: int,
    byte_comprimento: int,
    com_sinal: bool,
    multiplicador: float,
    offset: float,
) -> float | None:
    """
    Extrai e converte um valor físico de um payload CAN (formato candump).

    Lê `byte_comprimento` bytes a partir de `byte_inicio` no payload,
    interpreta como inteiro little-endian e aplica:
        valor_fisico = (raw * multiplicador) + offset

    Retorna None se o formato não for suportado ou o payload for curto demais.
    """
    formato = _FORMATOS_STRUCT.get((byte_comprimento, com_sinal))
    if formato is None:
        return None

    if byte_inicio + byte_comprimento > len(payload):
        return None

    try:
        valor_bruto = struct.unpack_from("<" + formato, payload, byte_inicio)[0]
        return round(valor_bruto * multiplicador + offset, 4)
    except Exception:
        return None


# ── Verificação de tamanho ────────────────────────────────────────────────────

def arquivo_tem_tamanho_valido(caminho: Path) -> bool:
    """
    Verifica se o arquivo .log tem pelo menos TAMANHO_MINIMO_KB.
    Arquivos menores provavelmente foram truncados ou não contêm dados.
    """
    tamanho_kb = caminho.stat().st_size / 1024
    if tamanho_kb < TAMANHO_MINIMO_KB:
        print(f"  [skip] {caminho.name}  ({tamanho_kb:.1f} KB < {TAMANHO_MINIMO_KB} KB mínimo)")
        return False
    return True


# ── Montagem de linhas do CSV de saída ───────────────────────────────────────

def montar_linha_csv(
    nome_sinal: str,
    timestamp: float,
    can_id: int,
    prioridade: int,
    valor: float,
    unidade: str,
) -> dict:
    """
    Retorna um dicionário no formato padrão do CSV de saída da pipeline.
    """
    return {
        "names":      nome_sinal,
        "timestamp":  round(timestamp, 6),
        "id_can":     f"0x{can_id:08X}",
        "prioridade": prioridade,
        "dado":       f"{valor:.2f} {unidade}",
    }


# ── Validação física ──────────────────────────────────────────────────────────

def validar_sinal(
    linhas: list[dict],
    limite_minimo: float | None,
    limite_maximo: float | None,
    delta_max: float | None,
    nome_sinal: str,
) -> tuple[list[dict], dict]:
    """
    Filtra amostras fisicamente impossíveis de um sinal.

    Aplica dois critérios de rejeição:
      1. Range físico: valores fora de [limite_minimo, limite_maximo] são descartados.
      2. Variação brusca: se |val[i] - val[i-1]| > delta_max, a amostra i é descartada.

    Retorna:
      - lista de linhas válidas
      - dicionário com métricas de qualidade (n_total, n_invalido, ratio, suspeito)
    """
    if not linhas:
        return linhas, {}

    valores = np.array([float(r["dado"].split()[0]) for r in linhas])
    n_total = len(valores)

    mascara_valida = np.ones(n_total, dtype=bool)

    # Critério 1: range físico
    if limite_minimo is not None:
        mascara_valida &= valores >= limite_minimo
    if limite_maximo is not None:
        mascara_valida &= valores <= limite_maximo

    # Critério 2: variação entre frames consecutivos
    if delta_max is not None and n_total > 1:
        variacoes = np.abs(np.diff(valores))
        saltos = np.concatenate([[False], variacoes > delta_max])
        mascara_valida &= ~saltos

    n_invalidas = int((~mascara_valida).sum())
    taxa_invalidade = n_invalidas / n_total

    metricas = {
        "n_total":    n_total,
        "n_invalido": n_invalidas,
        "ratio":      taxa_invalidade,
        "suspeito":   taxa_invalidade >= LIMIAR_INVALIDADE,
    }

    linhas_validas = [linha for linha, manter in zip(linhas, mascara_valida) if manter]
    return linhas_validas, metricas


# ── Leitura do arquivo candump ────────────────────────────────────────────────

def processar_arquivo_candump(caminho: Path) -> dict[str, list]:
    """
    Lê um arquivo candump linha a linha e decodifica todos os sinais mapeados.

    Estratégia de leitura:
      - Pré-agrupa os sinais por CAN ID para evitar busca linear por sinal
        a cada linha do arquivo (O(1) por lookup em vez de O(n_sinais)).
      - Usa expressão regular compilada para parsing eficiente do formato candump.
      - Linhas RTR (Remote Transmission Request) ou com payload inválido são
        silenciosamente ignoradas (bytes.fromhex() lança ValueError).

    Retorna um dicionário {nome_sinal: [lista de linhas CSV]}.
    """
    # Pré-índice: CAN ID → lista de nomes de sinais que pertencem a ele
    sinais_por_can_id: dict[int, list[str]] = {}
    for nome_sinal, (can_id, *_) in SINAIS_CANDUMP.items():
        sinais_por_can_id.setdefault(can_id, []).append(nome_sinal)

    resultado: dict[str, list] = {nome: [] for nome in SINAIS_CANDUMP}

    with caminho.open("r", encoding="utf-8", errors="replace") as arquivo:
        for linha in arquivo:
            match = _REGEX_LINHA_CANDUMP.match(linha.strip())
            if not match:
                continue  # linha de comentário, vazia ou com formato diferente

            timestamp_str, id_str, hex_payload = match.groups()
            can_id_int = int(id_str, 16)

            # Ignora mensagens cujo CAN ID não tem sinal mapeado
            sinais_do_id = sinais_por_can_id.get(can_id_int)
            if not sinais_do_id:
                continue

            timestamp = float(timestamp_str)

            try:
                payload = bytes.fromhex(hex_payload)
            except ValueError:
                # Frames RTR (sem payload) ou com caracteres inválidos
                continue

            for nome_sinal in sinais_do_id:
                _, byte_ini, byte_comp, com_sinal, mult, off, unidade, prio, *_ = SINAIS_CANDUMP[nome_sinal]
                valor = decodificar_payload(payload, byte_ini, byte_comp, com_sinal, mult, off)
                if valor is not None:
                    resultado[nome_sinal].append(
                        montar_linha_csv(nome_sinal, timestamp, can_id_int, prio, valor, unidade)
                    )

    return resultado


# ── Validação e salvamento ────────────────────────────────────────────────────

def salvar_sinais(
    sinais: dict[str, list],
    mapa_especificacoes: dict,
    diretorio_saida: Path,
) -> None:
    """
    Valida e persiste cada sinal decodificado em disco.

    Para sinais suspeitos (alta taxa de invalidade), gera um arquivo .invalid
    com diagnóstico em vez do CSV. Isso garante que dados corrompidos não
    entrem silenciosamente no pipeline de análise.
    """
    diretorio_saida.mkdir(parents=True, exist_ok=True)

    for nome_sinal, linhas in sinais.items():
        if not linhas:
            continue

        spec       = mapa_especificacoes.get(nome_sinal)
        limite_min = spec[8]  if spec else None
        limite_max = spec[9]  if spec else None
        delta_max  = spec[10] if spec else None

        linhas_validas, metricas = validar_sinal(linhas, limite_min, limite_max, delta_max, nome_sinal)

        if metricas.get("suspeito"):
            caminho_alerta = diretorio_saida / f"{nome_sinal}.invalid"
            n_t = metricas["n_total"]
            n_i = metricas["n_invalido"]
            pct = metricas["ratio"] * 100
            caminho_alerta.write_text(
                f"SINAL INVÁLIDO: {nome_sinal}\n"
                f"  Total de amostras brutas : {n_t}\n"
                f"  Amostras rejeitadas       : {n_i} ({pct:.1f}%)\n"
                f"  Limiar de suspeita        : {LIMIAR_INVALIDADE * 100:.0f}%\n"
                f"\n"
                f"  Diagnóstico: a taxa de amostras fora do range físico ou com\n"
                f"  variação brusca entre frames (Δ > {delta_max}) supera o limiar.\n"
                f"  Revise a especificação CAN ou o firmware antes de usar este sinal.\n"
            )
            print(f"  [INVÁLIDO] {nome_sinal:<26}  {n_i}/{n_t} amostras rejeitadas ({pct:.1f}%)  →  {nome_sinal}.invalid")

        elif linhas_validas:
            n_t    = metricas.get("n_total", len(linhas_validas))
            n_i    = metricas.get("n_invalido", 0)
            pct_ok = (len(linhas_validas) / n_t * 100) if n_t else 0
            df_saida = pd.DataFrame(linhas_validas)
            df_saida.to_csv(diretorio_saida / f"{nome_sinal}.csv", index=False)
            if n_i > 0:
                print(f"  [FILTRADO] {nome_sinal:<26}  {len(linhas_validas)}/{n_t} pts salvos ({pct_ok:.1f}% válidos)")


# ── Ponto de entrada ──────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  EXTRATOR DE TELEMETRIA CAN — CANDUMP")
    print("=" * 60)
    print(f"  DIR_BASE    : {DIR_BASE}")
    print(f"  CANDUMP_DIR : {DIR_CANDUMP}  {'[OK]' if DIR_CANDUMP.exists() else '[NAO ENCONTRADO]'}")
    print(f"  OUT_DIR     : {DIR_SAIDA}")
    print("=" * 60)

    if not DIR_CANDUMP.exists():
        print(f"\n[ERRO] Pasta de candump não encontrada: {DIR_CANDUMP}")
        print("       Crie a pasta data/raw/candumpFiles e coloque os arquivos .log lá.")
        return

    arquivos_log = sorted(DIR_CANDUMP.glob("*.log"))
    if not arquivos_log:
        print("\n[ERRO] Nenhum arquivo .log encontrado em CANDUMP_DIR.\n")
        return

    for caminho_arquivo in arquivos_log:
        print(f"\n[candump] {caminho_arquivo.name}  ({caminho_arquivo.stat().st_size / 1024:.0f} KB)")
        if not arquivo_tem_tamanho_valido(caminho_arquivo):
            continue

        sinais_brutos = processar_arquivo_candump(caminho_arquivo)
        diretorio_saida = DIR_SAIDA / caminho_arquivo.stem
        salvar_sinais(sinais_brutos, SINAIS_CANDUMP, diretorio_saida)

        # Imprime estatísticas de frequência e duração para cada sinal válido
        for nome_sinal, linhas in sinais_brutos.items():
            if not linhas:
                continue
            spec = SINAIS_CANDUMP[nome_sinal]
            linhas_validas, metricas = validar_sinal(linhas, spec[8], spec[9], spec[10], nome_sinal)
            if metricas.get("suspeito"):
                continue
            if linhas_validas:
                duracao = linhas_validas[-1]["timestamp"] - linhas_validas[0]["timestamp"]
                frequencia_hz = len(linhas_validas) / duracao if duracao > 0 else 0
                n_rejeitadas = metricas.get("n_invalido", 0)
                aviso = f"  [warn: {n_rejeitadas} rejeitadas]" if n_rejeitadas else ""
                print(f"  {nome_sinal:<26}  {len(linhas_validas):>6} pts  |  {duracao:.1f} s  |  {frequencia_hz:.1f} Hz{aviso}")

    print("\nPronto. CSVs em data/processed/<arquivo>/<SINAL>.csv\n")
    print("Sinais suspeitos geram arquivo .invalid no mesmo diretório.\n")


if __name__ == "__main__":
    main()