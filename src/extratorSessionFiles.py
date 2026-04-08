"""
extratorSessionFiles.py
=======================
Extrator de telemetria CAN a partir de arquivos de sessão no formato CSV.

VISÃO GERAL
-----------
Os arquivos de sessão são CSVs gerados pelo datalogger embarcado. Cada linha
representa uma mensagem CAN capturada, com colunas:

    timestamp_unix, can_id_dec, b0, b1, b2, b3, b4, b5, b6, b7

Este script lê esses arquivos, decodifica os sinais físicos de cada mensagem
CAN de interesse e salva um CSV por sinal no diretório data/processed/.

SINAIS SUPORTADOS
-----------------
Todos os sinais pertencem às mensagens dos inversores de tração A13 e B13
(endereços CAN 0x18FF01F7 e 0x18FF02F7) e aos setpoints de torque enviados
pelo VCU (endereços 0x18FFE180 e 0x18FFE280).

    Sinal            | Bytes | Tipo    | Escala          | Unidade
    ACT_SPEED_A13    | 1–2   | uint16  | raw − 32000     | rpm
    ACT_TORQUE_A13   | 3–4   | uint16  | raw / 5 − 6400  | Nm
    ACT_POWER_A13    | 5–6   | uint16  | raw / 200 − 160 | kW
    ACT_TEMP_A13     | 7     | uint8   | raw − 40        | °C
    (idem para B13)
    SETP_TORQUE_A13  | 6–7   | uint16  | raw / 5 − 6400  | Nm
    SETP_TORQUE_B13  | 6–7   | uint16  | raw / 5 − 6400  | Nm

NOTA SOBRE ACT_SPEED
--------------------
Os bytes b[1:3] do firmware dos inversores apresentam oscilações de ~16.000 rpm
entre frames consecutivos a 10 Hz durante operação — fisicamente impossível.
Isso indica que o firmware grava outro dado nesses bytes enquanto opera sob carga
(posição de encoder, contador de comutação ou dado de diagnóstico). O filtro de
variação máxima (delta_max = 2000 rpm/frame) descarta essas amostras espúrias.
Se a taxa de rejeição superar 20%, o sinal inteiro é marcado como inválido.

SAÍDA
-----
    data/processed/<nome_do_arquivo_csv>/<SINAL>.csv
    data/processed/<nome_do_arquivo_csv>/<SINAL>.invalid  (se suspeito)

FORMATO DO CSV DE SAÍDA
-----------------------
    names, timestamp, id_can, prioridade, dado
    ACT_TORQUE_A13, 946688468.12, 0x18FF01F7, 1, 25.00 Nm

USO
---
    python3 src/extratorSessionFiles.py
"""

import struct
import pandas as pd
import numpy as np
from pathlib import Path


# ── Parâmetros globais ────────────────────────────────────────────────────────

# Arquivos menores que este limite são ignorados (provavelmente vazios ou corrompidos)
TAMANHO_MINIMO_KB = 1000

# Se ≥ 20% das amostras de um sinal forem rejeitadas pela validação física,
# o sinal inteiro é considerado suspeito e não gera CSV
LIMIAR_INVALIDADE = 0.20


# ── Resolução de caminhos ─────────────────────────────────────────────────────

def _resolver_diretorio_base() -> Path:
    """
    Localiza o diretório raiz do projeto de forma robusta, independente
    de onde o script foi invocado (raiz, src/, ou qualquer subpasta).

    Testa candidatos em ordem crescente de distância e retorna o primeiro
    que contém a pasta data/sessioncsvFiles.
    """
    diretorio_script = Path(__file__).resolve().parent
    candidatos = [
        diretorio_script,         # script na raiz do projeto
        diretorio_script.parent,  # script em src/ ou subpasta
        Path.cwd(),               # diretório de trabalho atual
        Path.cwd().parent,
    ]
    for candidato in candidatos:
        if (candidato / "data" / "sessioncsvFiles").exists():
            return candidato
    return Path.cwd()


DIR_BASE    = _resolver_diretorio_base()
DIR_SESSOES = DIR_BASE / "data" / "raw" / "sessioncsvFiles"
DIR_SAIDA   = DIR_BASE / "data" / "processed"


# ── Mapa de sinais ────────────────────────────────────────────────────────────
#
# Estrutura de cada entrada:
#   nome_sinal → (
#       can_id_int,        ID CAN como inteiro
#       byte_inicio,       offset inicial no payload (bytes)
#       byte_comprimento,  número de bytes a ler (1 ou 2)
#       com_sinal,         True = int com sinal, False = unsigned
#       divisor,           divide o valor bruto
#       offset,            soma após a divisão
#       unidade,           string da unidade física
#       prioridade,        campo de prioridade no CSV de saída
#       limite_minimo,     valor físico mínimo aceitável
#       limite_maximo,     valor físico máximo aceitável
#       delta_max_frame,   variação máxima aceitável entre dois frames (None = sem filtro)
#   )
#
# Fórmula de decodificação: valor_fisico = (raw / divisor) + offset
#
# O byte 0 de cada mensagem carrega bits de status do inversor e é ignorado
# em todos os sinais de valor físico (byte_inicio começa em 1).
# Os bytes são little-endian (byte menos significativo primeiro).
# Temperatura usa uint8 com offset de -40 (padrão SAE J1939).

SINAIS_SESSAO = {
    #                      can_id       ini  compr sgn   div    off     unit   prio  min       max        delta
    "ACT_SPEED_A13":   (0x18FF01F7,  1,  2, False,   1, -32000, "rpm",  1, -32000,  33535,   2000),
    "ACT_TORQUE_A13":  (0x18FF01F7,  3,  2, False,   5,  -6400, "Nm",   1,  -6400,   6707,   None),
    "ACT_POWER_A13":   (0x18FF01F7,  5,  2, False, 200,   -160, "kW",   1,   -160,  167.675, None),
    "ACT_TEMP_A13":    (0x18FF01F7,  7,  1, False,   1,    -40, "°C",   1,    -40,    215,   None),
    "ACT_SPEED_B13":   (0x18FF02F7,  1,  2, False,   1, -32000, "rpm",  1, -32000,  33535,   2000),
    "ACT_TORQUE_B13":  (0x18FF02F7,  3,  2, False,   5,  -6400, "Nm",   1,  -6400,   6707,   None),
    "ACT_POWER_B13":   (0x18FF02F7,  5,  2, False, 200,   -160, "kW",   1,   -160,  167.675, None),
    "ACT_TEMP_B13":    (0x18FF02F7,  7,  1, False,   1,    -40, "°C",   1,    -40,    215,   None),
    "SETP_TORQUE_A13": (0x18FFE180,  6,  2, False,   5,  -6400, "Nm",   1,  -6400,   6707,   None),
    "SETP_TORQUE_B13": (0x18FFE280,  6,  2, False,   5,  -6400, "Nm",   1,  -6400,   6707,   None),
}


# ── Decodificação de bytes ────────────────────────────────────────────────────

# Mapeamento (comprimento_bytes, com_sinal) → formato struct little-endian
_FORMATOS_STRUCT = {
    (1, True):  "b",   # int8
    (1, False): "B",   # uint8
    (2, True):  "h",   # int16
    (2, False): "H",   # uint16
}


def decodificar_payload(
    payload: bytes,
    byte_inicio: int,
    byte_comprimento: int,
    com_sinal: bool,
    divisor: float,
    offset: float,
) -> float | None:
    """
    Extrai e converte um valor físico de um payload CAN.

    Lê `byte_comprimento` bytes a partir de `byte_inicio` no payload,
    interpreta como inteiro little-endian (com ou sem sinal) e aplica
    a fórmula: valor_fisico = (raw / divisor) + offset.

    Retorna None se o formato não for suportado ou se o payload for curto demais.
    """
    formato = _FORMATOS_STRUCT.get((byte_comprimento, com_sinal))
    if formato is None:
        return None

    if byte_inicio + byte_comprimento > len(payload):
        return None

    try:
        valor_bruto = struct.unpack_from("<" + formato, payload, byte_inicio)[0]
        return round((valor_bruto / divisor) + offset, 4)
    except Exception:
        return None


# ── Verificação de tamanho ────────────────────────────────────────────────────

def arquivo_tem_tamanho_valido(caminho: Path) -> bool:
    """
    Verifica se o arquivo tem pelo menos TAMANHO_MINIMO_KB.
    Arquivos menores provavelmente não contêm dados de sessão úteis.
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

    O campo 'dado' combina o valor numérico e a unidade em uma única string
    para facilitar a leitura humana e a identificação automática de unidade
    no plotador.
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
         Isso detecta saltos impostos por corrupção de dados ou comportamento
         anômalo do firmware.

    Retorna:
      - lista de linhas válidas (passaram em ambos os critérios)
      - dicionário com métricas de qualidade (n_total, n_invalido, ratio, suspeito)
    """
    if not linhas:
        return linhas, {}

    valores = np.array([float(r["dado"].split()[0]) for r in linhas])
    n_total = len(valores)

    # Começa com todos válidos e vai removendo
    mascara_valida = np.ones(n_total, dtype=bool)

    # Critério 1: range físico
    if limite_minimo is not None:
        mascara_valida &= valores >= limite_minimo
    if limite_maximo is not None:
        mascara_valida &= valores <= limite_maximo

    # Critério 2: variação entre frames consecutivos
    if delta_max is not None and n_total > 1:
        variacoes = np.abs(np.diff(valores))
        # Marca o frame *depois* da variação brusca (posição mais conservadora)
        saltos = np.concatenate([[False], variacoes > delta_max])
        mascara_valida &= ~saltos

    n_invalidas = int((~mascara_valida).sum())
    taxa_invalidade = n_invalidas / n_total

    metricas = {
        "n_total":   n_total,
        "n_invalido": n_invalidas,
        "ratio":     taxa_invalidade,
        "suspeito":  taxa_invalidade >= LIMIAR_INVALIDADE,
    }

    linhas_validas = [linha for linha, manter in zip(linhas, mascara_valida) if manter]
    return linhas_validas, metricas


# ── Leitura do arquivo de sessão ─────────────────────────────────────────────

def processar_arquivo_sessao(caminho: Path) -> dict[str, list]:
    """
    Lê um CSV de sessão e decodifica todos os sinais mapeados em SINAIS_SESSAO.

    Estratégia de leitura:
      - Agrupa os sinais por CAN ID para varrer o DataFrame uma vez por ID.
      - Para cada linha do subset filtrado, monta o payload de 8 bytes a partir
        das colunas b0–b7 e chama decodificar_payload() para cada sinal do grupo.

    Retorna um dicionário {nome_sinal: [lista de linhas CSV]}.
    """
    # A primeira linha do CSV de sessão é um cabeçalho de metadados (ignorado)
    df = pd.read_csv(caminho, skiprows=1)
    df.columns = [coluna.strip() for coluna in df.columns]

    # Agrupa sinais por CAN ID para minimizar iterações sobre o DataFrame
    sinais_por_can_id: dict[int, list[str]] = {}
    for nome_sinal, (can_id, *_) in SINAIS_SESSAO.items():
        sinais_por_can_id.setdefault(can_id, []).append(nome_sinal)

    resultado: dict[str, list] = {nome: [] for nome in SINAIS_SESSAO}

    for can_id, sinais_do_id in sinais_por_can_id.items():
        # Filtra apenas as mensagens deste CAN ID
        mensagens = df[df["can_id_dec"] == can_id]

        for linha in mensagens.itertuples(index=False):
            # Reconstrói o payload de 8 bytes a partir das colunas individuais
            payload = bytes([linha.b0, linha.b1, linha.b2, linha.b3,
                             linha.b4, linha.b5, linha.b6, linha.b7])
            timestamp = float(linha.timestamp_unix)

            for nome_sinal in sinais_do_id:
                _, byte_ini, byte_comp, com_sinal, divisor, offset, unidade, prio, *_ = SINAIS_SESSAO[nome_sinal]
                valor = decodificar_payload(payload, byte_ini, byte_comp, com_sinal, divisor, offset)
                if valor is not None:
                    resultado[nome_sinal].append(
                        montar_linha_csv(nome_sinal, timestamp, can_id, prio, valor, unidade)
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

    Para cada sinal:
      - Executa a validação física (range + delta).
      - Se a taxa de invalidade ≥ LIMIAR_INVALIDADE: gera arquivo .invalid com diagnóstico.
      - Caso contrário: salva o CSV com as amostras válidas.

    O arquivo .invalid substitui o CSV para sinalizar ao operador que o sinal
    não deve ser usado sem revisão da especificação CAN ou do firmware.
    """
    diretorio_saida.mkdir(parents=True, exist_ok=True)

    for nome_sinal, linhas in sinais.items():
        if not linhas:
            continue

        spec = mapa_especificacoes.get(nome_sinal)
        limite_min = spec[8]  if spec else None
        limite_max = spec[9]  if spec else None
        delta_max  = spec[10] if spec else None

        linhas_validas, metricas = validar_sinal(linhas, limite_min, limite_max, delta_max, nome_sinal)

        if metricas.get("suspeito"):
            # Sinal com alta taxa de rejeição → arquivo de alerta
            caminho_alerta = diretorio_saida / f"{nome_sinal}.invalid"
            n_t  = metricas["n_total"]
            n_i  = metricas["n_invalido"]
            pct  = metricas["ratio"] * 100
            caminho_alerta.write_text(
                f"SINAL INVÁLIDO: {nome_sinal}\n"
                f"  Total de amostras brutas : {n_t}\n"
                f"  Amostras rejeitadas       : {n_i} ({pct:.1f}%)\n"
                f"  Limiar de suspeita        : {LIMIAR_INVALIDADE * 100:.0f}%\n"
                f"\n"
                f"  Diagnóstico: a taxa de amostras fora do range físico ou com\n"
                f"  variação brusca entre frames (Δ > {delta_max}) supera o limiar.\n"
                f"  Provávelmente o firmware grava outro dado nesses bytes durante\n"
                f"  operação (e.g. contador de encoder, dado de diagnóstico).\n"
                f"  Revise a especificação CAN ou o firmware antes de usar este sinal.\n"
            )
            print(f"  [INVÁLIDO] {nome_sinal:<22}  {n_i}/{n_t} amostras rejeitadas ({pct:.1f}%)  →  {nome_sinal}.invalid")

        elif linhas_validas:
            n_t    = metricas.get("n_total", len(linhas_validas))
            n_i    = metricas.get("n_invalido", 0)
            pct_ok = (len(linhas_validas) / n_t * 100) if n_t else 0
            df_saida = pd.DataFrame(linhas_validas)
            df_saida.to_csv(diretorio_saida / f"{nome_sinal}.csv", index=False)
            if n_i > 0:
                print(f"  [FILTRADO] {nome_sinal:<22}  {len(linhas_validas)}/{n_t} pts salvos ({pct_ok:.1f}% válidos)")


# ── Ponto de entrada ──────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  EXTRATOR DE TELEMETRIA CAN — SESSION FILES")
    print("=" * 60)
    print(f"  DIR_BASE   : {DIR_BASE}")
    print(f"  SESSION_DIR: {DIR_SESSOES}  {'[OK]' if DIR_SESSOES.exists() else '[NAO ENCONTRADO]'}")
    print(f"  OUT_DIR    : {DIR_SAIDA}")
    print("=" * 60)

    if not DIR_SESSOES.exists():
        print(f"\n[ERRO] Pasta de sessões não encontrada: {DIR_SESSOES}")
        print("       Verifique se a estrutura data/raw/sessioncsvFiles existe")
        print("       a partir do diretório onde o script está localizado.")
        return

    arquivos_csv = sorted(DIR_SESSOES.glob("*.csv"))
    if not arquivos_csv:
        print("\n[ERRO] Nenhum arquivo .csv encontrado em SESSION_DIR.\n")
        return

    for caminho_arquivo in arquivos_csv:
        print(f"\n[session] {caminho_arquivo.name}  ({caminho_arquivo.stat().st_size / 1024:.0f} KB)")
        if not arquivo_tem_tamanho_valido(caminho_arquivo):
            continue

        sinais_brutos = processar_arquivo_sessao(caminho_arquivo)
        diretorio_saida = DIR_SAIDA / caminho_arquivo.stem
        salvar_sinais(sinais_brutos, SINAIS_SESSAO, diretorio_saida)

        # Imprime estatísticas de frequência e duração para cada sinal válido
        for nome_sinal, linhas in sinais_brutos.items():
            if not linhas:
                continue
            spec = SINAIS_SESSAO[nome_sinal]
            linhas_validas, metricas = validar_sinal(linhas, spec[8], spec[9], spec[10], nome_sinal)
            if metricas.get("suspeito"):
                continue
            if linhas_validas:
                duracao = linhas_validas[-1]["timestamp"] - linhas_validas[0]["timestamp"]
                frequencia_hz = len(linhas_validas) / duracao if duracao > 0 else 0
                n_rejeitadas = metricas.get("n_invalido", 0)
                aviso = f"  [warn: {n_rejeitadas} rejeitadas]" if n_rejeitadas else ""
                print(f"  {nome_sinal:<22}  {len(linhas_validas):>5} pts  |  {duracao:.1f} s  |  {frequencia_hz:.1f} Hz{aviso}")

    print("\nPronto. CSVs em data/processed/<arquivo>/<SINAL>.csv\n")
    print("Sinais suspeitos geram arquivo .invalid no mesmo diretório.\n")


if __name__ == "__main__":
    main()