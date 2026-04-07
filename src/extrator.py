"""
extrator.py
-----------
Lê arquivos brutos e gera um CSV por sinal no formato padrão:

    names, timestamp, id_can, prioridade, dado
    ACT_TORQUE_A13, 946688468.12, 0x18FF01F7, 1, 25.00 Nm

Saída: data/processed/<nome_arquivo>/<SINAL>.csv

Sinais
──────
Session CSV:
  ACT_SPEED_A13  / B13   rpm   b[1:3] signed  /1     offset 0
  ACT_TORQUE_A13 / B13   Nm    b[3:5] signed  /5     offset -6400
  ACT_POWER_A13  / B13   kW    b[5:7] signed  /200   offset -160
  ACT_TEMP_A13   / B13   °C    b[6]   unsigned /1     offset -40


Nota sobre encode:
  Todos os sinais de valor físico usam signed 16-bit.
  Fórmula: valor_real = raw / divisor + offset
  Byte 0 de cada mensagem contém bits de status e é ignorado nos sinais de valor.
  Os dados são little-endian (byte menos significativo primeiro).
  Exceção: temperatura usa unsigned 8-bit + offset -40.

Validação física (IMPORTANTE):
  Cada sinal tem limites de range e de variação máxima entre frames
  consecutivos (Δ_max). Amostras fora do range ou com Δ > Δ_max são
  marcadas como NaN e não são salvas no CSV final.

  Para ACT_SPEED em particular: os bytes b[1:3] do firmware apresentam
  oscilações de ~16.000 rpm entre frames consecutivos a 10 Hz durante
  a fase de operação — fisicamente impossível para qualquer motor. Isso
  indica que o firmware grava outro dado (posição de encoder, contador
  de comutação ou dado de diagnóstico) nesses bytes enquanto opera sob
  carga. A validação descarta essas amostras; se a taxa de rejeição
  superar INVALID_RATIO_THRESHOLD, o sinal inteiro é marcado como
  inválido e um arquivo .invalid é gerado no lugar do CSV.
"""

import struct
import pandas as pd
import numpy as np
from pathlib import Path

# ── Configuração ──────────────────────────────────────────────────────────────

MIN_SIZE_KB             = 20
INVALID_RATIO_THRESHOLD = 0.20   # ≥20 % de amostras inválidas → sinal suspeito

def _find_base_dir() -> Path:
    """
    Resolve BASE_DIR de forma robusta independente de onde o script foi colocado.
    Testa candidatos em ordem e usa o primeiro que contém data/sessioncsvFiles.
    Se nenhum for encontrado, usa o cwd e o main() reportará o caminho exato.
    """
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir,           # script na raiz do projeto
        script_dir.parent,    # script em src/ ou similar
        Path.cwd(),           # diretório de trabalho atual
        Path.cwd().parent,
    ]
    for c in candidates:
        if (c / "data" / "sessioncsvFiles").exists():
            return c
    return Path.cwd()

BASE_DIR    = _find_base_dir()
SESSION_DIR = BASE_DIR / "data" / "raw" / "sessioncsvFiles"
OUT_DIR     = BASE_DIR / "data" / "processed"

# ── Mapa de sinais ────────────────────────────────────────────────────────────
# nome → (can_id_int, byte_start, byte_len, signed, divisor, offset, unidade,
#          prioridade, phys_min, phys_max, delta_max_per_frame)
#
# Fórmula: valor_real = raw / divisor + offset
#
# Byte 0 de cada mensagem = bits de status → byte_start começa em 1
#
# delta_max_per_frame: variação física máxima aceitável entre dois frames
# consecutivos. Para 10 Hz, regra conservadora = 10 % do range por frame.
# None = sem filtro de variação.

SESSION_SIGNALS = {
    #                  can_id      bs bl  sgn   div    off     unit  prio phys_min  phys_max  delta
    "ACT_SPEED_A13":  (0x18FF01F7, 1, 2, False,  1,    -32000,     "rpm", 1, -32000,   33535,    2000),
    "ACT_TORQUE_A13": (0x18FF01F7, 3, 2, False,  5,    -6400,  "Nm",  1, -6400,    6707,     None),
    "ACT_POWER_A13":  (0x18FF01F7, 5, 2, False,  200,  -160,   "kW",  1, -160,     167.675,  None),
    "ACT_TEMP_A13":   (0x18FF01F7, 7, 1, False, 1,    -40,    "°C",  1, -40,      215,      None),
    "ACT_SPEED_B13":  (0x18FF02F7, 1, 2, False,  1,    -32000,     "rpm", 1, -32000,   33535,    2000),
    "ACT_TORQUE_B13": (0x18FF02F7, 3, 2, False,  5,    -6400,  "Nm",  1, -6400,    6707,     None),
    "ACT_POWER_B13":  (0x18FF02F7, 5, 2, False,  200,  -160,   "kW",  1, -160,     167.675,  None),
    "ACT_TEMP_B13":   (0x18FF02F7, 7, 1, False, 1,    -40,    "°C",  1, -40,      215,      None),
    "SETP_TORQUE_A13": (0x18FFE180, 6, 2, False, 5,    -6400,  "Nm",  1, -6400,    6707,     None),
    "SETP_TORQUE_B13": (0x18FFE280, 6, 2, False, 5,    -6400,  "Nm",  1, -6400,    6707,     None),
}


# ── Helpers ───────────────────────────────────────────────────────────────────

_STRUCT_FMT = {
    (1, True):  "b",
    (1, False): "B",
    (2, True):  "h",
    (2, False): "H",
}


def decode(data: bytes, bs: int, bl: int, signed: bool,
           div: float, offset: float) -> float | None:
    fmt = _STRUCT_FMT.get((bl, signed))
    if fmt is None:
        return None
    try:
        raw = struct.unpack_from("<" + fmt, data, bs)[0]
        return round((raw / div) + offset, 4)
    except Exception:
        return None


def size_ok(fp: Path) -> bool:
    kb = fp.stat().st_size / 1024
    if kb < MIN_SIZE_KB:
        print(f"  [skip] {fp.name}  ({kb:.1f} KB < {MIN_SIZE_KB} KB mínimo)")
        return False
    return True


def to_row(name: str, ts: float, can_id: int,
           prio: int, val: float, unit: str) -> dict:
    return {
        "names":      name,
        "timestamp":  round(ts, 6),
        "id_can":     f"0x{can_id:08X}",
        "prioridade": prio,
        "dado":       f"{val:.2f} {unit}",
    }


def validate_signal(rows: list[dict], phys_min: float | None,
                    phys_max: float | None, delta_max: float | None,
                    sig_name: str) -> tuple[list[dict], dict]:
    """
    Filtra amostras fora do range físico ou com variação brusca entre frames.

    Retorna (linhas_válidas, info_qualidade).
    """
    if not rows:
        return rows, {}

    vals = np.array([float(r["dado"].split()[0]) for r in rows])
    ts   = np.array([r["timestamp"] for r in rows])
    n    = len(vals)

    # Máscara de validade (True = manter)
    mask = np.ones(n, dtype=bool)

    # 1) Range físico
    if phys_min is not None:
        mask &= vals >= phys_min
    if phys_max is not None:
        mask &= vals <= phys_max

    # 2) Taxa de variação entre frames consecutivos
    if delta_max is not None and n > 1:
        diffs = np.abs(np.diff(vals))
        # marca o frame 'depois' da variação brusca (mais conservador)
        rapid = np.concatenate([[False], diffs > delta_max])
        mask &= ~rapid

    n_invalid = int((~mask).sum())
    ratio     = n_invalid / n

    info = {
        "n_total":    n,
        "n_invalid":  n_invalid,
        "ratio":      ratio,
        "suspicious": ratio >= INVALID_RATIO_THRESHOLD,
    }

    valid_rows = [r for r, keep in zip(rows, mask) if keep]
    return valid_rows, info

# ── Session CSV ───────────────────────────────────────────────────────────────

def process_session(fp: Path) -> dict[str, list]:
    df = pd.read_csv(fp, skiprows=1)
    df.columns = [c.strip() for c in df.columns]

    by_can: dict[int, list[str]] = {}
    for sig, (can_id, *_) in SESSION_SIGNALS.items():
        by_can.setdefault(can_id, []).append(sig)

    result = {s: [] for s in SESSION_SIGNALS}

    for can_id, sigs in by_can.items():
        subset = df[df["can_id_dec"] == can_id]
        for row in subset.itertuples(index=False):
            payload = bytes([row.b0, row.b1, row.b2, row.b3,
                             row.b4, row.b5, row.b6, row.b7])
            ts = float(row.timestamp_unix)
            for sig in sigs:
                _, bs, bl, sgn, div, off, unit, prio, *_ = SESSION_SIGNALS[sig]
                val = decode(payload, bs, bl, sgn, div, off)
                if val is not None:
                    result[sig].append(to_row(sig, ts, can_id, prio, val, unit))

    return result



# ── Validação e salvamento ────────────────────────────────────────────────────

def save(signals: dict[str, list],
         spec_map: dict,
         out_dir: Path) -> None:
    """
    Valida cada sinal contra os limites físicos e Δ_max antes de salvar.
    Sinais com taxa de invalidade ≥ INVALID_RATIO_THRESHOLD recebem um
    arquivo .invalid descrevendo o problema em vez do CSV de dados.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    for sig, rows in signals.items():
        if not rows:
            continue

        spec = spec_map.get(sig)
        phys_min  = spec[8]  if spec else None
        phys_max  = spec[9]  if spec else None
        delta_max = spec[10] if spec else None

        valid_rows, info = validate_signal(
            rows, phys_min, phys_max, delta_max, sig
        )

        if info.get("suspicious"):
            # Não salva CSV — gera arquivo de alerta
            alert_path = out_dir / f"{sig}.invalid"
            n_t = info["n_total"]
            n_i = info["n_invalid"]
            pct = info["ratio"] * 100
            alert_path.write_text(
                f"SINAL INVÁLIDO: {sig}\n"
                f"  Total de amostras brutas : {n_t}\n"
                f"  Amostras rejeitadas       : {n_i} ({pct:.1f}%)\n"
                f"  Limiar de suspeita        : {INVALID_RATIO_THRESHOLD*100:.0f}%\n"
                f"\n"
                f"  Diagnóstico: a taxa de amostras fora do range físico ou com\n"
                f"  variação brusca entre frames (Δ > {delta_max}) supera o limiar.\n"
                f"  Provávelmente o firmware grava outro dado nesses bytes durante\n"
                f"  operação (e.g. contador de encoder, dado de diagnóstico).\n"
                f"  Revise a especificação CAN ou o firmware antes de usar este sinal.\n"
            )
            print(f"  [INVÁLIDO] {sig:<22}  {n_i}/{n_t} amostras rejeitadas ({pct:.1f}%)  →  {sig}.invalid")
        elif valid_rows:
            n_t = info.get("n_total", len(valid_rows))
            n_i = info.get("n_invalid", 0)
            pct_ok = (len(valid_rows) / n_t * 100) if n_t else 0
            df = pd.DataFrame(valid_rows)
            df.to_csv(out_dir / f"{sig}.csv", index=False)
            if n_i > 0:
                print(f"  [FILTRADO] {sig:<22}  {len(valid_rows)}/{n_t} pts salvos ({pct_ok:.1f}% válidos)")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  EXTRATOR DE TELEMETRIA CAN")
    print("=" * 60)
    print(f"  BASE_DIR   : {BASE_DIR}")
    print(f"  SESSION_DIR: {SESSION_DIR}  {'[OK]' if SESSION_DIR.exists() else '[NAO ENCONTRADO]'}")
    print(f"  OUT_DIR    : {OUT_DIR}")
    print("=" * 60)

    if not SESSION_DIR.exists():
        print(f"\n[ERRO] Pasta de sessoes nao encontrada: {SESSION_DIR}")
        print("       Verifique se a estrutura data/sessioncsvFiles existe")
        print("       a partir do diretorio onde o script esta localizado.")
        return

    for fp in sorted(SESSION_DIR.glob("*.csv")):
        print(f"\n[session] {fp.name}  ({fp.stat().st_size/1024:.0f} KB)")
        if not size_ok(fp):
            continue

        raw_signals = process_session(fp)
        save(raw_signals, SESSION_SIGNALS, OUT_DIR / fp.stem)

        for sig, rows in raw_signals.items():
            if not rows:
                continue
            spec       = SESSION_SIGNALS[sig]
            valid, info = validate_signal(
                rows, spec[8], spec[9], spec[10], sig
            )
            if info.get("suspicious"):
                continue  # já reportado dentro de save()
            if valid:
                dur = valid[-1]["timestamp"] - valid[0]["timestamp"]
                freq = len(valid) / dur if dur > 0 else 0
                n_i  = info.get("n_invalid", 0)
                flag = f"  [warn: {n_i} rejeitadas]" if n_i else ""
                print(f"  {sig:<22}  {len(valid):>5} pts  |  {dur:.1f} s  |  {freq:.1f} Hz{flag}")


    print("\nPronto. CSVs em data/processed/<arquivo>/<SINAL>.csv\n")
    print("Sinais suspeitos geram arquivo .invalid no mesmo diretório.\n")


if __name__ == "__main__":
    main()