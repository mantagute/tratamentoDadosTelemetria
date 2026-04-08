"""
extrator_candump.py
-------------------
Lê arquivos no formato candump e gera um CSV por sinal no formato padrão:

    names, timestamp, id_can, prioridade, dado
    VENTOR_LINEAR_ACC_X, 946688473.19, 0x00000001, 1, -0.29 m/s²

Saída: data/processed/<nome_arquivo>/<SINAL>.csv

Sinais atualmente mapeados
──────────────────────────
ID 0x00000001 — acceleration_vector_x_y_1:
  VENTOR_LINEAR_ACC_X    b[0:2]  int16 LE  ×0.01  m/s²   (aceleração lateral)
  VENTOR_LINEAR_ACC_Y    b[4:6]  int16 LE  ×0.01  m/s²   (aceleração longitudinal)

ID 0x18FF1515 — VCU_DATA_OUT:
  APS_PERC               b[2:4]  uint16 LE  /100   %      (posição do pedal de acelerador)

Para adicionar novos sinais, basta inserir entradas em CANDUMP_SIGNALS seguindo
o mesmo padrão: nome → (can_id_int, byte_start, byte_len, signed, multiplier,
offset, unidade, prioridade, phys_min, phys_max, delta_max_per_frame)

Formato candump esperado:
  (timestamp) interface CANID#HEXDATA
  ex: (0946688473.192390) can0 00000001#E3FF000005000000

Validação física:
  Mesma lógica do extrator.py — amostras fora do range ou com Δ > delta_max
  entre frames consecutivos são descartadas. Sinais com taxa de invalidade
  ≥ INVALID_RATIO_THRESHOLD geram arquivo .invalid no lugar do CSV.
"""

import re
import struct
import pandas as pd
import numpy as np
from pathlib import Path

# ── Configuração ──────────────────────────────────────────────────────────────

MIN_SIZE_KB             = 1      # candump costuma ser menor que session CSV
INVALID_RATIO_THRESHOLD = 0.20   # ≥20% de amostras inválidas → sinal suspeito

def _find_base_dir() -> Path:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir,
        script_dir.parent,
        Path.cwd(),
        Path.cwd().parent,
    ]
    for c in candidates:
        if (c / "data").exists():
            return c
    return Path.cwd()

BASE_DIR     = _find_base_dir()
CANDUMP_DIR  = BASE_DIR / "data" / "raw" / "candumpFiles"
OUT_DIR      = BASE_DIR / "data" / "processed"

# ── Mapa de sinais ────────────────────────────────────────────────────────────
# nome → (can_id_int, byte_start, byte_len, signed, multiplier, offset,
#          unidade, prioridade, phys_min, phys_max, delta_max_per_frame)
#
# Fórmula: valor_real = raw * multiplier + offset
# delta_max_per_frame: None = sem filtro de variação entre frames

CANDUMP_SIGNALS = {
    #                          can_id      bs bl  sgn    mult   off  unit     prio  phys_min  phys_max  delta
    "VENTOR_LINEAR_ACC_X":  (0x00000001,  0,  2, True,  0.01,  0.0, "m/s²",  1,   -20.0,    20.0,     None),
    "VENTOR_LINEAR_ACC_Y":  (0x00000001,  4,  2, True,  0.01,  0.0, "m/s²",  1,   -20.0,    20.0,     None),
    # VCU — pedal de acelerador
    # bit(16-31) = bytes 2-3, little-endian, unsigned
    # multiplier=0.01 (spec usa divisor 100), sem offset, range 0–100 %
    "APS_PERC":             (0x18FF1515,  2,  2, False, 0.01,  0.0, "%",     1,    0.0,     100.0,    None),
}

# ── Helpers ───────────────────────────────────────────────────────────────────

# Regex para linha candump: (timestamp) interface CANID#HEXDATA
_LINE_RE = re.compile(
    r"^\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)\s*$"
)

_STRUCT_FMT = {
    (1, True):  "b",
    (1, False): "B",
    (2, True):  "h",
    (2, False): "H",
}


def decode(data: bytes, bs: int, bl: int, signed: bool,
           mult: float, offset: float) -> float | None:
    fmt = _STRUCT_FMT.get((bl, signed))
    if fmt is None:
        return None
    if bs + bl > len(data):
        return None
    try:
        raw = struct.unpack_from("<" + fmt, data, bs)[0]
        return round(raw * mult + offset, 4)
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
    n    = len(vals)
    mask = np.ones(n, dtype=bool)

    if phys_min is not None:
        mask &= vals >= phys_min
    if phys_max is not None:
        mask &= vals <= phys_max

    if delta_max is not None and n > 1:
        diffs = np.abs(np.diff(vals))
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

# ── Leitura do candump ────────────────────────────────────────────────────────

def process_candump(fp: Path) -> dict[str, list]:
    """Lê o arquivo candump linha a linha e extrai os sinais mapeados."""

    # agrupa sinais por CAN ID para uma única passagem no arquivo
    by_can: dict[int, list[str]] = {}
    for sig, (can_id, *_) in CANDUMP_SIGNALS.items():
        by_can.setdefault(can_id, []).append(sig)

    result = {s: [] for s in CANDUMP_SIGNALS}

    with fp.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = _LINE_RE.match(line.strip())
            if not m:
                continue

            ts_str, id_str, hex_data = m.groups()
            can_id_int = int(id_str, 16)

            sigs = by_can.get(can_id_int)
            if not sigs:
                continue

            ts = float(ts_str)
            try:
                payload = bytes.fromhex(hex_data)
            except ValueError:
                # Ignora a linha se o payload contiver caracteres inválidos (ex: frames RTR ou Error)
                continue

            for sig in sigs:
                _, bs, bl, sgn, mult, off, unit, prio, *_ = CANDUMP_SIGNALS[sig]
                val = decode(payload, bs, bl, sgn, mult, off)
                if val is not None:
                    result[sig].append(to_row(sig, ts, can_id_int, prio, val, unit))

    return result

# ── Validação e salvamento ────────────────────────────────────────────────────

def save(signals: dict[str, list], spec_map: dict, out_dir: Path) -> None:
    """
    Valida cada sinal e salva o CSV. Sinais suspeitos geram arquivo .invalid.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    for sig, rows in signals.items():
        if not rows:
            continue

        spec      = spec_map.get(sig)
        phys_min  = spec[8]  if spec else None
        phys_max  = spec[9]  if spec else None
        delta_max = spec[10] if spec else None

        valid_rows, info = validate_signal(rows, phys_min, phys_max, delta_max, sig)

        if info.get("suspicious"):
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
                f"  Revise a especificação CAN ou o firmware antes de usar este sinal.\n"
            )
            print(f"  [INVÁLIDO] {sig:<26}  {n_i}/{n_t} amostras rejeitadas ({pct:.1f}%)  →  {sig}.invalid")

        elif valid_rows:
            n_t    = info.get("n_total", len(valid_rows))
            n_i    = info.get("n_invalid", 0)
            pct_ok = (len(valid_rows) / n_t * 100) if n_t else 0
            df = pd.DataFrame(valid_rows)
            df.to_csv(out_dir / f"{sig}.csv", index=False)
            if n_i > 0:
                print(f"  [FILTRADO] {sig:<26}  {len(valid_rows)}/{n_t} pts salvos ({pct_ok:.1f}% válidos)")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  EXTRATOR DE TELEMETRIA CAN — CANDUMP")
    print("=" * 60)
    print(f"  BASE_DIR    : {BASE_DIR}")
    print(f"  CANDUMP_DIR : {CANDUMP_DIR}  {'[OK]' if CANDUMP_DIR.exists() else '[NAO ENCONTRADO]'}")
    print(f"  OUT_DIR     : {OUT_DIR}")
    print("=" * 60)

    if not CANDUMP_DIR.exists():
        print(f"\n[ERRO] Pasta de candump não encontrada: {CANDUMP_DIR}")
        print("       Crie a pasta data/raw/candumpFiles e coloque os arquivos .log lá.")
        return

    log_files = sorted(CANDUMP_DIR.glob("*.log"))
    if not log_files:
        print("\n[ERRO] Nenhum arquivo .log encontrado em CANDUMP_DIR.\n")
        return

    for fp in log_files:
        print(f"\n[candump] {fp.name}  ({fp.stat().st_size/1024:.0f} KB)")
        if not size_ok(fp):
            continue

        raw_signals = process_candump(fp)
        out_dir     = OUT_DIR / fp.stem
        save(raw_signals, CANDUMP_SIGNALS, out_dir)

        for sig, rows in raw_signals.items():
            if not rows:
                continue
            spec        = CANDUMP_SIGNALS[sig]
            valid, info = validate_signal(rows, spec[8], spec[9], spec[10], sig)
            if info.get("suspicious"):
                continue
            if valid:
                dur  = valid[-1]["timestamp"] - valid[0]["timestamp"]
                freq = len(valid) / dur if dur > 0 else 0
                n_i  = info.get("n_invalid", 0)
                flag = f"  [warn: {n_i} rejeitadas]" if n_i else ""
                print(f"  {sig:<26}  {len(valid):>6} pts  |  {dur:.1f} s  |  {freq:.1f} Hz{flag}")

    print("\nPronto. CSVs em data/processed/<arquivo>/<SINAL>.csv\n")
    print("Sinais suspeitos geram arquivo .invalid no mesmo diretório.\n")


if __name__ == "__main__":
    main()