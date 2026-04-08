"""
run_pipeline.py
---------------
Orquestra a pipeline completa de telemetria CAN em ordem:

  1. extratorSessionFiles.py   — session CSVs  → data/processed/
  2. extratorCandumpFiles.py   — candump logs   → data/processed/
  3. getVelocidade.py          — ACC CSVs       → VEL CSVs (automático)
  4. plotador.py               — todos os sinais → plots/

Uso:
  python3 src/run_pipeline.py                  # roda tudo
  python3 src/run_pipeline.py --skip-extract   # pula extratores (só velo + plot)
  python3 src/run_pipeline.py --skip-plot      # pula plotador
  python3 src/run_pipeline.py --only-plot      # só plota
"""

import sys
import subprocess
from pathlib import Path

# ── Caminhos ──────────────────────────────────────────────────────────────────

# O pulo do gato: .parent.parent faz o BASE_DIR ser a raiz do projeto
BASE_DIR = Path(__file__).resolve().parent.parent 
SRC_DIR  = BASE_DIR / "src"
PROC_DIR = BASE_DIR / "data" / "processed"

EXTRATOR_SESSION  = SRC_DIR / "extratorSessionFiles.py"
EXTRATOR_CANDUMP  = SRC_DIR / "extratorCandumpFiles.py"
GET_VELOCIDADE    = SRC_DIR / "getVelocidade.py"
PLOTADOR          = SRC_DIR / "plotador.py"

# ── Helpers ───────────────────────────────────────────────────────────────────

def run(script: Path, *args: str) -> bool:
    """Executa um script Python e retorna True se bem-sucedido."""
    cmd = [sys.executable, str(script), *args]
    print(f"\n{'='*60}")
    print(f"  Executando: {script.name} {' '.join(args)}")
    print(f"{'='*60}")
    result = subprocess.run(cmd, cwd=str(BASE_DIR))
    if result.returncode != 0:
        print(f"\n[ERRO] {script.name} terminou com código {result.returncode}")
        return False
    return True


def find_acc_csvs() -> list[Path]:
    """Encontra todos os CSVs de aceleração gerados pelos extratores."""
    if not PROC_DIR.exists():
        return []
    return sorted(PROC_DIR.glob("**/VENTOR_LINEAR_ACC_*.csv"))

# ── Etapas ────────────────────────────────────────────────────────────────────

def step_extract_session() -> bool:
    return run(EXTRATOR_SESSION)


def step_extract_candump() -> bool:
    return run(EXTRATOR_CANDUMP)


def step_velocidade() -> bool:
    acc_csvs = find_acc_csvs()

    if not acc_csvs:
        print("\n[velo] Nenhum CSV de aceleração encontrado em data/processed/.")
        print("       Verifique se os extratores rodaram corretamente.")
        return False

    print(f"\n{'='*60}")
    print(f"  getVelocidade — {len(acc_csvs)} arquivo(s) de aceleração encontrado(s)")
    print(f"{'='*60}")
    for p in acc_csvs:
        print(f"  {p.relative_to(BASE_DIR)}")

    # Passa todos os CSVs de uma vez para o getVelocidade
    result = subprocess.run(
        [sys.executable, str(GET_VELOCIDADE), *[str(p) for p in acc_csvs]],
        cwd=str(BASE_DIR)
    )
    return result.returncode == 0


def step_plot() -> bool:
    return run(PLOTADOR)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = set(sys.argv[1:])

    skip_extract = "--skip-extract" in args or "--only-plot" in args
    skip_plot    = "--skip-plot"    in args
    only_plot    = "--only-plot"    in args

    steps = []

    if not skip_extract:
        steps.append(("Extrator Session",  step_extract_session))
        steps.append(("Extrator Candump",  step_extract_candump))
        steps.append(("Integrador Velo",   step_velocidade))

    if not skip_plot:
        steps.append(("Plotador",          step_plot))

    print("\n" + "="*60)
    print("  PIPELINE DE TELEMETRIA CAN")
    print("="*60)
    print(f"  Etapas: {', '.join(name for name, _ in steps)}")

    for name, fn in steps:
        ok = fn()
        if not ok:
            print(f"\n[PIPELINE INTERROMPIDA] Falha em: {name}")
            sys.exit(1)

    print("\n" + "="*60)
    print("  PIPELINE CONCLUÍDA")
    print("="*60)
    print(f"  Resultados em: {PROC_DIR}\n")


if __name__ == "__main__":
    main()