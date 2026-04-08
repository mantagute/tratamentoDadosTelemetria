"""
runPipeline.py
==============
Orquestrador da pipeline completa de telemetria CAN.

VISÃO GERAL
-----------
Este script é o ponto de entrada único para processar uma sessão completa
de telemetria. Ele executa os módulos da pipeline na ordem correta:

    [1] extratorSessionFiles.py  → decodifica session CSVs
    [2] extratorCandumpFiles.py  → decodifica logs candump
    [3] getVelocidade.py         → integra aceleração → velocidade
    [4] plotador.py              → gera gráficos de todos os sinais

Cada etapa é executada como subprocesso independente, o que garante
isolamento de estado e facilita depuração (falhas ficam localizadas).

FLAGS DISPONÍVEIS
-----------------
  --skip-extract   Pula os extratores e o integrador (etapas 1, 2 e 3).
                   Útil quando os CSVs já foram gerados e você quer apenas
                   replotá-los após ajustar o plotador.

  --skip-plot      Pula o plotador (etapa 4). Útil para extrair dados sem
                   gerar gráficos (ex: ambiente sem display ou matplotlib).

  --only-plot      Atalho para --skip-extract: executa apenas o plotador.

USO
---
  python3 src/runPipeline.py                  # pipeline completa
  python3 src/runPipeline.py --skip-extract   # só plota dados já extraídos
  python3 src/runPipeline.py --skip-plot      # extrai sem plotar
  python3 src/runPipeline.py --only-plot      # plota sem extrair
"""

import sys
import subprocess
from pathlib import Path


# ── Caminhos dos módulos da pipeline ─────────────────────────────────────────

DIR_BASE = Path(__file__).resolve().parent.parent  # raiz do projeto
DIR_SRC  = DIR_BASE / "src"
DIR_PROCESSADO = DIR_BASE / "data" / "processed"

SCRIPT_EXTRATOR_SESSION  = DIR_SRC / "extratorSessionFiles.py"
SCRIPT_EXTRATOR_CANDUMP  = DIR_SRC / "extratorCandumpFiles.py"
SCRIPT_GET_VELOCIDADE    = DIR_SRC / "getVelocidade.py"
SCRIPT_PLOTADOR          = DIR_SRC / "plotador.py"


# ── Execução de subprocessos ──────────────────────────────────────────────────

def executar_script(script: Path, *argumentos: str) -> bool:
    """
    Executa um script Python como subprocesso e retorna True se bem-sucedido.

    Usa o mesmo executável Python do processo atual (sys.executable) para
    garantir compatibilidade com ambientes virtuais e conda.
    O diretório de trabalho é sempre a raiz do projeto (DIR_BASE).
    """
    comando = [sys.executable, str(script), *argumentos]
    print(f"\n{'=' * 60}")
    print(f"  Executando: {script.name} {' '.join(argumentos)}")
    print(f"{'=' * 60}")
    resultado = subprocess.run(comando, cwd=str(DIR_BASE))

    if resultado.returncode != 0:
        print(f"\n[ERRO] {script.name} terminou com código {resultado.returncode}")
        return False
    return True


def localizar_csvs_de_aceleracao() -> list[Path]:
    """
    Busca recursivamente todos os CSVs de aceleração gerados pelos extratores.

    Padrão de nome: VENTOR_LINEAR_ACC_*.csv em qualquer subpasta de processed/.
    Esses arquivos são os inputs do integrador de velocidade.
    """
    if not DIR_PROCESSADO.exists():
        return []
    return sorted(DIR_PROCESSADO.glob("**/VENTOR_LINEAR_ACC_*.csv"))


# ── Etapas da pipeline ────────────────────────────────────────────────────────

def etapa_extrair_sessoes() -> bool:
    """Etapa 1: decodifica arquivos CSV de sessão → sinais de inversores e VCU."""
    return executar_script(SCRIPT_EXTRATOR_SESSION)


def etapa_extrair_candump() -> bool:
    """Etapa 2: decodifica arquivos de log candump → sinais IMU e VCU."""
    return executar_script(SCRIPT_EXTRATOR_CANDUMP)


def etapa_calcular_velocidade() -> bool:
    """
    Etapa 3: integra sinais de aceleração IMU para obter velocidade.

    Localiza todos os CSVs de aceleração gerados nas etapas anteriores
    e passa todos de uma vez para o getVelocidade.py, que processa
    cada arquivo independentemente.
    """
    csvs_aceleracao = localizar_csvs_de_aceleracao()

    if not csvs_aceleracao:
        print("\n[velo] Nenhum CSV de aceleração encontrado em data/processed/.")
        print("       Verifique se os extratores rodaram corretamente.")
        return False

    print(f"\n{'=' * 60}")
    print(f"  getVelocidade — {len(csvs_aceleracao)} arquivo(s) de aceleração encontrado(s)")
    print(f"{'=' * 60}")
    for caminho in csvs_aceleracao:
        print(f"  {caminho.relative_to(DIR_BASE)}")

    resultado = subprocess.run(
        [sys.executable, str(SCRIPT_GET_VELOCIDADE), *[str(p) for p in csvs_aceleracao]],
        cwd=str(DIR_BASE),
    )
    return resultado.returncode == 0


def etapa_plotar() -> bool:
    """Etapa 4: gera gráficos de todos os sinais em data/processed/."""
    return executar_script(SCRIPT_PLOTADOR)


# ── Ponto de entrada ──────────────────────────────────────────────────────────

def main():
    flags = set(sys.argv[1:])

    # Resolve combinações de flags
    pular_extratores = "--skip-extract" in flags or "--only-plot" in flags
    pular_plotador   = "--skip-plot" in flags

    # Monta a sequência de etapas com base nas flags
    etapas = []

    if not pular_extratores:
        etapas.append(("Extrator Session",  etapa_extrair_sessoes))
        etapas.append(("Extrator Candump",  etapa_extrair_candump))
        etapas.append(("Integrador Velo",   etapa_calcular_velocidade))

    if not pular_plotador:
        etapas.append(("Plotador",          etapa_plotar))

    print("\n" + "=" * 60)
    print("  PIPELINE DE TELEMETRIA CAN")
    print("=" * 60)
    print(f"  Etapas: {', '.join(nome for nome, _ in etapas)}")

    # Executa cada etapa em sequência; interrompe na primeira falha
    for nome_etapa, funcao_etapa in etapas:
        sucesso = funcao_etapa()
        if not sucesso:
            print(f"\n[PIPELINE INTERROMPIDA] Falha em: {nome_etapa}")
            sys.exit(1)

    print("\n" + "=" * 60)
    print("  PIPELINE CONCLUÍDA COM SUCESSO")
    print("=" * 60)
    print(f"  Resultados em: {DIR_PROCESSADO}\n")


if __name__ == "__main__":
    main()