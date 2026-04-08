# CAN Telemetry Pipeline

Pipeline de processamento de telemetria automotiva CAN em Python. Lê arquivos brutos capturados no barramento CAN (session CSVs e logs candump), decodifica sinais físicos dos inversores de tração, IMU e VCU, integra aceleração em velocidade e gera gráficos analíticos por sinal.

---

## Índice

- [CAN Telemetry Pipeline](#can-telemetry-pipeline)
  - [Índice](#índice)
  - [Visão Geral](#visão-geral)
  - [Estrutura do Projeto](#estrutura-do-projeto)
  - [Pré-requisitos](#pré-requisitos)
  - [Início Rápido](#início-rápido)
  - [Fontes de Dados](#fontes-de-dados)
    - [Session CSVs](#session-csvs)
    - [Logs Candump](#logs-candump)
  - [Módulos](#módulos)
    - [extratorSessionFiles.py](#extratorsessionfilespy)
    - [extratorCandumpFiles.py](#extratorcandumpfilespy)
    - [getVelocidade.py](#getvelocidadepy)
    - [plotador.py](#plotadorpy)
    - [runPipeline.py](#runpipelinepy)
  - [Formato CSV de Saída](#formato-csv-de-saída)
  - [Validação Física](#validação-física)
  - [Adicionando Novos Sinais](#adicionando-novos-sinais)
  - [Diagnóstico de Erros](#diagnóstico-de-erros)

---

## Visão Geral

```
Dados brutos                Pipeline                    Saídas
─────────────               ────────                    ───────
session CSVs    ──►  extratorSessionFiles  ──►  CSVs por sinal
candump .logs   ──►  extratorCandumpFiles  ──►  CSVs por sinal  ──►  plotador  ──►  .png por sinal
                              │
                         getVelocidade
                              │
                        CSVs de velocidade
```

O barramento CAN de um veículo elétrico transporta dezenas de mensagens por segundo: RPM, torque e temperatura dos inversores, posição do pedal, dados de IMU, entre outros. Este projeto oferece uma pipeline modular que:

1. **Decodifica** bytes brutos em valores físicos com a fórmula correta por sinal.
2. **Valida** cada amostra contra limites físicos e taxa de variação máxima, descartando medições corrompidas.
3. **Integra** aceleração IMU em velocidade com correção de bias e filtro anti-drift.
4. **Visualiza** cada sinal em gráfico individual com tema escuro, pronto para relatório.

---

## Estrutura do Projeto

```
project-root/
│
├── src/
│   ├── extratorSessionFiles.py   # decodifica session CSVs → sinais de inversores/VCU
│   ├── extratorCandumpFiles.py   # decodifica logs candump → sinais IMU/VCU
│   ├── getVelocidade.py          # integra aceleração → velocidade
│   ├── plotador.py               # plota todos os sinais em data/processed/
│   └── runPipeline.py            # orquestrador: executa toda a pipeline
│
└── data/
    ├── raw/
    │   ├── sessioncsvFiles/      # ← coloque os session CSVs aqui
    │   └── candumpFiles/         # ← coloque os .log do candump aqui
    │
    └── processed/
        └── <nome_arquivo>/
            ├── SINAL_A.csv
            ├── SINAL_B.csv
            ├── SINAL_X.invalid   # sinal rejeitado (ver Validação Física)
            └── plots/
                ├── SINAL_A.png
                ├── SINAL_B.png
                └── SINAL_X.png   # cartão de erro vermelho
```

---

## Pré-requisitos

**Python 3.10+** (usa `float | None` como type hint nativo)

```bash
pip install pandas numpy scipy matplotlib
```

Sem outras dependências externas. A pipeline não requer banco de dados, servidor ou configuração adicional.

---

## Início Rápido

```bash
# 1. Clone o repositório e entre na pasta
git clone <repo-url>
cd <repo>

# 2. Coloque seus arquivos de dados
cp sua_sessao.csv   data/raw/sessioncsvFiles/
cp seu_candump.log  data/raw/candumpFiles/

# 3. Execute a pipeline completa
python3 src/runPipeline.py

# Os resultados estarão em:
# data/processed/<nome_arquivo>/<SINAL>.csv
# data/processed/<nome_arquivo>/plots/<SINAL>.png
```

**Flags úteis:**

```bash
# Só gráficos (dados já extraídos anteriormente)
python3 src/runPipeline.py --only-plot

# Extrai dados sem gerar gráficos
python3 src/runPipeline.py --skip-plot

# Gráfico de uma pasta específica
python3 src/plotador.py candump-1999-12-31
```

---

## Fontes de Dados

### Session CSVs

Formato gerado pelo datalogger embarcado. Cada linha é uma mensagem CAN com colunas:

```
timestamp_unix, can_id_dec, b0, b1, b2, b3, b4, b5, b6, b7
946688468.120000, 418886135, 0, 200, 1, 176, 49, 0, 88, 0
```

A primeira linha do arquivo é um cabeçalho de metadados e é ignorada (`skiprows=1`).

### Logs Candump

Formato gerado pelo utilitário `candump` do pacote `can-utils` do Linux:

```
(0946688473.192390) can0 00000001#E3FF000005000000
```

Estrutura de cada linha: `(timestamp) interface CANID#HEXDATA`

- `timestamp`: Unix epoch com microssegundos
- `interface`: nome da interface CAN (ex: `can0`, `vcan0`)
- `CANID`: identificador hexadecimal de 8 dígitos (extended frame)
- `HEXDATA`: payload de até 8 bytes em hexadecimal

---

## Módulos

### extratorSessionFiles.py

**Responsabilidade:** Ler os session CSVs e decodificar os sinais dos inversores de tração (A13 e B13) e dos setpoints de torque do VCU.

**Sinais extraídos:**

| Sinal           | CAN ID     | Bytes | Tipo   | Fórmula              | Unidade |
|-----------------|------------|-------|--------|----------------------|---------|
| ACT_SPEED_A13   | 0x18FF01F7 | 1–2   | uint16 | raw − 32000          | rpm     |
| ACT_TORQUE_A13  | 0x18FF01F7 | 3–4   | uint16 | raw / 5 − 6400       | Nm      |
| ACT_POWER_A13   | 0x18FF01F7 | 5–6   | uint16 | raw / 200 − 160      | kW      |
| ACT_TEMP_A13    | 0x18FF01F7 | 7     | uint8  | raw − 40             | °C      |
| *(idem para B13)* | 0x18FF02F7 | — | —   | —                    | —       |
| SETP_TORQUE_A13 | 0x18FFE180 | 6–7   | uint16 | raw / 5 − 6400       | Nm      |
| SETP_TORQUE_B13 | 0x18FFE280 | 6–7   | uint16 | raw / 5 − 6400       | Nm      |

**Nota sobre ACT_SPEED:** Durante operação sob carga, os bytes b[1:3] dos inversores apresentam saltos de ~16.000 rpm entre frames consecutivos — fisicamente impossível. O firmware parece gravar dados de diagnóstico (encoder, contador de comutação) nesses bytes nessa condição. O filtro `delta_max = 2000 rpm/frame` descarta essas amostras. Se a taxa de rejeição superar 20%, um arquivo `.invalid` é gerado.

**Uso isolado:**
```bash
python3 src/extratorSessionFiles.py
```

---

### extratorCandumpFiles.py

**Responsabilidade:** Parsear arquivos `.log` no formato candump e decodificar sinais de IMU e VCU.

**Sinais extraídos:**

| Sinal               | CAN ID     | Bytes | Tipo  | Fórmula    | Unidade |
|---------------------|------------|-------|-------|------------|---------|
| VENTOR_LINEAR_ACC_X | 0x00000001 | 0–1   | int16 | raw × 0.01 | m/s²    |
| VENTOR_LINEAR_ACC_Y | 0x00000001 | 4–5   | int16 | raw × 0.01 | m/s²    |
| APS_PERC            | 0x18FF1515 | 2–3   | uint16| raw × 0.01 | %       |

**Diferença de decodificação em relação ao extrator de sessão:** os sinais IMU usam multiplicador (×) em vez de divisor (/), pois a especificação do sensor define o fator dessa forma.

**Uso isolado:**
```bash
python3 src/extratorCandumpFiles.py
```

---

### getVelocidade.py

**Responsabilidade:** Transformar sinais de aceleração IMU em sinais de velocidade por integração numérica.

**Por que não integrar diretamente?**

A aceleração bruta de uma IMU não pode ser integrada de forma ingênua. Três problemas fundamentais:

- **Bias do sensor:** offsets de fabricação e temperatura fazem com que a aceleração lida em repouso não seja exatamente zero. Mesmo um bias de 0.01 m/s² acumula 36 m/s de erro em 1 hora.
- **Ruído de alta frequência:** vibrações mecânicas, EMI e quantização do ADC introduzem energia em frequências acima da dinâmica de interesse. A integração amplifica esses componentes.
- **Drift linear:** erros sistemáticos residuais se acumulam como uma tendência linear crescente na velocidade.

**Pipeline de correção:**

```
Aceleração bruta
      │
      ▼
[1] Correção de bias
    Estima o offset estático usando a média das amostras de menor magnitude
    absoluta (percentil 5% ≈ momentos de quasi-repouso).
      │
      ▼
[2] Filtro Butterworth passa-baixa
    4ª ordem, cutoff 3 Hz, aplicado com filtfilt (fase zero).
    Remove ruído sem introduzir atraso de fase.
      │
      ▼
[3] Integração trapezoidal
    Usa timestamps reais: vel[i] = vel[i-1] + 0.5 × (acc[i] + acc[i-1]) × Δt[i]
    Robusto a taxas de amostragem irregulares e lacunas no log.
      │
      ▼
[4] Remoção de drift linear
    Subtrai uma rampa linear do valor inicial ao valor final da velocidade.
    Assume que o deslocamento líquido da sessão é ~0 (teste em pista fechada).
      │
      ▼
Velocidade (m/s) → CSV de saída
```

**Convenção de nomes:**
```
VENTOR_LINEAR_ACC_X  →  VENTOR_LINEAR_VEL_X
VENTOR_LINEAR_ACC_Y  →  VENTOR_LINEAR_VEL_Y
```

**Uso isolado:**
```bash
python3 src/getVelocidade.py data/processed/candump-xyz/VENTOR_LINEAR_ACC_Y.csv

# Múltiplos arquivos
python3 src/getVelocidade.py data/processed/candump-xyz/VENTOR_LINEAR_ACC_*.csv
```

---

### plotador.py

**Responsabilidade:** Varrer `data/processed/` e gerar um gráfico `.png` por sinal em cada pasta.

**Dois tipos de gráfico:**

1. **Sinal válido (CSV):** série temporal com eixo X em tempo relativo (t − t₀ em segundos). Exibe estatísticas no canto: n, frequência, mín, máx e média.

2. **Sinal inválido (.invalid):** "cartão de erro" em vermelho exibindo o diagnóstico do arquivo `.invalid`. Garante que problemas fiquem visíveis no relatório, em vez de silenciosamente omitidos.

**Estilo:** tema escuro GitHub Dark, com cores diferenciadas por tipo de sinal (azul = velocidade/RPM, verde = torque, laranja = potência, vermelho = temperatura, amarelo = pedal).

**Uso isolado:**
```bash
python3 src/plotador.py                    # todos os arquivos
python3 src/plotador.py session_0055       # filtra por nome de pasta
python3 src/plotador.py candump-1999-12-31
```

---

### runPipeline.py

**Responsabilidade:** Orquestrar a execução sequencial de todos os módulos.

Cada módulo é executado como subprocesso independente — falhas ficam localizadas e o diagnóstico é claro. A pipeline interrompe imediatamente se qualquer etapa retornar código de erro diferente de zero.

```bash
python3 src/runPipeline.py                  # pipeline completa
python3 src/runPipeline.py --skip-extract   # só velo + plot (dados já extraídos)
python3 src/runPipeline.py --skip-plot      # extrai sem plotar
python3 src/runPipeline.py --only-plot      # atalho para --skip-extract
```

---

## Formato CSV de Saída

Todos os módulos produzem CSVs no mesmo formato padronizado:

```csv
names,timestamp,id_can,prioridade,dado
ACT_TORQUE_A13,946688468.120000,0x18FF01F7,1,25.00 Nm
ACT_TORQUE_A13,946688468.220000,0x18FF01F7,1,26.50 Nm
```

| Coluna      | Tipo    | Descrição                                           |
|-------------|---------|-----------------------------------------------------|
| names       | string  | Nome do sinal (igual ao nome do arquivo CSV)        |
| timestamp   | float   | Unix epoch em segundos (com microssegundos)         |
| id_can      | string  | CAN ID no formato `0x` hexadecimal com 8 dígitos    |
| prioridade  | int     | Prioridade do sinal (atualmente sempre 1)           |
| dado        | string  | Valor físico seguido da unidade: `"25.00 Nm"`       |

O campo `dado` combina valor e unidade em uma string para facilitar leitura humana e extração automática de unidade no plotador (via regex).

---

## Validação Física

Cada sinal tem dois parâmetros de validação definidos no dicionário de sinais:

| Parâmetro         | Descrição                                                      |
|-------------------|----------------------------------------------------------------|
| `phys_min/max`    | Limites do range físico do sensor/sinal                       |
| `delta_max_frame` | Variação máxima aceitável entre dois frames consecutivos      |

**Critérios de rejeição (aplicados por amostra):**
1. Valor abaixo de `phys_min` ou acima de `phys_max` → rejeitado
2. `|val[i] - val[i-1]| > delta_max` → amostra `i` rejeitada (salto brusco)

**Limiar de suspeita:** se ≥ 20% das amostras de um sinal forem rejeitadas, o sinal inteiro é considerado suspeito. Em vez de um CSV parcialmente correto, o extrator gera um arquivo `<SINAL>.invalid` com:
- contagem de amostras totais e rejeitadas
- percentual de rejeição
- diagnóstico textual do possível problema

Isso impede que dados corrompidos entrem silenciosamente no pipeline de análise downstream.

---

## Adicionando Novos Sinais

Para adicionar um novo sinal, basta inserir uma entrada no dicionário `SINAIS_SESSAO` (session) ou `SINAIS_CANDUMP` (candump) seguindo o padrão:

```python
# extratorSessionFiles.py — fórmula: (raw / divisor) + offset
"NOME_DO_SINAL": (
    0x18FFXXXX,   # CAN ID como inteiro
    byte_inicio,  # offset inicial no payload
    byte_comp,    # comprimento em bytes (1 ou 2)
    com_sinal,    # True = signed, False = unsigned
    divisor,      # divide o valor bruto
    offset,       # soma após divisão
    "unidade",    # string da unidade física
    1,            # prioridade (manter 1)
    phys_min,     # limite mínimo físico
    phys_max,     # limite máximo físico
    delta_max,    # variação máxima por frame (None = sem filtro)
),
```

```python
# extratorCandumpFiles.py — fórmula: (raw * multiplicador) + offset
"NOME_DO_SINAL": (
    0x00000001,    # CAN ID
    byte_inicio,
    byte_comp,
    com_sinal,
    multiplicador, # multiplica o valor bruto (≠ divisor do session extrator)
    offset,
    "unidade",
    1,
    phys_min,
    phys_max,
    delta_max,
),
```

Para que o plotador exiba o novo sinal com cor e título corretos, adicione uma entrada em `METADADOS_SINAIS` no `plotador.py`:

```python
"NOME_DO_SINAL": ("#cor_hex", "Título legível do sinal", "unidade"),
```

---

## Diagnóstico de Erros

| Situação                               | O que acontece                              | Ação recomendada                                          |
|----------------------------------------|---------------------------------------------|-----------------------------------------------------------|
| Arquivo menor que o mínimo             | `[skip]` no log, arquivo ignorado           | Verificar se o arquivo foi gravado corretamente           |
| ≥ 20% de amostras rejeitadas           | Arquivo `.invalid` gerado, sem CSV          | Revisar especificação CAN ou firmware do dispositivo      |
| < 20% de amostras rejeitadas           | CSV gerado com `[FILTRADO]` no log          | Verificar se o range físico está correto na especificação |
| Sinal com 0 amostras                   | Nada é salvo (sem aviso)                    | Verificar se o CAN ID está correto e o barramento estava ativo |
| Pasta `data/raw/` não encontrada       | `[ERRO]` no log, script encerra            | Criar a pasta e adicionar os arquivos de dados            |