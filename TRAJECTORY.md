# Reconstrução de Trajetória — Decisões Técnicas

Pipeline para reconstruir a trajetória 2D do veículo no plano da pista a partir dos sinais CAN já capturados.

---

## Índice

- [Reconstrução de Trajetória — Decisões Técnicas](#reconstrução-de-trajetória--decisões-técnicas)
  - [Índice](#índice)
  - [Contexto](#contexto)
  - [Abordagem adotada — Dead Reckoning](#abordagem-adotada--dead-reckoning)
    - [Obtenção da velocidade angular](#obtenção-da-velocidade-angular)
    - [Fluxo de processamento](#fluxo-de-processamento)
    - [Regra de negócio — mapa base e tracking](#regra-de-negócio--mapa-base-e-tracking)
    - [Hipóteses assumidas](#hipóteses-assumidas)
    - [Riscos e mitigações](#riscos-e-mitigações)
  - [Fusão IMU + RPM e parâmetros de execução](#fusão-imu--rpm-e-parâmetros-de-execução)
    - [Estimativas por odometria diferencial](#estimativas-por-odometria-diferencial)
    - [Filtro de fusão atual — complementar](#filtro-de-fusão-atual--complementar)
    - [Filtro de fusão futuro — EKF](#filtro-de-fusão-futuro--ekf)
    - [Progressão técnica](#progressão-técnica)
  - [Resultados empíricos e diagnóstico de erros](#resultados-empíricos-e-diagnóstico-de-erros)
    - [Padrão observado](#padrão-observado)
    - [Causas identificadas](#causas-identificadas)
    - [Evidências visuais](#evidências-visuais)
  - [Inspeção geral (dados atuais) e melhorias recentes](#inspeção-geral-dados-atuais-e-melhorias-recentes)
  - [Pendências](#pendências)

## Contexto

A IMU embarcada fornece aceleração linear (`ACC_X`, `ACC_Y`, `ACC_Z`) e velocidade angular (`ANG_VEL_Z`). No padrão FSAE adotado aqui, `ACC_X` é a aceleração longitudinal do carro e `ACC_Z` é positivo para baixo. Os inversores de tração fornecem RPM de cada motor (`ACT_SPEED_A13`, `ACT_SPEED_B13`). O objetivo é combinar esses sinais para estimar posição `x, y` ao longo do tempo.

O problema central é que a IMU vive no **body frame** do carro — seus eixos estão colados ao chassi e giram com ele. A trajetória precisa estar no **world frame** — coordenadas fixas no chão. A velocidade angular é a chave que converte um referencial no outro a cada instante: ela acumula o ângulo de rotação do veículo (heading), que permite projetar o movimento do body frame para o world frame.

---

## Abordagem adotada — Dead Reckoning

Combina a velocidade escalar longitudinal estimada por `RPM + ACC_X` com o heading acumulado a partir de `VENTOR_ANGULAR_SPEED_Z` para reconstruir a trajetória 2D.

### Obtenção da velocidade angular

`VENTOR_ANGULAR_SPEED_Z` é lido **diretamente do sensor** (giroscópio) via barramento candump — CAN ID `0x00000002`, bytes 2–3, int16, fator `× 0.01 rad/s`. O sinal já está mapeado em `SINAIS_CANDUMP` e é extraído por `extratorCandumpFiles.py`.

A opção de derivar `ω` pela relação entre aceleração lateral e velocidade longitudinal fica descartada como abordagem primária. Pode ser mantida como validação cruzada: se o yaw rate derivado diferir sistematicamente do sensor, pode indicar erro de calibração ou montagem da IMU.

> **Pendência de convenção de sinal:** o sentido positivo de `VENTOR_ANGULAR_SPEED_Z` (horário ou anti-horário visto de cima) ainda não foi confirmado empiricamente. Ver seção [Pendências](#pendências).

### Fluxo de processamento

```
ACT_SPEED_A13/B13 ──► rpm / 11,72 ──► rps_roda × circunferência ──┐
                                                                  ├──► velocidade longitudinal ──► vx = vel · cos(θ) ──► x,y (bruto)
ACC_X ───────────────────────────────► predição de transientes ───┘                              └──► vy = vel · sin(θ) ──►
VENTOR_ANGULAR_SPEED_Z (sensor)  ──►  getTrajetoria.py  ──►  θ[i] = θ[i-1] + 0.5·(ω[i]+ω[i-1])·Δt  ────────────►
                                                                                         x,y (bruto) ──► mapa base + tracking ──► x,y (CSV)
```

Etapas do módulo `getTrajetoria.py`:

1. Carregar `ACT_SPEED_A13.csv`/`ACT_SPEED_B13.csv`, `VENTOR_LINEAR_ACC_X.csv` e `VENTOR_ANGULAR_SPEED_Z.csv` do mesmo diretório de sessão.
2. Corrigir bias de `VENTOR_ANGULAR_SPEED_Z` pelo mesmo método do `getVelocidade.py` (percentil 5%).
3. Aplicar filtro Butterworth passa-baixa (4ª ordem, 2 Hz) para remover ruído antes de integrar.
4. Converter RPM de motor em velocidade linear de roda e fundir com `ACC_X` por filtro complementar.
5. Integrar `VENTOR_ANGULAR_SPEED_Z` → heading `θ` por método trapezoidal com timestamps reais.
6. Decompor a velocidade longitudinal fundida em componentes world frame usando `θ`.
7. Integrar `vx`, `vy` → posição `x`, `y` por método trapezoidal (trajetória bruta contínua).
8. **Mapa base + tracking (padrão):** estimar duração típica de volta pela autocorrelação da velocidade fundida (ou usar `--lap-period-sec`); tratar a primeira volta como mapa fixo (`MAPA_BASE_X/Y`); depois disso, não gerar novas geometrias de pista, apenas projetar a posição do carro nesse mapa por progresso de distância. Desligar com `--no-track-map` (`--no-mini-slam` segue como alias legado).
9. Reportar erro de fechamento como métrica de qualidade da sessão.
10. Salvar `TRAJETORIA_X.csv` e `TRAJETORIA_Y.csv` no formato padrão da pipeline.

### Regra de negócio — mapa base e tracking

A regra correta do produto é: **mapa não é uma estimativa iterativa infinita da pista**. O primeiro objetivo é gerar um mapa base plausível na primeira volta completa. A partir daí, a tarefa muda: o sistema deve **trackear a posição do veículo dentro desse mapa**, não redesenhar a pista volta após volta.

Na implementação, a primeira volta é congelada como `MAPA_BASE_X.csv` e `MAPA_BASE_Y.csv`. O restante da sessão usa a distância acumulada pela velocidade fundida (`RPM + ACC_X`) para calcular o progresso do carro na volta e projetar o ponto correspondente sobre a polilinha fixa do mapa. Assim as distorções de uma volta posterior não viram uma pista nova e não contaminam o mapa. O CSV `TRAJETORIA_X/Y` passa a representar a posição rastreada no mapa base quando o tracking está ativo; o `plotador.py` sobrepõe o mapa base em amarelo no gráfico `TRAJETORIA_2D.png`.

**Estimativa de período:** autocorrelação da série de velocidade (centrada na média); primeiro máximo relevante acima de ~12 s e abaixo de ~360 s; exige duração da sessão ≥ ~1,28× o período detectado para tentar duas voltas. Se a detecção falhar (pista irregular, SC, poucas voltas), informe o período manualmente: `--lap-period-sec 52.5` ou `--lap-period-sec=52.5`. A `runPipeline.py` repassa essas flags ao `getTrajetoria.py`.

**Limitações:** se a primeira volta estiver ruim, o mapa base ficará ruim; o tracking só impede que as voltas seguintes deformem ainda mais a pista. Voltas com tempos muito diferentes, bandeiras, pit ou recorte que corta no meio da volta degradam a estimativa de progresso. O fechamento da primeira volta continua sendo uma métrica importante de qualidade do mapa base.

### Hipóteses assumidas

- `θ₀ = 0`: heading inicial arbitrário (norte local do sistema de coordenadas).
- A velocidade longitudinal vem do RPM como grandeza escalar; o sinal dos motores é tratado em módulo para evitar cancelamento quando a montagem elétrica usa sinais opostos entre lados.
- Trajetória aproximadamente fechada para interpretação do drift — válido para pista de testes.
- `VENTOR_ANGULAR_SPEED_Z` mede rotação em torno do eixo Z (vertical), confirmado pela especificação da IMU.

### Riscos e mitigações

**Drift de posição acumulado** — inevitável em dead reckoning puro. O erro cresce com o tempo e fica visível quando o ponto final não coincide com o inicial no mapa. Mitigações: reportar o erro de fechamento; fusão RPM + `ACC_X`; mapa base da primeira volta + tracking de posição; futuro: fusão com GPS.

**Bias de `VENTOR_ANGULAR_SPEED_Z`** — offsets estáticos do giroscópio se acumulam no heading e distorcem toda a trajetória. Mitigação: correção de percentil 5% antes de integrar. **Limitação conhecida:** se a sessão começa já em movimento, o percentil 5% captura amostras de curva suave em vez de repouso real e subestima o bias. Ver causas identificadas abaixo.

**Convenção de sinal do yaw** — se o sentido positivo estiver invertido, a trajetória será espelhada. Mitigação: flag `--negar-yaw` no `getTrajetoria.py`.

**Dessincronização de timestamps** — RPM, `VENTOR_LINEAR_ACC_X` e `VENTOR_ANGULAR_SPEED_Z` têm timestamps independentes. Mitigação: interpolação linear antes da integração conjunta.

**Remoção de drift linear da velocidade** — `getVelocidade.py` subtrai uma rampa linear assumindo velocidade final ~0. Se a sessão não termina com o carro parado, essa correção introduz uma velocidade residual artificial que se integra em erro de posição.

---

## Fusão IMU + RPM e parâmetros de execução

**Convenção FSAE (telemetria deste repositório):** `VENTOR_LINEAR_ACC_X` é aceleração longitudinal; `ACC_Z` é positivo para baixo. **RPM dos inversores** está considerado válido e ancora a escala de velocidade.

**Conversão motor → velocidade linear da roda** (constantes em `getTrajetoria.py`):

```
rpm_roda   = rpm_motor / 11,72        # redução planetária
rps_roda   = rpm_roda / 60
circunf    = 2π × raio_roda
raio_roda  = 10 pol × 0,0254 m/pol = 0,254 m
vel_m/s    = rps_roda × circunf      # equivalente a |rpm_motor| × (circunf / (60 × 11,72))
```

Implementação NumPy: `RPM_MOTOR_PARA_MPS = CIRCUNFERENCIA_RODA_M / (60.0 * FATOR_REDUCAO_PLANETARIA)`. Com dois motores, interpola-se cada lado na grade temporal mais densa e usa-se a média das velocidades derivadas antes da fusão com `ACC_X`.

**Fusão velocidade (filtro complementar):** a cada passo, a velocidade predita pela integração de `ACC_X` é misturada com a velocidade a partir do RPM (`PESO_CORRECAO_RPM`, padrão 0,35) para manter transientes da IMU sem deriva livre de escala entre trechos longos.

O padrão adotado por equipes europeias de Formula SAE competitivas é a fusão de IMU com encoders de roda — no contexto deste projeto, os RPMs dos inversores. As duas fontes se complementam nos seus pontos cegos:

- **IMU sozinha** deriva — erros de bias e ruído se acumulam nas integrações e o drift cresce com o tempo.
- **RPM sozinho** mente em transientes — em aceleração forte, frenagem brusca e curvas com escorregamento o pneu patina e o encoder reporta uma velocidade que não corresponde ao deslocamento real.

### Estimativas por odometria diferencial

Com RPM confiável, a diferença de velocidade entre `ACT_SPEED_A13` e `ACT_SPEED_B13` fornece diretamente velocidade escalar e yaw rate por geometria diferencial:

```
vel = (RPM_A + RPM_B) / 2 · fator_conversão
ω   = (RPM_A - RPM_B) / distância_entre_eixos
```

### Filtro de fusão atual — complementar

A implementação atual usa um filtro complementar simples em `getTrajetoria.py`: `ACC_X` prevê transientes entre frames e o RPM corrige continuamente a escala de velocidade. A camada de **mapa base + tracking** trata a regra de negócio no plano `x,y`: primeira volta define o mapa; voltas seguintes só atualizam a posição sobre ele.

### Filtro de fusão futuro — EKF

A fusão é feita por um **Filtro de Kalman Estendido (EKF)**, que combina as duas fontes com pesos dinâmicos baseados na covariância estimada de cada sinal. Quando o RPM está confiável o filtro aumenta seu peso; quando detecta inconsistência (escorregamento), aumenta o peso da IMU. O ângulo de esterço como terceiro sinal permite um modelo cinemático completo.

### Progressão técnica

```
[agora]   Dead reckoning  →  IMU only, valida pipeline e sinais individualmente
             ↓
[próximo] Correção bias giroscópio  →  estimar bias só nos trechos de repouso real
             ↓
[próximo] Correção drift velocidade →  não aplicar rampa quando sessão não termina em repouso
             ↓
[agora]   Fusão complementar  →  RPM ancora velocidade, ACC_X modela transientes
             ↓
[agora]   Mapa base + tracking  →  primeira volta vira pista; depois só rastreia posição nela
             ↓
[futuro]  EKF  →  fusão IMU + RPM (+ ângulo de esterço)  →  padrão competitivo SAE
```

---

## Resultados empíricos e diagnóstico de erros

### Padrão observado

Nas primeiras sessões testadas (abril 2026), o erro de fechamento ficou entre **47% e 79% da extensão máxima da trajetória**, independente da duração da sessão. Esse padrão de erro **proporcional e consistente entre sessões diferentes** indica que a fonte principal não é ruído aleatório — é um erro sistemático de processamento.

| Sessão  | Duração | Extensão aprox. | Erro fechamento | Ratio |
|---------|---------|-----------------|-----------------|-------|
| 230150  | 33.9s   | ~88m            | 41.14m          | ~47%  |
| 230123  | 61.6s   | ~78m            | 61.71m          | ~79%  |
| 230135  | 172.4s  | ~1500m          | 916.11m         | ~61%  |
| 230112  | 9.0s    | ~0.5m           | 0.55m           | ~100% |

### Causas identificadas

**1. Bias do giroscópio mal estimado quando não há repouso inicial**

O percentil 5% da magnitude assume que os menores valores correspondem ao carro parado. Se a sessão começa já em movimento, as amostras de menor magnitude são de curvas suaves — o bias estimado fica incorreto. Um erro de bias de apenas 0.01 rad/s acumula ~1.7°/s de erro de heading — em 172s isso é ~290° de desvio, suficiente para girar a trajetória inteira e criar centenas de metros de erro lateral.

**Mitigação a implementar:** estimar o bias **apenas nos primeiros e últimos N segundos da sessão** (onde o carro provavelmente está parado), em vez do percentil global. Isso exige que as sessões sejam recortadas incluindo um pequeno trecho de repouso no início e no fim.

**2. Remoção de drift linear da velocidade distorcendo a posição**

O `getVelocidade.py` remove drift subtraindo `linspace(0, vel_final, n)`. Isso assume que a velocidade ao final da janela deve ser 0. Quando a sessão é cortada no meio do movimento — o que ocorre com o recorte manual atual — a rampa introduz uma velocidade artificial negativa que se integra diretamente em erro de posição.

**Mitigação a implementar:** verificar se a velocidade está próxima de zero no início e fim da janela antes de aplicar a remoção de drift. Se não estiver, emitir aviso e não aplicar a correção.

**3. Sessões muito curtas sem curvatura real**

Sessões curtas em linha reta produzem trajetórias com proporção de aspecto extrema (ex: 0.5m × 0.025m). O `set_aspect("equal")` do plotador transforma isso em um gráfico achatado ilegível. O erro de fechamento nesse caso não tem significado geométrico útil.

**Mitigação implementada:** o plotador detecta trajetórias degeneradas e aplica padding mínimo nos eixos para garantir legibilidade.

### Evidências visuais

- **Shape plausível em sessões medianas (30–65s):** as sessões 230150 e 230123 mostram formas de teardrop/retorno geometricamente consistentes com manobras reais em pista. O drift existe mas a topologia geral está correta, indicando que o pipeline está funcionando — o erro é de magnitude, não de estrutura.
- **Drift de heading dominante em sessões longas (172s):** a sessão 230135 começa com uma linha diagonal de ~400m antes de entrar nas curvas — sinal claro de bias residual no giroscópio acumulando erro de heading desde o primeiro frame.
- **Trajetória reta degenerada (9s):** sessão 230112 gerou gráfico achatado ilegível por ausência de curvatura real no trecho capturado.

---

## Inspeção geral (dados atuais) e melhorias recentes

**Constatação:** em `data/processed/` **não há** `ACT_SPEED_A13/B13.csv` — toda a trajetória cai no **fallback** `VENTOR_LINEAR_VEL_*` (integração do ACC). Sem RPM, não há fusão que ancore a escala longitudinal; qualquer erro do integrador de velocidade ou do gyro pesa direto no `x,y`.

**Conflito com corte manual do ACC:** o `getVelocidade.py` ainda aplicava (1) **segundo corte** por “janela de movimento” e (2) **rampa** que força `v_final ≈ 0`. Se já recortaste o ACC à mão, isso pode **encolher de mais** o trecho ou **distorcer** a escala de velocidade.

**Flags adicionadas (sem precisar de extrator):**

| Flag | Script | Efeito |
|------|--------|--------|
| `--sem-janela-movimento` | `getVelocidade.py` | Integra **todo** o CSV de ACC (não recorta repouso auto). |
| `--sem-rampa-drift` | `getVelocidade.py` | **Não** aplica rampa linear até zero no fim; só ancora `v(0)=0`. |
| `--bias-yaw-bordas-sec S` | `getTrajetoria.py` | Bias do gyro = média nos **primeiros e últimos S s** (útil com parado nas pontas). |
| `--no-track-map` | `getTrajetoria.py` | Desativa o tracking no mapa da primeira volta e salva a trajetória bruta contínua. |

A `runPipeline.py` **repassa** essas flags quando presentes em `sys.argv`.

**Fluxo recomendado** (preserva o teu corte de ACC; não roda extratores):

```bash
./.venv/bin/python src/getVelocidade.py data/processed/**/VENTOR_LINEAR_ACC_*.csv \
  --sem-janela-movimento --sem-rampa-drift
./.venv/bin/python src/getTrajetoria.py data/processed/candump-*/ \
  --bias-yaw-bordas-sec 3
./.venv/bin/python src/plotador.py
```

(Ajusta `S` em função de quantos segundos de parado tens no início/fim; se não houver repouso real, mantém o bias por percentil omitindo esta flag.)

**Outras alternativas de médio prazo:** gerar `ACT_SPEED_*.csv` a partir dos logs de sessão (extrator só de inversores, se existir fonte separada); EKF; GPS ou marcação explícita de voltas; validar escala do CAN do gyro em sessões com `yaw_max` suspeitamente baixo (~0,01 rad/s).

---

## Validação com dados sintéticos perfeitos

Para separar erro de algoritmo de ruído real, foi adicionado o gerador:

`src/gerarDadosPerfeitos.py`

Ele cria uma sessão sintética com sinais fisicamente coerentes entre si:
- `ACT_SPEED_A13/B13` (RPM ideal)
- `VENTOR_LINEAR_ACC_X` (aceleração longitudinal ideal)
- `VENTOR_ANGULAR_SPEED_Z` (yaw rate ideal)
- `REF_TRAJETORIA_X/Y` (trajetória de referência “ground truth”)

### Objetivo do teste

Validar em ambiente controlado:
- se o mapa 2D é plotado com geometria correta;
- se o tracking no mapa base da primeira volta acompanha a posição corretamente;
- se o erro de fechamento fica baixo quando não há ruído.

### Protocolo de execução

```bash
python3 src/gerarDadosPerfeitos.py --saida sintetico-perfeito
python3 src/getTrajetoria.py data/processed/sintetico-perfeito --lap-period-sec 40 --bias-yaw-bordas-sec 1
python3 src/plotador.py sintetico-perfeito
```

### Critérios de aceite (regra de negócio)

- O plot `TRAJETORIA_2D.png` deve mostrar trajetória fechada e estável (sem deformação progressiva entre voltas).
- O arquivo `MAPA_BASE_X.csv/Y.csv` deve existir quando o tracking estiver ativo.
- O erro de fechamento reportado no `getTrajetoria.py` deve ser baixo (ordem de poucos metros ou menos; idealmente próximo de zero nesse cenário).
- Referência atual do teste sintético (21/05/2026): erro de fechamento `~0.14 m` com `--bias-yaw-bordas-sec 1`.
- O traçado reconstruído (`TRAJETORIA_X/Y`) deve permanecer aderente ao mapa base ao longo das voltas.

Se os critérios acima forem atendidos no sintético e falharem no real, a próxima etapa deve focar em:
- redução de ruído (filtro/parametrização),
- bias de yaw em repouso real,
- sincronização temporal,
- mitigação da propagação de erro por integração.

---

## Execução real — diagnóstico de MAPA_BASE (21/05/2026)

### Comandos executados

Detecção automática de período:

```bash
python3 src/getTrajetoria.py data/processed/candump-* --bias-yaw-bordas-sec 3
```

Tentativa com período fixo:

```bash
python3 src/getTrajetoria.py data/processed/candump-* --lap-period-sec 45 --bias-yaw-bordas-sec 3
```

### Resultado observado

Sessões sem `MAPA_BASE` na detecção automática:
- `candump-1999-12-31_230112`
- `candump-1999-12-31_230113`
- `candump-1999-12-31_230123`
- `candump-1999-12-31_230132`
- `candump-1999-12-31_230140`
- `candump-1999-12-31_230143`
- `candump-1999-12-31_230150`

Motivos reportados pelo próprio `getTrajetoria.py`:
- `pico_autocorr_fraco`: série de velocidade sem periodicidade forte o bastante para inferir volta.
- `duracao_ou_fs_insuficiente`: janela útil curta para estimar período.

Após fixar `--lap-period-sec 45`, `MAPA_BASE` passou a existir em parte dos casos (ex.: `230113`, `230123`, `230140`), mas não em todos.

Motivo dos que ainda falharam com período fixo:
- `duracao_curta_para_multiplas_voltas`: após sobreposição entre sinais, a janela efetiva ficou curta para a regra atual de map/tracking.

### Por que sessões “grandes” podem não gerar mapa

A decisão não usa a duração bruta do arquivo. Usa a duração da **grade comum** (interseção temporal entre velocidade e yaw). Em vários casos, apesar de log longo, a sobreposição útil ficou curta (ex.: 10 s, 29 s, 41 s), impedindo mapa/tracking.

### Como arrumar na prática

1. Sempre verificar a linha `Grade comum` no log do `getTrajetoria.py` (não apenas a duração total do arquivo).
2. Rodar com `--lap-period-sec` quando o auto falhar por `pico_autocorr_fraco`.
3. Ajustar o período por sessão (ex.: 45/50/55) e escolher o que reduz fechamento da 1ª volta.
4. Revisar o recorte da sessão para aumentar a sobreposição `VEL × YAW`.
5. Priorizar entrada com `RPM + ACC_X` (modo fusão) em vez de fallback `VEL_X + YAW`, pois a periodicidade de velocidade fica mais robusta.

### Validação do teste sintético

Teste sintético confirmado funcional para validar algoritmo e plotagem:

```bash
python3 src/gerarDadosPerfeitos.py --saida sintetico-perfeito
python3 src/getTrajetoria.py data/processed/sintetico-perfeito --lap-period-sec 40 --bias-yaw-bordas-sec 1
python3 src/plotador.py sintetico-perfeito
```

Referência observada na execução: erro de fechamento ~`0.14 m`, com geração de `MAPA_BASE_X/Y`.
Isso confirma que, em dado coerente e sem ruído, mapa e tracking funcionam; problemas remanescentes no real são majoritariamente de qualidade/compatibilidade de sinal e janela útil.

---

## Integração com frontend/backend (status atual)

### Acesso ao Telemetria

No ambiente local atual foi encontrado `TelemetriaV2.0` (não foi localizado `TelemetriaV2.1`).
No `V2.0`, o painel `TrackMapPanel` ainda consome uma `source` de imagem (`<img src=...>`), então o caminho mais rápido é servir um PNG de mapa.

### Nova etapa de exportação para frontend

Foi adicionado o script:

`src/exportMapaFrontend.py`

Ele gera, por sessão:
- `frontend/track_map.png` (mapa para o painel atual)
- `frontend/track_timeline.json` (timeline de posição para evolução futura do cockpit)

Estrutura de saída:

`data/processed/<sessao>/frontend/`

### Pipeline recomendada para produção de mapa no frontend

1. Rodar reconstrução de trajetória (com `MAPA_BASE` quando possível).
2. Exportar artefatos frontend.
3. Servir `track_map.png` no backend e apontar `trackMapSource` para essa URL.

Comandos:

```bash
python3 src/getTrajetoria.py data/processed/candump-* --lap-period-sec 45 --bias-yaw-bordas-sec 3
python3 src/exportMapaFrontend.py
python3 src/plotador.py
```

Ou pelo orquestrador:

```bash
python3 src/runPipeline.py --skip-extract --export-track-frontend --lap-period-sec 45 --bias-yaw-bordas-sec 3
```

### Motivos de falha para “sem mapa base” (resumo operacional)

- `pico_autocorr_fraco`: periodicidade de velocidade fraca para inferir volta automaticamente.
- `duracao_ou_fs_insuficiente`: grade comum curta para detectar período.
- `duracao_curta_para_multiplas_voltas`: mesmo com período fixo, janela útil não atende regra de tracking.

### Critério de validação para integração frontend

- Sessão deve conter `MAPA_BASE_X/Y` (ou fallback explícito aceito em operação).
- `frontend/track_map.png` deve existir e abrir corretamente no cockpit.
- `track_timeline.json` deve conter `track.points` e `timeline.vehicle.{x,y}` para futura animação de posição sobre o mapa.

---

## Modularização da regra de negócio do mapa

Para preparar integração em tempo real, a regra foi separada em módulos:

- `src/getVelocidade.py`: velocidade por integração da aceleração, com corte automático de parado (janela de movimento) por padrão.
- `src/mapaRegraNegocio.py`: regra central do mapa:
  1. estimar período de volta (`estimar_periodo_volta_segundos`);
  2. congelar 1ª volta em `MAPA_BASE`;
  3. rastrear posição no mapa por progresso escalar.
- `src/getTrajetoria.py`: integração IMU/RPM + uso explícito do `mapaRegraNegocio`.
- `src/exportMapaFrontend.py`: exporta `frontend/track_map.png` e `frontend/track_timeline.json`.
- `src/runPipelineMapa.py`: pipeline dedicada a essa regra (velocidade → trajetória/mapa → export frontend).

### Processo validado (pré-integração realtime)

1. Extrair/corrigir velocidade de `ACC` com corte de parado:
   - padrão do `getVelocidade.py` (sem `--sem-janela-movimento`).
2. Gerar trajetória e mapa base:
   - `getTrajetoria.py` com `--lap-period-sec` quando auto-detecção falhar.
3. Confirmar artefatos:
   - `TRAJETORIA_X/Y.csv`
   - `MAPA_BASE_X/Y.csv` (quando sessão válida para mapa)
   - `frontend/track_map.png`
   - `frontend/track_timeline.json`

### Comando único da pipeline de negócio do mapa

```bash
python3 src/runPipelineMapa.py --lap-period-sec 45 --bias-yaw-bordas-sec 3
```

### Critério de pronto para integração

- Velocidade integrada sem trecho de repouso contaminando início/fim (janela de movimento coerente no log).
- Mapa base gerado a partir da 1ª volta para sessões com duração/sobreposição adequadas.
- Tracking projetando posição no mapa fixo (sem redesenhar pista a cada volta).
- Export frontend disponível por sessão para acoplar no backend em tempo real.

---

## Pendências

| Pendência | Descrição | Bloqueia |
|---|---|---|
| Convenção de sinal do yaw | Confirmar sentido positivo de `VENTOR_ANGULAR_SPEED_Z`. Fazer uma curva à direita em baixa velocidade e verificar se o sinal resultante é positivo ou negativo. Registrar aqui e corrigir em `getTrajetoria.py` se necessário. | Qualidade do mapa — trajetória pode estar espelhada |
| Estimativa de bias com repouso real | Opcional: `--bias-yaw-bordas-sec S` no `getTrajetoria.py`. Pendente automatizar escolha de S ou detetar repouso. | Erro sistemático de heading em sessões longas |
| Validação da remoção de drift de velocidade | Opcional: `--sem-rampa-drift` quando a rampa for incorrecta; pendente detetar automaticamente `|v_início|,|v_fim|` pequenos antes de aplicar rampa. | Erro de posição em cortes manuais / trecho aberto |
| Visualização de trajetória reta | Melhorar o plotador para detectar trajetórias com proporção de aspecto extrema e aplicar padding mínimo nos eixos. | Legibilidade de sessões curtas em linha reta |
| Distância entre eixos | Medida física necessária para a fórmula de odometria diferencial. | Abordagem futura (EKF) |
