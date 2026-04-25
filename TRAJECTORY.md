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
    - [Hipóteses assumidas](#hipóteses-assumidas)
    - [Riscos e mitigações](#riscos-e-mitigações)
  - [Resultados empíricos e diagnóstico de erros](#resultados-empíricos-e-diagnóstico-de-erros)
    - [Padrão observado](#padrão-observado)
    - [Causas identificadas](#causas-identificadas)
    - [Evidências visuais](#evidências-visuais)
  - [Abordagem futura — Fusão IMU + RPM](#abordagem-futura--fusão-imu--rpm)
    - [Estimativas por odometria diferencial](#estimativas-por-odometria-diferencial)
    - [Filtro de fusão — EKF](#filtro-de-fusão--ekf)
    - [Progressão técnica](#progressão-técnica)
    - [Por que está bloqueada hoje](#por-que-está-bloqueada-hoje)
  - [Pendências](#pendências)

---

## Contexto

A IMU embarcada fornece aceleração linear (`ACC_X`, `ACC_Y`) e velocidade angular (`ANG_VEL_Z`). Os inversores de tração fornecem RPM de cada motor (`ACT_SPEED_A13`, `ACT_SPEED_B13`). O objetivo é combinar esses sinais para estimar posição `x, y` ao longo do tempo.

O problema central é que a IMU vive no **body frame** do carro — seus eixos estão colados ao chassi e giram com ele. A trajetória precisa estar no **world frame** — coordenadas fixas no chão. A velocidade angular é a chave que converte um referencial no outro a cada instante: ela acumula o ângulo de rotação do veículo (heading), que permite projetar o movimento do body frame para o world frame.

---

## Abordagem adotada — Dead Reckoning

Combina a velocidade escalar longitudinal (já obtida pela integração de `ACC_Y`) com o heading acumulado a partir de `VENTOR_ANGULAR_SPEED_Z` para reconstruir a trajetória 2D.

### Obtenção da velocidade angular

`VENTOR_ANGULAR_SPEED_Z` é lido **diretamente do sensor** (giroscópio) via barramento candump — CAN ID `0x00000002`, bytes 2–3, int16, fator `× 0.01 rad/s`. O sinal já está mapeado em `SINAIS_CANDUMP` e é extraído por `extratorCandumpFiles.py`.

A opção de derivar `ω = ACC_X / VEL_Y` fica descartada como abordagem primária. Pode ser mantida como validação cruzada: se o yaw rate derivado diferir sistematicamente do sensor, pode indicar erro de calibração ou montagem da IMU.

> **Pendência de convenção de sinal:** o sentido positivo de `VENTOR_ANGULAR_SPEED_Z` (horário ou anti-horário visto de cima) ainda não foi confirmado empiricamente. Ver seção [Pendências](#pendências).

### Fluxo de processamento

```
ACC_Y  ──►  getVelocidade.py  ──►  VENTOR_LINEAR_VEL_Y (m/s)  ────────────────────────────────►  vx = vel · cos(θ)  ──►  x[i] = x[i-1] + vx · Δt
                                                                                                                      ►  vy = vel · sin(θ)  ──►  y[i] = y[i-1] + vy · Δt
VENTOR_ANGULAR_SPEED_Z (sensor)  ──►  getTrajetoria.py  ──►  θ[i] = θ[i-1] + 0.5·(ω[i]+ω[i-1])·Δt  ────────────►
```

Etapas do módulo `getTrajetoria.py`:

1. Carregar `VENTOR_LINEAR_VEL_Y.csv` e `VENTOR_ANGULAR_SPEED_Z.csv` do mesmo diretório de sessão.
2. Corrigir bias de `VENTOR_ANGULAR_SPEED_Z` pelo mesmo método do `getVelocidade.py` (percentil 5%).
3. Aplicar filtro Butterworth passa-baixa (4ª ordem, 2 Hz) para remover ruído antes de integrar.
4. Interpolar `VENTOR_ANGULAR_SPEED_Z` nos timestamps de `VENTOR_LINEAR_VEL_Y` (os dois sinais têm timestamps independentes).
5. Integrar `VENTOR_ANGULAR_SPEED_Z` → heading `θ` por método trapezoidal com timestamps reais.
6. Decompor `VENTOR_LINEAR_VEL_Y` em componentes world frame usando `θ`.
7. Integrar `vx`, `vy` → posição `x`, `y` por método trapezoidal.
8. Reportar erro de fechamento como métrica de qualidade da sessão.
9. Salvar `TRAJETORIA_X.csv` e `TRAJETORIA_Y.csv` no formato padrão da pipeline.

### Hipóteses assumidas

- `θ₀ = 0`: heading inicial arbitrário (norte local do sistema de coordenadas).
- `vel(t₀) = 0`: carro parado no início da janela de movimento (herdado do `getVelocidade.py`).
- Trajetória aproximadamente fechada para interpretação do drift — válido para pista de testes.
- `VENTOR_ANGULAR_SPEED_Z` mede rotação em torno do eixo Z (vertical), confirmado pela especificação da IMU.

### Riscos e mitigações

**Drift de posição acumulado** — inevitável em dead reckoning puro. O erro cresce com o tempo e fica visível quando o ponto final não coincide com o inicial no mapa. Mitigação imediata: reportar o erro de fechamento como métrica. Mitigação futura: fusão com GPS.

**Bias de `VENTOR_ANGULAR_SPEED_Z`** — offsets estáticos do giroscópio se acumulam no heading e distorcem toda a trajetória. Mitigação: correção de percentil 5% antes de integrar. **Limitação conhecida:** se a sessão começa já em movimento, o percentil 5% captura amostras de curva suave em vez de repouso real e subestima o bias. Ver causas identificadas abaixo.

**Convenção de sinal do yaw** — se o sentido positivo estiver invertido, a trajetória será espelhada. Mitigação: flag `--negar-yaw` no `getTrajetoria.py`.

**Dessincronização de timestamps** — `VENTOR_ANGULAR_SPEED_Z` e `VENTOR_LINEAR_VEL_Y` têm timestamps independentes. Mitigação: interpolação linear antes da integração conjunta.

**Remoção de drift linear da velocidade** — `getVelocidade.py` subtrai uma rampa linear assumindo velocidade final ~0. Se a sessão não termina com o carro parado, essa correção introduz uma velocidade residual artificial que se integra em erro de posição.

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

## Abordagem futura — Fusão IMU + RPM

O padrão adotado por equipes europeias de Formula SAE competitivas é a fusão de IMU com encoders de roda — no contexto deste projeto, os RPMs dos inversores. A razão é que as duas fontes se complementam nos seus pontos cegos:

- **IMU sozinha** deriva — erros de bias e ruído se acumulam nas integrações e o drift cresce com o tempo.
- **RPM sozinho** mente em transientes — em aceleração forte, frenagem brusca e curvas com escorregamento o pneu patina e o encoder reporta uma velocidade que não corresponde ao deslocamento real.

### Estimativas por odometria diferencial

Com RPM confiável, a diferença de velocidade entre `ACT_SPEED_A13` e `ACT_SPEED_B13` fornece diretamente velocidade escalar e yaw rate por geometria diferencial:

```
vel = (RPM_A + RPM_B) / 2 · fator_conversão
ω   = (RPM_A - RPM_B) / distância_entre_eixos
```

### Filtro de fusão — EKF

A fusão é feita por um **Filtro de Kalman Estendido (EKF)**, que combina as duas fontes com pesos dinâmicos baseados na covariância estimada de cada sinal. Quando o RPM está confiável o filtro aumenta seu peso; quando detecta inconsistência (escorregamento), aumenta o peso da IMU. O ângulo de esterço como terceiro sinal permite um modelo cinemático completo.

### Progressão técnica

```
[agora]   Dead reckoning  →  IMU only, valida pipeline e sinais individualmente
             ↓
[próximo] Correção bias giroscópio  →  estimar bias só nos trechos de repouso real
             ↓
[próximo] Correção drift velocidade →  não aplicar rampa quando sessão não termina em repouso
             ↓
[futuro]  Correção firmware inversores  →  RPM confiável em operação
             ↓
[futuro]  EKF  →  fusão IMU + RPM (+ ângulo de esterço)  →  padrão competitivo SAE
```

### Por que está bloqueada hoje

Os bytes `b[1:3]` dos inversores apresentam saltos de ~16.000 rpm entre frames consecutivos durante operação sob carga. O firmware grava dados de diagnóstico nesses bytes nessa condição. A taxa de rejeição supera 20%, gerando arquivos `.invalid`. Requer revisão com o fornecedor do inversor.

---

## Pendências

| Pendência | Descrição | Bloqueia |
|---|---|---|
| Convenção de sinal do yaw | Confirmar sentido positivo de `VENTOR_ANGULAR_SPEED_Z`. Fazer uma curva à direita em baixa velocidade e verificar se o sinal resultante é positivo ou negativo. Registrar aqui e corrigir em `getTrajetoria.py` se necessário. | Qualidade do mapa — trajetória pode estar espelhada |
| Estimativa de bias com repouso real | Modificar `getTrajetoria.py` para estimar o bias do giroscópio apenas nos primeiros e últimos N segundos, em vez do percentil global que falha quando a sessão começa em movimento. | Erro sistemático de heading — causa principal do drift em sessões longas |
| Validação da remoção de drift de velocidade | Modificar `getVelocidade.py` para verificar se a velocidade está próxima de zero no início e fim da janela antes de aplicar a rampa de drift. Emitir aviso quando a hipótese não for satisfeita. | Erro sistemático de posição em sessões cortadas no meio do movimento |
| Visualização de trajetória reta | Melhorar o plotador para detectar trajetórias com proporção de aspecto extrema e aplicar padding mínimo nos eixos. | Legibilidade de sessões curtas em linha reta |
| Correção de firmware RPM | Separar mapeamento de bytes de RPM real e dados de diagnóstico nos inversores A13/B13. | Abordagem futura (EKF) |
| Distância entre eixos | Medida física necessária para a fórmula de odometria diferencial. | Abordagem futura (EKF) |