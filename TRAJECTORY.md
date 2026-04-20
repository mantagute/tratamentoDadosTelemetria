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

Combina a velocidade escalar longitudinal (já obtida pela integração de `ACC_Y`) com o heading acumulado a partir de `ANG_VEL_Z` para reconstruir a trajetória 2D.

### Obtenção da velocidade angular

`ANG_VEL_Z` pode ser obtido de duas formas:

**Opção 1 — Direto pelo sensor (giroscópio)**
Leitura direta do sinal `ANG_VEL_Z` já mapeado no barramento candump. O giroscópio mede a taxa de rotação em torno do eixo vertical diretamente, sem dependência de outros sinais.

**Opção 2 — Derivação a partir da aceleração lateral**
Usando a relação da aceleração centrípeta:

```
ω = ACC_X / VEL_Y
```

Quando o carro faz uma curva, a aceleração lateral sentida pela IMU é `ω × v`. Reorganizando, obtém-se o yaw rate a partir de sinais que já existem na pipeline.

| | Direto pelo sensor | Derivado de ACC_X / VEL_Y |
|---|---|---|
| Dependências | CAN ID e bytes de `ANG_VEL_Z` | `ACC_X` e `VEL_Y` já disponíveis |
| Qualidade do sinal | Alta — medição direta, sem propagação de erro | Menor — ruído de `ACC_X` e erro acumulado de `VEL_Y` se propagam |
| Divisão por zero | Não se aplica | Explode quando `VEL_Y ≈ 0` — requer limiar mínimo de velocidade |
| Viés lateral | Não captura | `ACC_X` inclui vibrações mecânicas e cambagem além da aceleração centrípeta |
| Recomendação | **Preferida** | Fallback ou validação cruzada |

A opção derivada pode ser usada enquanto o CAN ID de `ANG_VEL_Z` não estiver mapeado, com a ressalva de que a qualidade do mapa gerado será inferior, especialmente em velocidades baixas e em curvas lentas.

### Fluxo de processamento

```
ACC_Y  ──►  getVelocidade.py  ──►  VEL_Y (m/s)  ──────────────────────────────►  vx = vel · cos(θ)  ──►  x[i] = x[i-1] + vx · Δt
                                                                                                        ►  vy = vel · sin(θ)  ──►  y[i] = y[i-1] + vy · Δt
ANG_VEL_Z  ──►  getTrajetoria.py  ──►  θ[i] = θ[i-1] + 0.5 · (ω[i] + ω[i-1]) · Δt  ──────────────►
```

Etapas do módulo `getTrajetoria.py` (a implementar):

1. Corrigir bias de `ANG_VEL_Z` pelo mesmo método do `getVelocidade.py` (percentil 5%)
2. Aplicar filtro Butterworth passa-baixa para remover ruído antes de integrar
3. Integrar `ANG_VEL_Z` → heading `θ` por método trapezoidal com timestamps reais
4. Decompor `VEL_Y` em componentes world frame usando `θ`
5. Integrar `vx`, `vy` → posição `x`, `y`
6. Reportar erro de fechamento (distância entre ponto inicial e final) como métrica de qualidade da sessão

### Hipóteses assumidas

- `θ₀ = 0`: heading inicial arbitrário (norte local do sistema de coordenadas)
- `vel(t₀) = 0`: carro parado no início da janela de movimento
- Trajetória aproximadamente fechada para interpretação do drift — válido para pista de testes
- `ANG_VEL_Z` mede rotação em torno do eixo vertical (Z = vertical, confirmado)

### Riscos e mitigações

**Drift de posição acumulado** — inevitável em dead reckoning puro. O erro cresce com o tempo e fica visível quando o ponto final não coincide com o inicial no mapa. Mitigação imediata: reportar o erro de fechamento como métrica. Mitigação futura: fusão com GPS.

**Bias de `ANG_VEL_Z`** — offsets estáticos do giroscópio se acumulam no heading e distorcem toda a trajetória. Mitigação: aplicar a mesma correção de percentil 5% já usada em `getVelocidade.py` antes de integrar.

**Janela de movimento frágil** — um spike de vibração nos primeiros frames pode impedir o corte correto do repouso inicial, contaminando a integração com aceleração espúria. Mitigação: revisar o algoritmo de detecção para tolerar outliers isolados antes de confirmar repouso.

---

## Abordagem futura — Fusão IMU + RPM

O padrão adotado por equipes europeias de Formula SAE competitivas é a fusão de IMU com encoders de roda — no contexto deste projeto, os RPMs dos inversores. A razão é que as duas fontes se complementam nos seus pontos cegos:

- **IMU sozinha** deriva — erros de bias e ruído se acumulam nas integrações e o drift cresce com o tempo
- **RPM sozinho** mente em transientes — em aceleração forte, frenagem brusca e curvas com escorregamento o pneu patina e o encoder reporta uma velocidade que não corresponde ao deslocamento real

A fusão cobre ambos: o RPM ancora a velocidade nos momentos de rolamento limpo, a IMU cobre os transientes e detecta escorregamento quando a aceleração medida não bate com a variação esperada pelo RPM.

### Estimativas por odometria diferencial

Com RPM confiável, a diferença de velocidade entre `ACT_SPEED_A13` e `ACT_SPEED_B13` (motores em eixos opostos) fornece diretamente velocidade escalar e yaw rate por geometria diferencial — sem depender de integração de aceleração:

```
vel = (RPM_A + RPM_B) / 2 · fator_conversão
ω   = (RPM_A - RPM_B) / distância_entre_eixos
```

Vantagem principal: velocidade derivada de RPM é uma medição direta, não integrada — não acumula erro da mesma forma que `ACC_Y`.

### Filtro de fusão — EKF

A fusão em si é feita por um **Filtro de Kalman Estendido (EKF)**, que combina continuamente as duas fontes com pesos dinâmicos baseados na covariância estimada de cada sinal. Quando o RPM está confiável o filtro aumenta seu peso; quando detecta inconsistência (escorregamento), aumenta o peso da IMU. Muitos times adicionam ainda o **ângulo de esterço** como terceiro sinal — com ele e os RPMs tem-se um modelo cinemático completo do veículo que serve de preditor para o filtro, melhorando significativamente a estimativa em curvas.

### Progressão técnica

O dead reckoning implementado agora é a base necessária para chegar no EKF. Cada sinal precisa ser compreendido e validado individualmente antes de ser fundido. A progressão natural é:

```
[agora]   Dead reckoning  →  IMU only, valida pipeline e sinais individualmente
             ↓
[próximo] Correção firmware inversores  →  RPM confiável em operação
             ↓
[futuro]  EKF  →  fusão IMU + RPM (+ ângulo de esterço)  →  padrão competitivo SAE
```

A correção do firmware dos inversores não é apenas um fix de conveniência — é o que desbloqueia a abordagem competitiva.

### Por que está bloqueada hoje

Os bytes `b[1:3]` dos inversores apresentam saltos de ~16.000 rpm entre frames consecutivos durante operação sob carga — fisicamente impossível. O firmware grava dados de diagnóstico (posição de encoder ou contador de comutação) nesses bytes nessa condição. A taxa de rejeição supera 20%, gerando arquivos `.invalid`. Requer revisão com o fornecedor do inversor para separar os mapeamentos de bytes antes de poder ser utilizada.

---

## Pendências

| Pendência | Descrição | Bloqueia |
|---|---|---|
| CAN ID de `ANG_VEL_Z` | Identificar CAN ID, offset de bytes e fator de escala do giroscópio no barramento candump. Mapear em `SINAIS_CANDUMP`. | Abordagem adotada (opção sensor) |
| Convenção de sinal do yaw | Confirmar sentido positivo de `ANG_VEL_Z`: horário ou anti-horário visto de cima. Afeta orientação do mapa. | Abordagem adotada |
| Correção de firmware RPM | Separar mapeamento de bytes de RPM real e dados de diagnóstico nos inversores A13/B13. | Abordagem futura |
| Distância entre eixos | Medida física do veículo necessária para a fórmula de odometria diferencial. | Abordagem futura |
