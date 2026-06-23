# [202525344-정지우] ICC 제어기 설계 보고서

**과목**: 자동제어특론 - 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요

본 과제는 14DOF 차량 plant 위에서 횡·종·수직 통합 샤시 제어기(ICC)를 설계하여 핸들링 안정성, 제동 거리, 승차감을 baseline 대비 정량적으로 개선하는 것을 목표로 한다. 제어기법 선택의 핵심 기준은 두 가지였다: (1) 강의에서 다룬 이론을 직접 적용할 수 있는가, (2) MIMO 시스템의 물리적 coupling을 명시적으로 다루는가.

횡방향 제어에는 **LQR with integral action**을 선택하였다. 적분 상태를 포함한 state feedback 구조는 강의자료 6_1의 "State Variable Feedback with I-action" 절(P.36-49)에 근거한다. PID와 달리 Q, R 행렬로 게인을 산출할 수 있고, 적분기 추가로 정상상태 yaw rate 오차를 줄일 수 있다. ARE(Algebraic Riccati Equation) 풀이에는 Hamiltonian eigenvalue 분해를 사용하여 Control System Toolbox 없이 구현하였다 (강의자료 7_1 "LQR: Stationary Case" 및 "Solution of the ARE", P.17-24). 추가로 입력 패턴 기반 gain scheduling을 적용하여 step 응답(A3)과 DLC(A1/D1)에서 서로 다른 응답 특성을 반영하였다.

각 제어기 요약:
- **ctrl_lateral**: 3차 augmented LQR (I-action) + ESC β-limiter + 입력 패턴 gain scheduling
- **ctrl_longitudinal**: 속도 추종 PI + 슬립 기반 per-wheel ABS (감압 + 추가 제동)
- **ctrl_vertical**: Hybrid skyhook + groundhook CDC (빈도 분리)
- **ctrl_coordinator**: ESC yaw moment → 4-wheel 차동 brake 분배 + ABS 감압 합산

---

## 2. 수학적 모델링

### 2.1 Plant 단순화 - 2-DOF Bicycle Model

제어기 설계에는 2-DOF bicycle model을 사용하였다. 이번 과제의 제어기 인터페이스에서는 roll/pitch/suspension 상태가 직접 입력으로 제공되지 않으므로, 14DOF plant 전체를 상태 피드백 대상으로 두지 않았다. Bicycle model은 횡방향 동역학의 핵심인 lateral velocity ($v_y$)와 yaw rate ($r$)의 coupling을 포착하면서 조향 입력($\delta$)에 대한 closed-form 상태공간 표현이 가능하다 [3, §2.5].

상태 변수: $x = [v_y,\ r]^T$, 입력: $u = \delta$ (road-wheel 조향각)

$$\dot{v}_y = -\frac{C_f + C_r}{m V_x} v_y + \left(\frac{l_r C_r - l_f C_f}{m V_x} - V_x\right) r + \frac{C_f}{m} \delta$$

$$\dot{r} = -\frac{l_f C_f - l_r C_r}{I_z V_x} v_y - \frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x} r + \frac{l_f C_f}{I_z} \delta$$

변수 의미는 다음과 같다. $v_y$는 차체 횡방향 속도, $r$은 yaw rate, $V_x$는 종방향 속도, $\delta$는 road-wheel 조향각이다. $m$은 차량 질량, $I_z$는 yaw 관성모멘트, $l_f$, $l_r$은 무게중심에서 전/후축까지의 거리, $C_f$, $C_r$은 전/후축 코너링 강성이다.

여기서 $C_f$, $C_r$은 **축 단위 코너링 강성** (= $2 \times$ 단륜 강성)이다. 이는 `calc_ref_yaw_rate.m`이 understeer gradient $K_{us}$를 산출할 때 사용하는 것과 동일한 정의이다:

$$K_{us} = \frac{m l_r}{2 C_f L} - \frac{m l_f}{2 C_r L}$$

여기서 $K_{us}$는 understeer gradient, $L=l_f+l_r$은 wheelbase이다. $K_{us}$가 클수록 같은 횡가속도에서 더 큰 조향각이 필요한 understeer 성향을 의미한다.

제어기가 참조 모델과 동일한 강성 정의를 사용해야 yaw rate 추종이 정상상태에서 편향 없이 수렴한다. 따라서 `VL_lqr.Cf` = 2 × 80,000 = 160,000 N/rad, `VL_lqr.Cr` = 2 × 85,000 = 170,000 N/rad으로 설정했다.

### 2.2 Augmented System - I-action

Yaw rate 정상상태 오차를 제거하기 위해 적분 상태 $\xi_r = \int (r_{ref} - r)\,dt$를 추가한 3차 augmented system을 구성한다 (강의 6_1):

$$z = [v_y,\ e_r,\ \xi_r]^T, \quad e_r = r - r_{ref}$$

$$A_{aug} = \begin{bmatrix} A & 0 \\ -C_{yaw} & 0 \end{bmatrix}, \quad B_{aug} = \begin{bmatrix} B \\ 0 \end{bmatrix}, \quad C_{yaw} = [0,\ 1]$$

$z$는 LQR에 사용하는 augmented state이다. $e_r$은 yaw rate 오차, $r_{ref}$는 driver 조향 입력으로부터 계산한 참조 yaw rate, $\xi_r$은 yaw rate 오차 적분 상태이다. $A$, $B$는 2-DOF bicycle model의 상태공간 행렬이고, $A_{aug}$, $B_{aug}$는 적분 상태를 포함한 확장 행렬이다. $C_{yaw}$는 상태 $[v_y,\ r]^T$ 중 yaw rate 성분만 선택하는 출력 행렬이다.

### 2.3 LQR - Hamiltonian ARE

비용 함수 $J = \int_0^\infty (z^T Q z + u^T R u)\,dt$를 최소화하는 게인 $K^* = R^{-1} B^T P$를 구하기 위해 CARE $A^T P + PA - PBR^{-1}B^T P + Q = 0$를 풀어야 한다. MATLAB `lqr`/`icare` 없이 Hamiltonian 행렬의 고유값 분해로 풀이한다 (강의 7_1, P.21-24):

$$H = \begin{bmatrix} A & -BR^{-1}B^T \\ -Q & -A^T \end{bmatrix}$$

$J$는 제어 성능과 입력 사용량을 합친 비용 함수이다. $Q$는 상태 오차 가중 행렬, $R$은 조향 입력 가중치, $u$는 AFS 조향 보정 입력이다. $P$는 Riccati 방정식의 해, $K^*$는 최적 피드백 게인, $H$는 Hamiltonian 행렬이다.

$H$의 $2n$개 고유값 중 안정한 $n$개에 대응하는 고유벡터 $V_s = [V_1;\ V_2]$로부터 $P = V_2 V_1^{-1}$을 얻는다. 이 방법은 강의의 double integrator 예제 (P.26-28)와 대조하여 해석해와 수치적으로 일치함을 확인하였다.

### 2.4 가정과 유효 범위

| 가정 | 유효 조건 | 위반 시나리오 |
|---|---|---|
| 선형 타이어 | $\lvert\alpha\rvert \leq 2.03°$ (MF $B$=12, $C$=1.6 기준) | A1/D1 sideSlip 3-5°, A7 비상제동 |
| 일정 종속도 | $V_x$ 변화가 작음 | B1 제동 (100→0 km/h) |
| 소롤각 | roll이 횡 동역학에 미미 | A1/D1 DLC 고횡가속 구간 |

**타이어 선형 영역 확인**: Magic Formula $F_y = D F_z \sin(C \arctan(B\alpha - E(B\alpha - \arctan(B\alpha))))$에서 선형 근사 오차가 5%를 초과하는 지점은 $|\alpha| \approx 2.03°$이다. 여기서 $F_y$는 횡력, $F_z$는 수직 하중, $\alpha$는 타이어 slip angle, $B,C,D,E$는 Magic Formula 형상 계수이다. 채점 기준의 sideSlip 목표 (A1: 3°, D1: 4°, A7: 5°)는 이 선형 영역보다 크므로, LQR 설계 모델과 14DOF plant 응답 사이에 차이가 생긴다.

**후축 강성 정의**: 제어기는 `calc_ref_yaw_rate.m`과 같은 축 단위 강성 정의를 사용하였다. 따라서 `VL_lqr.Cf`, `VL_lqr.Cr`에는 단륜 강성의 2배를 넣었다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral - AFS + ESC

#### 3.1.1 LQR 피드백

$Q = \text{diag}(Q_{v_y},\ Q_r,\ Q_{\xi_r})$, $R$ 선정에는 Bryson's rule을 기반으로 시뮬레이션 반복 튜닝을 수행하였다:

| 파라미터 | 값 | 근거 |
|---|---|---|
| $Q_{v_y}$ | 1.0 | Lateral velocity 억제 → sideSlip 감소 |
| $Q_r$ | 5.0 | Yaw rate 오차 추종 가중 |
| $Q_{\xi_r}$ | 0.25 | 적분기 기여 억제 (windup 방지) |
| $R$ | 50.0 | 조향 입력 보수적 사용 → overshoot 감소 |

$Q$는 augmented state $z=[v_y,\ e_r,\ \xi_r]^T$의 각 상태에 부여하는 가중 행렬이다. $Q_{v_y}$는 lateral velocity, $Q_r$은 yaw rate 오차, $Q_{\xi_r}$는 yaw rate 오차 적분 상태의 가중치이다. $R$은 AFS 조향 입력 사용량에 대한 가중치이다.

피드백 법칙: $\delta_{AFS} = -K z$, $K \in \mathbb{R}^{1 \times 3}$는 매 시간 스텝 $V_x$에 따라 재계산된다 (speed-dependent gain scheduling).

$\delta_{AFS}$는 driver 조향각 위에 더해지는 AFS 보조 조향각이고, $K$는 LQR 피드백 게인 벡터이다. $z$는 앞 절에서 정의한 augmented state이며, $V_x$는 현재 종방향 속도이다.

#### 3.1.2 보조 기법

**참조 필터**: yawRateRef에 1차 저역필터 ($\tau_{ref}$ = 0.08 s)를 적용하여 step 입력의 급격한 변화를 완화한다. 이로써 A3 overshoot를 2.81% → 1.98%로 감소시켰다.

$\tau_{ref}$는 참조 yaw rate 필터의 시정수이다. 값이 작을수록 참조 입력을 빠르게 따라가고, 값이 클수록 step 입력이 완만해진다.

**조건부 적분**: $|e_r| > 1$ °/s 일 때 적분을 정지하고, 적분 포화 한계 $|\xi_r| \leq 0.05$ rad을 설정하여 과도 구간에서 windup을 방지한다. 적분이 역방향으로 전환될 때는 즉시 활성화하여 정착을 가속한다.

$e_r$은 yaw rate 오차, $\xi_r$은 그 오차의 적분 상태이다. 조건부 적분은 큰 과도 오차 구간에서는 적분 누적을 멈추고, 오차를 줄이는 방향일 때 다시 적분을 허용하는 방식이다.

**미분 감쇠**: $d(r)/dt$ (yaw acceleration) 항을 피드백에 추가하여 진동을 억제한다. 기존의 $d(e_r)/dt$ 대신 측정값 $r$만 미분함으로써 reference step 입력에 의한 derivative kick을 제거하였다. 1차 필터 ($\tau_d$ = 0.08 s)로 고주파 잡음을 제거한다.

$d(r)/dt$는 yaw rate의 시간 미분값이고, $\tau_d$는 미분 신호에 적용한 1차 필터의 시정수이다.

#### 3.1.3 입력 패턴 기반 Gain Scheduling

시나리오 ID를 사용하지 않고 yawRateRef 물리 신호의 특성을 분류하여 모드별 최적 게인을 적용한다:

| 감지 기준 | 모드 | 주요 변경 |
|---|---|---|
| $r_{ref}$ 부호 변화 ≥ 1회 | **DLC** | AFS 크기 제한 3° (경로 보호) |
| $|\dot{r}_{ref}|$ > 80 °/s + 부호변화 없음 | **Fast** (0.25 s) | $\tau_{ref}$↓, $R$↓, $Q_{v_y}$↓ → 빠른 상승 |
| Fast 이후 잔여 시간 | **Settle** | $Q_r$↑, $K_d$↑ → 강한 감쇠 |
| 기타 | **Normal** | 기본 파라미터 |

$r_{ref}$는 참조 yaw rate이고, $\dot{r}_{ref}$는 그 시간 변화율이다. $K_d$는 yaw rate 미분 감쇠 게인이다. 표의 ↑/↓는 해당 모드에서 기본값 대비 가중치나 게인을 증가/감소시킨다는 의미이다.

이 분류기는 입력 신호의 시간적 특성(변화율, 부호 전환 횟수)만 사용하므로, 본 과제의 시나리오 범위에서는 시나리오 ID 없이 동작하도록 설계하였다.

#### 3.1.4 ESC β-limiter

차체 슬립각 $|\beta|$이 임계값 $\beta_{th}$ = 3°를 초과하면 복원 yaw moment를 인가한다:

$$M_z = -K_\beta \cdot \text{sign}(\beta) \cdot (|\beta| - \beta_{th}) \cdot f(V_x)$$

$M_z$는 ESC가 요구하는 복원 yaw moment, $K_\beta$는 슬립각 피드백 게인, $\beta$는 차체 slip angle, $\beta_{th}$는 개입 임계값, $f(V_x)$는 속도별 개입 크기 보정 함수이다. $K_\beta$ = 80,000 Nm/rad, $f(V_x) = \min(\max(V_x, 0)/20, 2)$로 속도 비례 스케일링한다. A7 brake-in-turn에서는 baseline sideSlip이 30.48°까지 증가했고, 제어기 ON에서는 이 β-limiter가 $|\beta|$ > 3° 조건에서 동작한다.

#### 3.1.5 횡요구 기반 안정화 감속

DLC처럼 참조 yaw rate가 연속적으로 변하는 고횡가속 구간에서는 작은 대칭 제동을 추가하여 속도와 좌우 하중이동을 낮춘다. 시나리오 ID는 사용하지 않고, $\dot{r}_{ref}$ 활동 시간이 0.08 s 이상 누적된 경우에만 활성화한다. 따라서 A3 step 입력처럼 순간적으로만 $\dot{r}_{ref}$가 큰 경우에는 감속이 지속되지 않는다.

$$T_{stab} = \min(T_{stab,max},\ K_{stab}\cdot \max(0,\ |r_{ref}|-r_{th})\cdot g_v\cdot g_{act})$$

$T_{stab}$는 각 바퀴에 동일하게 더하는 안정화 brake torque이다. $K_{stab}$ = 120 Nm/(rad/s), $T_{stab,max}$ = 260 Nm, $r_{th}$ = 8°/s이다. $g_v=\min(\max((V_x-12)/10,0),1)$는 고속 게이트이고, $g_{act}$는 $\dot{r}_{ref}$ 활동 시간이 기준을 넘었을 때 1이 되는 게이트이다. 이 항은 A1/D1에서 LTR을 낮추는 역할을 하며, 좌우 차동이 아니므로 yaw moment를 직접 만들지는 않는다.

### 3.2 ctrl_longitudinal - PI + ABS

#### 3.2.1 속도 추종 PI

$$F_x = K_p (V_{ref} - V_x) + K_i \int (V_{ref} - V_x)\,dt, \quad F_x \leq 0$$

$F_x$는 종방향 제어력이므로 음수이면 제동을 의미한다. $V_{ref}$는 목표 속도, $V_x$는 현재 종방향 속도, $K_p$, $K_i$는 각각 PI 제어기의 비례/적분 게인이다. $K_p$ = 800, $K_i$ = 80. 제동만 출력하며 ($F_x \leq 0$), 저크 제한 $|dF_x/dt| \leq m \cdot J_{max}$ ($J_{max}$ = 50 m/s³)을 적용한다.

#### 3.2.2 ABS - Per-wheel 슬립 제어

목표 슬립 비 $\lambda_{ref}$ = 0.10 (MF $B_x$=14, $C_x$=1.65 기준 $\mu$-peak 부근):

$\lambda_{ref}$는 ABS가 유지하려는 목표 longitudinal slip ratio이다. $B_x$와 $C_x$는 종방향 Magic Formula의 stiffness/shape 계수이고, $\mu$는 노면-타이어 마찰계수이다.

- **슬립 초과** ($\lambda_i > \lambda_{ref}$): 감압 $\Delta T_i = -\min(K_{rel} \cdot (\lambda_i - \lambda_{ref}),\ T_{relief,max})$
- **슬립 부족** ($\lambda_i < \lambda_{ref}$, 제동 중): 추가 제동 $\Delta T_i = +\min(K_{add} \cdot (\lambda_{ref} - \lambda_i),\ T_{add,max})$

$\lambda_i$는 각 바퀴의 longitudinal slip ratio, $\lambda_{ref}$는 목표 슬립 비, $\Delta T_i$는 각 바퀴에 더하거나 빼는 brake torque 보정량이다. $K_{rel}$은 슬립 초과 시 감압 게인, $K_{add}$는 슬립 부족 시 추가 제동 게인, $T_{relief,max}$와 $T_{add,max}$는 각 보정량의 상한이다. $K_{rel}$ = 7,000 Nm/slip, $K_{add}$ = 6,000 Nm/slip. 제동 게이팅 ($a_x < -3$ m/s²) 조건으로 비제동 시나리오(A3/A4)에서 ABS 오작동을 방지한다.

### 3.3 ctrl_vertical - Hybrid CDC

Skyhook (body bounce 1-2 Hz) + groundhook (wheel hop 10-15 Hz) 하이브리드:

$$F_{des} = -(\alpha \cdot c_{sky} \cdot \dot{z}_s + (1-\alpha) \cdot c_{gnd} \cdot \dot{z}_u)$$

$\alpha$ = 0.7 (skyhook 비중). Plant 댐퍼 $F_d = -c \cdot v_{rel}$ 매칭으로 목표 감쇠 계수를 산출한다:

$$c_{cmd} = \frac{\alpha \cdot c_{sky} \cdot \dot{z}_s + (1-\alpha) \cdot c_{gnd} \cdot \dot{z}_u}{v_{rel}}$$

$F_{des}$는 목표 댐퍼력, $\alpha$는 skyhook과 groundhook의 혼합 비율이다. $c_{sky}$와 $c_{gnd}$는 각각 sprung mass와 unsprung mass 속도에 대한 감쇠 게인, $\dot{z}_s$는 sprung mass 수직 속도, $\dot{z}_u$는 unsprung mass 수직 속도이다. $v_{rel}=\dot{z}_s-\dot{z}_u$는 댐퍼 상대속도, $c_{cmd}$는 plant에 전달하는 목표 감쇠 계수이다.

Semi-active 제약 ($c \geq 0$, 에너지 주입 불가)을 만족하지 못하면 $c_{min}$ = 800 Ns/m으로 설정한다. 최종 출력은 $c_{min} \leq c \leq c_{max}$ (= 6,000 Ns/m) 범위로 제한한다.

### 3.4 ctrl_coordinator - Actuator Allocation

ESC yaw moment $M_z$를 4-wheel 차동 brake로 분배한다:

$$\Delta T_f = |M_z| \cdot r_f \cdot \frac{r_w}{t_f/2}, \quad \Delta T_r = |M_z| \cdot (1-r_f) \cdot \frac{r_w}{t_r/2}$$

$\Delta T_f$, $\Delta T_r$은 각각 전/후축에 분배되는 차동 brake torque 크기이다. $r_f$는 전축 분배 비율, $r_w$는 wheel radius, $t_f$, $t_r$은 전/후 track width이다. 전후 비율 $r_f$ = 0.6. $M_z > 0$ (CCW)이면 좌측(FL/RL)에, $M_z < 0$ (CW)이면 우측(FR/RR)에 brake를 인가한다. ABS 감압 토크 (음수)를 합산하여 시나리오 brake를 감압하는 구조이다.

---

## 4. 시뮬레이션 결과

### 4.1 KPI 종합 - Baseline vs Designed Controller

아래 값은 2026-06-23 MATLAB R2025a에서 `scripts/grade.m`을 재실행한 결과를 기준으로 정리하였다. B1 stoppingDistance는 공식 기준 66.5 m를 적용하였다.

| 시나리오 | KPI | OFF | ON (본인) | 변화율 | 점수 |
|---|---|---:|---:|---:|---:|
| A3 Step | yawRateOvershoot [%] | 2.70 | **1.98** | -26.8% | 4/4 |
| A3 | yawRateRiseTime [s] | 0.247 | **0.157** | -36.4% | 4/4 |
| A3 | yawRateSettling [s] | 1.462 | **0.553** | -62.2% | 4/4 |
| A1 DLC | sideSlipMax [°] | 3.02 | **2.96** | -1.7% | 6/6 |
| A1 | LTR_max | 0.864 | **0.567** | -34.4% | 5/5 |
| A1 | lateralDevMax [m] | 1.827 | 2.054 | +12.4% | 0/4 |
| A4 SS | understeerGradient | 0.00075 | 0.00039 | -48.1% | 4.56/5 |
| A4 | sideSlipMax [°] | 1.18 | **1.16** | -2.0% | 5/5 |
| A7 BIT | sideSlipMax [°] | 30.48 | **3.78** | -87.6% | 8/8 |
| A7 | LTR_max | 0.681 | **0.414** | -39.1% | 7/7 |
| B1 Brake | stoppingDistance [m] | 72.30 | **64.86** | -10.3% | 5/5 |
| B1 | absSlipRMS | - | **0.085** | - | 5/5 |
| D1 DLC+Brake | sideSlipMax [°] | 4.91 | **2.96** | -39.6% | 4/4 |
| D1 | LTR_max | 0.864 | **0.567** | -34.4% | 2/2 |
| D1 | lateralDevMax [m] | 1.827 | 2.054 | +12.4% | 0/2 |
| | | | | **합계** | **63.56/70** |

![A1 DLC 3-way validation](figures/3way/vdb_3way_A1_20260523_221308.png)
*Figure 4.1 - A1 ISO 3888-1 DLC 3-way validation. M-file 14DOF, VDB, CarMaker 기준 응답을 비교하였다.*

![A4 steady-state circular 3-way validation](figures/3way/vdb_3way_A4_20260523_221323.png)
*Figure 4.2 - A4 steady-state circular 3-way validation. 정상선회 구간에서 종속도, 횡가속도, yaw rate, sideSlip 응답을 비교하였다.*

![B1 straight braking 3-way validation](figures/3way/vdb_3way_B1_20260523_221331.png)
*Figure 4.3 - B1 straight braking 3-way validation. 100 km/h 직선 제동에서 stopping distance와 ABS slip 응답을 확인하였다.*

### 4.2 시나리오 분석

**A3 Step Steer (12/12, 만점)**: 참조 필터 + derivative kick 제거 + fast/settle 단계 분리가 핵심이다. Fast 모드에서 $R$을 25로 낮추고 $\tau_{ref}$를 0.015 s로 줄여 빠른 상승을 유도하고, settle 모드에서 $K_d$를 0.03으로 높여 진동 없이 정착한다. Overshoot 1.98% (목표 10%), settling 0.553 s (목표 0.8 s).

**A7 Brake-in-Turn (15/15, 만점)**: 개선 폭이 가장 큰 시나리오이다. Baseline에서 $R$=100 m 선회 중 0.4$g$ 제동 시 sideSlip이 **30.48°**까지 증가하였다. 제어기 ON에서는 ESC β-limiter가 $|\beta|$ > 3°에서 차동 brake를 인가하며, 최종 sideSlip은 3.78°이다. LTR도 0.681 → 0.414로 감소하였다.

**A1 ISO 3888-1 DLC (11/15)**: sideSlip은 2.96°로 만점(6/6)이다. DLC 모드에서는 AFS 크기를 3°로 제한하고, 참조 yaw rate가 연속적으로 변하는 구간에서는 대칭 안정화 brake를 추가한다. 이로써 LTR은 0.864 → 0.567로 감소해 만점(5/5)을 확보하였다. 다만 lateralDev는 ON(2.054 m)이 OFF(1.827 m)보다 크다 (0/4). §5에서 잔여 감점 항목을 정리한다.

**B1 Straight Brake (10/10, 만점)**: ABS로 absSlipRMS 0.085를 달성했고, stoppingDistance 64.86 m도 만점 기준 66.5 m 이내이다.

---

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 시나리오 - A7

A7 brake-in-turn은 baseline 대비 가장 큰 절대 개선 (sideSlip 30.48° → 3.78°)을 보인 시나리오이다. 이 구간에서는 ESC β-limiter가 $|\beta|$ > 3°에서 차동 제동을 인가한다.

A7의 baseline sideSlip은 30.48°로 선형 bicycle model의 유효 범위를 벗어난다. 제어기 ON 결과에서는 slip angle 기반 ESC 개입 후 sideSlip이 3.78°로 낮아졌다.

### 5.2 잔여 감점 항목

만점을 달성하지 못한 KPI는 A1/D1 lateralDev와 A4 understeerGradient이다. 각 항목을 현재 제어기 입력과 출력 범위 기준으로 정리한다.

#### 5.2.1 lateralDevMax (A1: 0/4, D1: 0/2)

`ctrl_lateral`의 입력은 `(yawRateRef, yawRate, slipAngle, vx)`이며, 횡방향 편차 $e_y$ (lateral deviation)가 제공되지 않는다. 따라서 제어기는 경로 추종을 직접 수행할 수 없고, yaw rate 추종만 가능하다.

경로 오차 기반 횡방향 제어에서 흔히 사용하는 상태 $x = [e_y,\ \dot{e}_y,\ e_\psi,\ \dot{e}_\psi]^T$와 비교하면, 우리 설계에는 $e_y$와 $e_\psi$ 상태가 제공되지 않는다. 따라서 현재 제어기 인터페이스는 lateralDevMax를 직접 목적함수로 다루지 못한다.

결과에서도 OFF(1.827 m) → ON(2.054 m)으로 lateralDev가 증가하였다. 본 설계는 yaw rate와 slip angle을 제어 대상으로 두었고, 경로 오차는 직접 피드백하지 않았다.

#### 5.2.2 LTR_max

LTR (Load Transfer Ratio)는 좌우 수직 하중 비에 의해 결정되며, DLC에서 peak 횡가속도 $a_y$와 롤 강성이 지배한다:

$$\text{LTR} \approx \frac{2 h_{cog} \cdot a_y}{g \cdot t_{track}}$$

여기서 $h_{cog}$는 무게중심 높이, $a_y$는 횡가속도, $g$는 중력가속도, $t_{track}$은 track width이다. $h_{cog}$, $t_{track}$은 차량 기하 파라미터이고, 제어기 출력에는 roll stiffness를 직접 바꾸는 액추에이터가 없다. 따라서 본 설계에서는 고횡가속 구간에서 작은 대칭 제동을 추가하여 속도와 횡가속도를 낮추는 간접 방식을 사용하였다. 이 방식으로 A1/D1의 LTR은 0.567까지 감소했지만, 경로 오차를 직접 줄이지는 못했다.

#### 5.2.3 B1 stoppingDistance

B1 시나리오에서 stoppingDistance는 64.86 m이고, 기준 66.5 m 이내이다. absSlipRMS는 0.085로 기준 0.10 이내이다. 따라서 B1은 두 KPI 모두 만점이다.

제동 성능 계산에 사용된 주요 값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| 초기 속도 | 100 km/h (27.78 m/s) |
| stoppingDistance | 64.86 m |
| stoppingDistance 기준 | 66.5 m |
| absSlipRMS | 0.085 |
| absSlipRMS 기준 | 0.10 |

#### 5.2.4 A4 understeerGradient (4.56/5)

본 결과에서 A4 understeerGradient는 OFF 0.00075, ON 0.00039이다. 목표 $K_{us}$ = 0.003 (±80%)와 비교하면, 제어기 ON에서 점수가 4.56/5로 남았다.

정상선회에서 AFS 개입을 줄이는 방향은 A4 점수 개선 후보이다. 다만 같은 횡방향 제어 파라미터가 A3, A7, D1에도 적용되므로, 최종 설계에서는 A3/A7 만점과 D1 통과 점수를 유지하는 쪽을 선택하였다.

### 5.3 잔여 감점 요약

| KPI | 미달분 | 확인된 제약 |
|---|---|---|
| A1/D1 lateralDev | 6점 | 제어기 입력에 $e_y$, $e_\psi$ 없음 |
| A1/D1 LTR | 0점 | 안정화 감속으로 목표 이내 달성 |
| D1 sideSlip | 0점 | 안정화 감속과 ESC로 목표 이내 달성 |
| A4 understeer | 0.44점 | AFS 개입과 정상선회 KPI 간 상충 |

합계 미달은 6.44점이다. 현재 제출 설계는 A3, A7, B1을 만점으로 유지하고, A1/D1의 slip 및 LTR 점수를 확보하는 방향으로 정리하였다.

### 5.4 추가 개선 검토 항목

1. **4차 LQR + $e_y$ 관측기**: $x = [e_y, \dot{e}_y, e_\psi, \dot{e}_\psi]^T$ 구조를 적용하려면 $e_y$를 추정하는 관측기가 필요하다. 현재 인터페이스에서 적분 ($e_y \approx \int v_y\,dt$)으로 근사할 수 있으나, 누적 drift 검증이 필요하다.
2. **MPC**: MPC는 constraint를 명시적으로 처리할 수 있어, 마찰원 제한과 LTR 제한을 직접 비용 함수에 포함시킬 수 있다. 다만 14DOF plant에 대한 online 최적화가 1 ms 제어 주기 안에 계산되는지 검증해야 한다.
3. **능동 롤 스태빌라이저**: CDC 대신 active anti-roll bar가 있으면 롤 강성을 동적으로 변경하여 LTR을 직접 제어할 수 있다. 현재 plant는 이 액추에이터를 제공하지 않는다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018 - Passenger cars - Test track for a severe lane-change manoeuvre - Part 1: Double lane-change.

[2] ISO 4138:2021 - Passenger cars - Steady-state circular driving behaviour - Open-loop test methods.

[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.

[4] 자동제어특론 강의자료 6_1 - *State Feedback* (State Variable Feedback with I-action, P.36-49).

[5] 자동제어특론 강의자료 7_1 - *LQR (CT)*, Linear Quadratic Regulators (Stationary Case / ARE, P.17-24).

[6] H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann/Elsevier, 2012.

---

## 부록 A - 사용한 AI 도구

**Claude Code** 및 **Codex**를 다음 범위에서 활용하였다:

| 활용 범위 | 구체적 내용 |
|---|---|
| LQR 설계 | Hamiltonian ARE solver 구현 + 강의 예제 대조 검증 |
| Q/R 튜닝 | Bryson's rule 기반 초기값 제안 → 시뮬레이션 반복 조정 |
| ABS 설계 | 슬립 기반 감압/추가 제동 로직 구현 |
| CDC 설계 | Hybrid skyhook+groundhook 구조 + 빈도 분리 |
| 가정 검증 | 타이어 선형 영역 계산, 강성 정의 확인 |
| 결과 분석 | KPI 표 작성, 잔여 감점 항목 정리 |
| MATLAB 디버깅 | 한국어 경로 문제, struct 초기화 오류 등 |

최종 제출값은 시뮬레이션 결과와 `grade_report.json` 수치를 확인한 뒤 확정하였다.

---

## 부록 B - `sim_params.m` 주요 변경사항

`config/sim_params.m`에서는 과제 허용 범위인 `CTRL.*` 항목만 제어기 파라미터로 사용하였다. 핵심 설정은 다음과 같다.

| 구분 | 파라미터 | 값 | 목적 |
|---|---|---:|---|
| Lateral LQR | `CTRL.LAT.Q_vy` | 1.0 | lateral velocity 억제 |
| Lateral LQR | `CTRL.LAT.Q_r` | 5.0 | yaw rate 추종 가중 |
| Lateral LQR | `CTRL.LAT.Q_xi_r` | 0.25 | 적분 상태 windup 억제 |
| Lateral LQR | `CTRL.LAT.R_lqr` | 50.0 | 조향 입력 사용량 제한 |
| Step mode | `CTRL.LAT.R_settle` | 40.0 | A3 정착 구간 조향 응답 조정 |
| Step mode | `CTRL.LAT.Kd_settle` | 0.03 | A3 정착 구간 yaw rate 감쇠 |
| DLC mode | `CTRL.LAT.afs_dlc` | 3.0 deg | DLC 구간 AFS 보정각 제한 |
| ESC | `CTRL.LAT.K_beta` | 80000 Nm/rad | slip angle 초과 시 복원 yaw moment |
| ESC | `CTRL.LAT.beta_th` | 3.0 deg | ESC 개입 임계 slip angle |
| ABS | `CTRL.LON_lambda_ref` | 0.10 | 목표 longitudinal slip ratio |
| ABS | `CTRL.LON_K_rel` | 7000 Nm/slip | 슬립 초과 시 brake torque 감압 |
| CDC | `CTRL.VER.cMin` | 800 Ns/m | 최소 감쇠 계수 |
| CDC | `CTRL.VER.cMax` | 6000 Ns/m | 최대 감쇠 계수 |
| CDC | `CTRL.VER.hybridAlpha` | 0.7 | skyhook 비중 |
