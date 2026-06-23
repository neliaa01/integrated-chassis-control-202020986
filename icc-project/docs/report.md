# [202020986-구민상] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요 (1 페이지)

본 보고서에서는 차량의 종방향 및 횡방향 주행 안정성을 동시에 확보하기 위한 통합 새시 제어 시스템(Integrated Chassis Control)을 설계하고 시뮬레이션을 통해 그 유효성을 검증하였다. 차량이 고속 급선회(A1, A7) 및 슬라롬(D1) 등 타이어 한계 마찰원 경계에 도달하는 극한 상황에서 발생하는 오버스티어 및 스핀아웃 현상을 억제하고, 안정적인 선회 궤적을 유지하는 것을 주 목적으로 한다.

이를 달성하기 위해 횡방향 제어에는 **차속 가변형 Gain Scheduling PID 제어**와 **비선형 $\beta$-Limiter 기반의 ESC 제어**를 통합 적용하였다. 고속 거동 시 타이어 마찰 포화로 인해 차량 플랜트가 극심한 비선형성(Non-linear Region)을 보이기 때문에, 선형 영역을 가정한 상태 공간 기반의 LQR이나 pole placement 제어는 모델 불확실성(Model Uncertainty)에 따른 입력 발산 위험이 존재한다. 반면, 직관적인 강인성(Robustness)이 입증된 PID 기반 구조에 차속 반비례 게인 스케일링을 결합함으로써 고속 안정성을 최대로 확보할 수 있음을 정당화하였다. 

종방향 제어에는 Anti-windup 기믹을 포함한 속도 추종 PI 제어를 설계하여 종·횡방향 간섭을 최소화하는 하이브리드 제어 파이프라인을 구축하였다.

### 각 제어기 한 줄 요약
* **ctrl_lateral**: 차속 가변 Gain Scheduling PID 제어로 yaw rate 추종 + 비선형 사하중 영역(Dead-zone) 기법으로 $\beta$-limiter 구현
* **ctrl_longitudinal**: Anti-windup 제한기가 포함된 오차 감지형 PI 속도 추종 제어 적용
* **ctrl_vertical**: 기본 수동 댐핑 매핑(Pass-through) 적용 및 종·횡 능동 안전 제어 집중
* **ctrl_coordinator**: 상위 횡/종방향 제어 명령(Fx_total, yawMoment)을 수용하는 정적 물리 분배 메커니즘 활용

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화
실제 검증은 14자유도(DOF)의 고충실도 전차량 풀 플랜트 모델(Full Vehicle Plant Model)에서 수행되지만, 능동 제어기의 수학적 설계 및 게인 튜닝의 직관성을 확보하기 위해 제어계 설계 단계에서는 횡방향 2자유도 자전거 모델(2-DOF Bicycle Model)을 대용 플랜트로 채택하여 선형화하였다.

### 2.2 State-space 표현
차량의 횡방향 상태 변수를 횡속도 $v_y$와 요레이트 $r$로 정의하고, 제어 입력을 전륜 조향각 $\delta$로 설정한 상태 공간 방정식은 다음과 같습니다.

$\dot{x} = Ax + Bu, \quad y = Cx + Du$

여기서 상태 벡터 $x$와 입력 $u$는 다음과 같습니다.

x = [v_y; r],  u = delta

행렬 A와 B의 정의:
A = [ -(Cf + Cr)/(m*Vx),  (lr*Cr - lf*Cf)/(m*Vx) - Vx;
      (lr*Cr - lf*Cf)/(Iz*Vx),  -(lf^2*Cf + lr^2*Cr)/(Iz*Vx) ]

B = [ Cf/m;
      (lf*Cf)/Iz ]

이를 미분 방정식으로 전개하면 다음과 같습니다.

$\dot{v}_y = -\frac{C_f + C_r}{m V_x} v_y + \left( \frac{l_r C_r - l_f C_f}{m V_x} - V_x \right) r + \frac{C_f}{m} \delta$

$\dot{r} = \frac{l_r C_r - l_f C_f}{I_z V_x} v_y - \frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x} r + \frac{l_f C_f}{I_z} \delta$

---
### 2.3 가정 + 한계
1. **일정 종속도 가정**: 각 타임스텝 내에서 종방향 차속 $V_x$는 일정하다고 가정하여 선형 시변(LTV) 시스템으로 근사 분리하였다.
2. **선형 타이어 마찰 영역 제한**: 타이어의 슬립각이 매우 작은 영역($\alpha < 3^\circ$) 내에서 코너링 포스가 슬립각에 선형 비례한다는 가정을 사용한다. 따라서 마찰 포화가 발생하는 한계 주행 영역(High-$\mu$ 급선회)에서는 해당 선형 모델 기반의 제어 입력에 왜곡이 발생할 수 있으므로, 비선형 사하중 슬립 임계 구동기($\beta$-limiter)를 별도로 독립 배치하여 상호 보완하도록 설계하였다.

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC
* **설계 목표**: 횡방향 요레이트 추종성 확보(정착 시간 $T_s < 0.8\text{s}$, 오버슛 $M_p < 10\%$), 급격한 레인 체인지 시 차체 슬립각 최댓값 억제($|\beta| > \beta_{th}$ 시 ESC 강제 복원 모멘트 개입).
* **선택 기법**: **Speed-adaptive Gain Scheduling PID + Non-linear $\beta$-Limiter**
* **Gain 계산 과정**: 
  지글러-니콜스(Ziegler-Nichols) 한계 주기법의 기본 조정 메커니즘을 벤치마킹하였다. 임계 게인 $K_{cr}$을 포착한 후 감쇄비($\zeta$)가 약 0.707 수준의 최적 감쇄 특성을 갖도록 시뮬레이션 반복 수행(Iteration)을 거쳤다. 추가적으로 차속 $V_x$가 증가함에 따라 조향 감도가 극도로 예민해져 오버슛이 폭발하는 현상을 방지하기 위해, 다음과 같은 차속 가변형 스케일링 함수 $f(V_x)$를 제어 루프에 내장하였다.
  
  $$speedGain = \frac{12}{\max(V_x, 6)}, \quad speedGain \in [0.52, 0.95]$$

  동시에 요레이트 오차의 급격한 변화에 따른 고주파 노이즈 및 서보 떨림을 억제하고자 미분 제어 항에 시정수 $\tau = 0.20$ 초의 1차 저역통과필터(LPF)를 결합한 불완전 미분기(Filtered Derivative)를 설계하였다.
  
  $$\alpha = \frac{dt}{\tau + dt}, \quad \dot{e}_{filtered} = \alpha \cdot \dot{e}_{raw} + (1 - \alpha) \cdot \dot{e}_{prev\_filtered}$$

* **최종 게인 + 정당화**:
```matlab
% 상위 런너 기본 게인 매핑 및 구조적 스케일링
CTRL.LAT.Kp = 2.50;  % 선회 강성 대응 기본 비례 게인
CTRL.LAT.Ki = 0.40;  % 정상상태 오차 제거용 적분 게인
CTRL.LAT.Kd = 0.08;  % 오실레이션 억제를 위한 미분 게인

% 비선형 ESC 임계 영역 설정
BETA_THRESHOLD = 0.60 * LIM.MAX_SLIP_ANGLE; % 차체 슬립 임계 사하중 경계
K_BETA_STEER = 0.08;   % 슬립 과다 시 AFS 보정 스티어 복원 게인
K_BETA_MOMENT = 9500;  % 슬립 임계 초과 시 ESC 차동 제동 요모멘트 게인

### 3.2 ctrl_longitudinal — 속도 + ABS
* **선택 기법**: Anti-windup PI 제어
* **구조**: `vxRef < vx - 1.0` 조건문을 통해 선회/가속 루프와 제동 루프를 공간적으로 분리.
* **최종 게인**: `CTRL.LON.Kp = 1.0; CTRL.LON.Ki = 0.1;` (Anti-windup 캡핑 적용)

### 3.3 ctrl_vertical — CDC
* 기본 수동 댐핑 매핑(Pass-through) 구조를 사용하여 횡방향 안정성 제어에 집중.

### 3.4 ctrl_coordinator — Actuator Allocation
* 상위 명령($F_x, M_z$)을 수용하여 차량 물리 모델에 따른 4륜 차동 제동 토크 분배 수행.

---

## 4. 시뮬레이션 결과
### 4.1 P1 시나리오 benchmark
* **최종 점수: 51.09 / 70.00**
* A4(SS), A7(BIT) 등 주요 선회 시나리오에서 만점 달성.

### 4.2 핵심 plot
*(Figure 4.1: A1 DLC Trajectory, Figure 4.2: A1 Yaw rate 응답 비교 그래프 첨부)*

### 4.3 A7 Brake-in-Turn 분석
* 베이스라인 $46.3^\circ$의 스핀아웃을 본 설계 적용 후 **$1.96^\circ$**로 억제하여 8점 만점 획득.

---

## 5. 분석 + 한계
### 5.1 가장 성공적이었던 시나리오
A4, A7에서 조향 게인과 비선형 β-limiter의 상호 보완적 튜닝으로 주행 안정성 극대화.

### 5.2 가장 부족했던 시나리오 (자기비판)
B1(직선 제동)의 0점 과락은 상위 런너와 제어기 간의 액추에이터 분배 메커니즘 정적 바인딩에 따른 파이프라인 락(Lock) 현상으로 판단됨.

### 5.3 만약 더 시간이 있었다면
WLS(Weighted Least Squares) allocation 최적화 기법을 도입하여 종방향 제동 토크의 동적 할당 알고리즘을 구현했을 것임.

---

## 6. 참고문헌
[1] ISO 3888-1 / 4138
[2] Rajamani, *Vehicle Dynamics and Control*
[3] Wong, *Theory of Ground Vehicles*

---

## 7. 부록 A — 사용한 AI 도구
* ChatGPT: Used for code refactoring and structural debugging of control loops.

---

## 8. 부록 B — 본인 sim_params.m 변경사항
```matlab
CTRL.LAT.Kp = 2.50; CTRL.LAT.Ki = 0.40; CTRL.LAT.Kd = 0.08;
CTRL.LON.Kp = 1.00; CTRL.LON.Ki = 0.10;