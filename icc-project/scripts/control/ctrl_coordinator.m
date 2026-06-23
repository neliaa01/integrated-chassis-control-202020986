function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

% 1. 조향각 통과 및 기본 배열 선언
    actuatorCmd.steerAngle = min(max(latCmd.steerAngle, -LIM.MAX_STEER_ANGLE), LIM.MAX_STEER_ANGLE);
    actuatorCmd.dampingCoeff = zeros(4,1);
    
    brakeTorque = zeros(4, 1);
    r_w = 0.33; 
    
    % 2. 명세서 요구사항 1: 종방향 제동력 분배 (전후 비율 60:40)
    if lonCmd.Fx_total < 0
        T_total = abs(lonCmd.Fx_total) * r_w;
        brakeTorque(1) = (T_total * 0.60) / 2; % FL
        brakeTorque(2) = (T_total * 0.60) / 2; % FR
        brakeTorque(3) = (T_total * 0.40) / 2; % RL
        brakeTorque(4) = (T_total * 0.40) / 2; % RR
    end
    
    % 3. 명세서 요구사항 2: ESC yaw moment -> 차동 브레이크 분배
    if abs(latCmd.yawMoment) > 1e-3
        t_f = VEH.track_f;
        t_r = VEH.track_r;
        ratio_f = 0.60;
        
        dT_f = (latCmd.yawMoment * ratio_f / (t_f / 2)) * r_w;
        dT_r = (latCmd.yawMoment * (1 - ratio_f) / (t_r / 2)) * r_w;
        
        % 주석 양의 M_z (CCW) -> 좌측 brake 증가 기믹 매핑 반영
        brakeTorque(1) = brakeTorque(1) - dT_f; % FL
        brakeTorque(2) = brakeTorque(2) + dT_f; % FR
        brakeTorque(3) = brakeTorque(3) - dT_r; % RL
        brakeTorque(4) = brakeTorque(4) + dT_r; % RR
    end
    
    % 4. 최종 출력 캡핑 및 Saturation 클리핑
    actuatorCmd.brakeTorque = min(max(brakeTorque, 0), LIM.MAX_BRAKE_TRQ);
end