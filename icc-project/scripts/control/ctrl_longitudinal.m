function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 PI + ABS 슬립 피드백)
%
%   구조 (신버전 run_icc_scenario):
%   - runner가 ctrlState.wheelSlip(4)로 직전 스텝 휠 슬립 비율을 전달
%   - brake_total = brk_scenario + brakeESC(coordinator 출력)
%     → ABS는 슬립 초과 바퀴에 "음의 brake"(감압)를 더해 잠김 해제
%   - per-wheel 감압량을 forceCmd.absBrake(4×1)로 coordinator에 전달
%
%   ABS 원리: 슬립을 λ_ref(≈0.12) 부근 유지 → 잠김 방지로
%             stoppingDist↓ + absSlipRMS↓ 동시 개선
%
%   출력:
%      forceCmd.Fx_total   - 속도추종 종방향 힘 [N] (제동 시 음수)
%      forceCmd.brakeRatio - 0~1
%      forceCmd.absBrake   - 4×1 ABS 감압 토크 [Nm] (음수, coordinator가 합산)

    %% 파라미터
    Kp_v       = getfield_default(CTRL, 'LON_Kp_v',   800);
    Ki_v       = getfield_default(CTRL, 'LON_Ki_v',   80);
    Fx_min     = getfield_default(CTRL, 'LON_Fx_min', -14000);
    lambda_ref = getfield_default(CTRL, 'LON_lambda_ref', 0.12);
    K_rel      = getfield_default(CTRL, 'LON_K_rel',  9000);   % Nm per unit slip
    relief_max = getfield_default(CTRL, 'LON_relief_max', 1300);

    %% 상태 초기화
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0; end

    %% 1. 속도 추종 PI (제동만 — Fx ≤ 0)
    e_v = vxRef - vx;
    ctrlState.intError = ctrlState.intError + e_v * dt;
    intMax = getfield_default(CTRL, 'LON_intMax', 50);
    ctrlState.intError = max(-intMax, min(intMax, ctrlState.intError));
    Fx_pi = Kp_v * e_v + Ki_v * ctrlState.intError;
    Fx_pi = min(0, Fx_pi);
    Fx_total = max(Fx_min, Fx_pi);

    %% 1b. 저크 제한 (요구 §3.2-3): |dFx/dt| <= m·J_max
    %   jerk j = d(ax)/dt = (1/m)·dFx/dt → |ΔFx| <= m·J_max·dt
    m_veh  = getfield_default(CTRL.LAT, 'VEH', struct('mass',1500));
    if isstruct(m_veh); m_veh = m_veh.mass; end
    J_max  = getfield_default(LIM, 'MAX_JERK', 50);
    dFx_max = m_veh * J_max * dt;
    dFx = Fx_total - ctrlState.prevForce;
    dFx = max(-dFx_max, min(dFx_max, dFx));
    Fx_total = ctrlState.prevForce + dFx;
    ctrlState.prevForce = Fx_total;

    %% 2. ABS — 슬립 기반 per-wheel 조절 (제동 상황에서만 활성)
    %   제동 게이팅: 실제 강한 감속(ax < ax_brake_th) 중일 때만 ABS 개입
    %     → 제동 명령이 없는 선회 구간에서 오작동 방지
    %   λ_i > λ_ref: brake 감소 (잠김 방지)
    %   λ_i < λ_ref: brake 증가 (노는 바퀴 활용 → stoppingDist↓)
    ax_brake_th = getfield_default(CTRL, 'LON_ax_brake_th', -3.0);  % [m/s²]
    K_add       = getfield_default(CTRL, 'LON_K_add',   6000);
    add_max     = getfield_default(CTRL, 'LON_add_max', 1500);

    lambda   = abs(ctrlState.wheelSlip(:));
    slip_def = lambda_ref - lambda;
    absBrake = zeros(4,1);

    braking_now = (ax < ax_brake_th);   % 강한 감속 중 = 제동 상황
    for i = 1:4
        if slip_def(i) < 0              % 슬립 초과 → 감압 (항상 허용: 안전)
            absBrake(i) = -min(K_rel * (-slip_def(i)), relief_max);
        elseif braking_now             % 슬립 부족 + 제동중 → 추가 제동
            absBrake(i) =  min(K_add * slip_def(i), add_max);
        end
    end

    %% 출력
    forceCmd.Fx_total   = Fx_total;
    forceCmd.brakeRatio = min(max(-Fx_total / abs(Fx_min), 0), 1);
    forceCmd.absBrake   = absBrake;   % 4×1, coordinator가 brakeTorque에 합산

end
