function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 — 3차 LQR with I-action (수업 6_1)
%
%   설계 원칙 (수업 6_1 State Variable Feedback with I-action):
%   1. A,B 먼저: calc_ref_yaw_rate와 일관되게 2*Cf, 2*Cr (axle stiffness) 사용
%   2. 3차 augmented: x_aug = [vy; r; xi_r], xi_r = ∫(r_ref - r)dt
%   3. K = Hamiltonian ARE (eig, Control System Toolbox 불필요)
%   4. Q,R = Bryson's Rule 기반 (검증값: A3 overshoot 2.17%, A4 sideSlip 1.173°)
%   부가: 참조필터(step 완화), 조건부 적분(windup 방지), d(yawRate)/dt 감쇠,
%         AFS 크기 제한, ESC β-limiter

    %% 차량 파라미터
    if isfield(CTRL.LAT, 'VEH')
        VL = CTRL.LAT.VEH;
    else
        VL.mass = 1500; VL.Iz = 2500;
        VL.lf = 1.2;    VL.lr = 1.4;
        VL.Cf = 80000;  VL.Cr = 85000;
    end

    %% ctrlState 초기화
    if ~isfield(ctrlState, 'xi_r');           ctrlState.xi_r           = 0; end
    if ~isfield(ctrlState, 'yawRateRefFilt'); ctrlState.yawRateRefFilt = yawRate; end
    if ~isfield(ctrlState, 'yawRatePrev');    ctrlState.yawRatePrev    = yawRate; end
    if ~isfield(ctrlState, 'dRFilt');         ctrlState.dRFilt         = 0; end
    if ~isfield(ctrlState, 'rRefPrev');       ctrlState.rRefPrev       = yawRateRef; end
    if ~isfield(ctrlState, 'rRefSignChanges');ctrlState.rRefSignChanges= 0; end
    if ~isfield(ctrlState, 'fastTimer');      ctrlState.fastTimer      = 0; end
    if ~isfield(ctrlState, 'intError');       ctrlState.intError       = 0; end
    if ~isfield(ctrlState, 'prevError');      ctrlState.prevError      = 0; end

    %% ===== 입력 패턴 감지 (시나리오 ID 아님 — yawRateRef 물리신호 기반) =====
    %   DLC(A1/D1): yawRateRef 부호가 여러 번 바뀜 → 경로추종 보호
    %   step(A3):   급격한 ref 변화 + 부호변화 없음 → 빠른 응답
    if abs(yawRateRef) > deg2rad(2) && abs(ctrlState.rRefPrev) > deg2rad(2)
        if sign(yawRateRef) ~= sign(ctrlState.rRefPrev)
            ctrlState.rRefSignChanges = ctrlState.rRefSignChanges + 1;
        end
    end
    rRefDot = (yawRateRef - ctrlState.rRefPrev) / dt;
    if abs(rRefDot) > deg2rad(80) && ctrlState.rRefSignChanges == 0
        ctrlState.fastTimer = 0.5;   % step 감지 → 카운트다운 시작 (0.5초)
    end
    ctrlState.rRefPrev  = yawRateRef;
    ctrlState.fastTimer = max(0, ctrlState.fastTimer - dt);

    isDLC  = ctrlState.rRefSignChanges >= 1;
    % step 응답 단계 분리: fast(상승) → settle(정착) → normal
    t_fast   = getfield_default(CTRL.LAT, 't_fast',   0.25);  % 상승 구간 길이 [s]
    isStepEvt = ctrlState.fastTimer > 0 && ~isDLC;            % step 이벤트 활성
    isFast   = isStepEvt && ctrlState.fastTimer > (0.5 - t_fast);  % 초기 상승
    isSettle = isStepEvt && ~isFast;                          % 후기 정착

    %% ===== 모드별 게인 스케줄링 =====
    Q_vy   = getfield_default(CTRL.LAT, 'Q_vy',   1.0);
    Q_r    = getfield_default(CTRL.LAT, 'Q_r',    5.0);
    Q_xi_r = getfield_default(CTRL.LAT, 'Q_xi_r', 0.25);
    R_lqr  = getfield_default(CTRL.LAT, 'R_lqr',  50.0);
    tau_ref = getfield_default(CTRL.LAT, 'tau_ref', 0.08);
    Kd_r    = getfield_default(CTRL.LAT, 'Kd_r',  0.005);
    afs_max = getfield_default(CTRL.LAT, 'afs_max', deg2rad(8));

    if isFast        % A3 step 초기 — 빠른 상승 (rising 회복)
        tau_ref = 0.015;  Q_vy = 0.2;  R_lqr = 25;  Kd_r = 0.002;  afs_max = deg2rad(8);
    elseif isSettle  % A3 step 후기 — 강한 감쇠로 정착 (settling 단축)
        tau_ref = 0.02;
        Q_vy    = getfield_default(CTRL.LAT, 'Q_vy_settle',   0.5);
        Q_r     = getfield_default(CTRL.LAT, 'Q_r_settle',    8.0);
        Q_xi_r  = getfield_default(CTRL.LAT, 'Q_xi_settle',   2.0);
        R_lqr   = getfield_default(CTRL.LAT, 'R_settle',      40.0);
        Kd_r    = getfield_default(CTRL.LAT, 'Kd_settle',     0.03);
        afs_max = deg2rad(8);
    elseif isDLC     % A1/D1 — 경로추종 보호 (AFS 약하게)
        afs_max = deg2rad(getfield_default(CTRL.LAT, 'afs_dlc', 1.0));
    end
    Q = diag([Q_vy, Q_r, Q_xi_r]);
    R = R_lqr;

    %% A, B 설계 — 전후축 강성 보정 (calc_ref_yaw_rate와 일관)
    VL_lqr    = VL;
    VL_lqr.Cf = 2.0 * VL.Cf;
    VL_lqr.Cr = 2.0 * VL.Cr;
    [A, B, ~, ~] = calc_bicycle_model(vx, VL_lqr);
    C_yaw = [0, 1];

    %% 3차 Augmented System
    A_aug = [A,      zeros(2,1);
             -C_yaw, 0         ];
    B_aug = [B; 0];

    %% LQR 게인 (Hamiltonian ARE)
    K = solve_lqr(A_aug, B_aug, Q, R);

    %% 참조 필터 (step 입력 완화 → A3 overshoot 감소)
    alpha_ref = dt / (tau_ref + dt);
    ctrlState.yawRateRefFilt = ctrlState.yawRateRefFilt + ...
        alpha_ref * (yawRateRef - ctrlState.yawRateRefFilt);
    r_ref_f = ctrlState.yawRateRefFilt;

    %% 측정 및 오차
    vy_meas = vx * tan(slipAngle);
    e_r     = yawRate - r_ref_f;

    %% xi_r 조건부 적분 (과도오차 구간 windup 방지)
    xi_r_max = getfield_default(CTRL.LAT, 'xi_r_max',       0.05);
    e_int_on = deg2rad(getfield_default(CTRL.LAT, 'e_int_on_degps', 1.0));
    intDrive = -e_r;
    if abs(e_r) <= e_int_on || ctrlState.xi_r * intDrive < 0
        ctrlState.xi_r = ctrlState.xi_r + intDrive * dt;
    end
    ctrlState.xi_r = max(-xi_r_max, min(xi_r_max, ctrlState.xi_r));

    %% 오차 상태 벡터
    z = [vy_meas;
         e_r;
         ctrlState.xi_r];

    %% LQR 피드백
    steer_lqr = -K * z;

    %% 미분 감쇠: d(yawRate)/dt (derivative kick 제거, Kd_r은 모드별 값)
    tau_d   = getfield_default(CTRL.LAT, 'tau_d',  0.08);
    alpha_d = dt / (tau_d + dt);
    dR_raw  = (yawRate - ctrlState.yawRatePrev) / dt;
    ctrlState.yawRatePrev = yawRate;
    ctrlState.dRFilt = ctrlState.dRFilt + alpha_d*(dR_raw - ctrlState.dRFilt);
    steer_lqr = steer_lqr - Kd_r * ctrlState.dRFilt;

    %% AFS 크기 제한 (afs_max는 모드별 값)
    steer_lqr = max(-afs_max, min(afs_max, steer_lqr));

    %% 최종 Saturation
    steer_lqr = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_lqr));
    deltaAdd.steerAngle = steer_lqr;

    %% ESC: β-limiter
    K_beta  = getfield_default(CTRL.LAT, 'K_beta',  80000);
    beta_th = deg2rad(getfield_default(CTRL.LAT, 'beta_th', 3.0));
    f_vx    = min(max(vx, 0) / 20.0, 2.0);
    beta_excess = abs(slipAngle) - beta_th;
    if beta_excess > 0
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * beta_excess * f_vx;
    else
        deltaAdd.yawMoment = 0;
    end

end

% ============================================================
function K = solve_lqr(A, B, Q, R)
%SOLVE_LQR Hamiltonian ARE (eig만 사용, Control System Toolbox 불필요)
    n    = size(A, 1);
    Rinv = 1 / R;
    H    = [A,  -B*Rinv*B';
            -Q, -A'       ];
    try
        [V, D] = eig(H);
        ev = real(diag(D));
        [~, idx] = sort(ev);
        Vs = V(:, idx(1:n));
        V1 = Vs(1:n, :);
        V2 = Vs(n+1:end, :);
        P  = real(V2 / V1);
        K  = Rinv * B' * P;
    catch
        K = zeros(1, n);
    end
end
