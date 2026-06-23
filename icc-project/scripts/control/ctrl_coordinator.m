function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation
%
%   (1) AFS 조향 pass-through
%   (2) ESC yaw moment → 차동 brake (좌/우 비대칭)
%   (3) ABS 감압 (lonCmd.absBrake) → per-wheel brake에 합산 (음수)
%   (4) CDC 댐핑 pass-through
%
%   주의: runner에서 brake_total = brk_scenario + brakeTorque(이 함수 출력)
%         → ABS 감압은 음의 brakeTorque로 출력 → 시나리오 brake를 덜어냄

    ratio_f  = 0.6;
    t_f_half = VEH.track_f / 2;
    t_r_half = VEH.track_r / 2;
    Mz       = latCmd.yawMoment;

    %% 1. AFS 조향각 pass-through
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                                  min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% 2. ESC: yaw moment → 차동 brake
    if abs(Mz) > 10
        dT_f = abs(Mz) * ratio_f     * VEH.rw / t_f_half;
        dT_r = abs(Mz) * (1-ratio_f) * VEH.rw / t_r_half;
        if Mz > 0
            brakeESC = [dT_f; 0; dT_r; 0];   % FL, RL (CCW)
        else
            brakeESC = [0; dT_f; 0; dT_r];   % FR, RR (CW)
        end
    else
        brakeESC = zeros(4, 1);
    end

    %% 3. ABS 감압 (lonCmd.absBrake, 음수)
    if isfield(lonCmd, 'absBrake') && numel(lonCmd.absBrake) == 4
        absBrake = lonCmd.absBrake(:);
    else
        absBrake = zeros(4, 1);
    end

    %% 4. 합산 + Clipping
    %   brakeESC(양수, ESC) + absBrake(음수, ABS 감압)
    %   하한은 -MAX_BRAKE_TRQ (감압이 시나리오 brake를 초과 상쇄 가능)
    brakeTot = brakeESC + absBrake;
    actuatorCmd.brakeTorque = max(-LIM.MAX_BRAKE_TRQ, ...
                                   min(LIM.MAX_BRAKE_TRQ, brakeTot));

    %% 5. CDC 댐핑 pass-through
    actuatorCmd.dampingCoeff = verCmd;

end
