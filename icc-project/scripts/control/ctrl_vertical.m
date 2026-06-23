function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (Continuous Damping Control) — hybrid skyhook+groundhook
%
%   설계 (semi-active, 빈도 분리 §3.3):
%   - skyhook  : F_sky = -c_sky * v_s  (sprung 속도 억제 → body bounce 1-2Hz)
%   - groundhook: F_gnd = -c_gnd * v_u  (unsprung 속도 억제 → wheel hop 10-15Hz)
%   - hybrid   : F_des = alpha*F_sky + (1-alpha)*F_gnd
%   - plant 댐퍼 F_d = -c*v_rel 매칭 → c = (alpha*c_sky*v_s + (1-alpha)*c_gnd*v_u)/v_rel
%   - 수동소자 제약(c>=0) 만족 시만 적용, 아니면 c_min
%   - cMin <= c <= cMax 제한

    %% 파라미터
    c_min  = getfield_default(CTRL.VER, 'cMin',    800);
    c_max  = getfield_default(CTRL.VER, 'cMax',    6000);
    c_sky  = getfield_default(CTRL.VER, 'skyGain', 3500);
    c_gnd  = getfield_default(CTRL.VER, 'gndGain', 1500);   % groundhook 게인
    alpha  = getfield_default(CTRL.VER, 'hybridAlpha', 0.7);% skyhook 비중 (1=순수skyhook)

    %% suspState 안전 추출
    if isfield(suspState, 'zs_dot') && numel(suspState.zs_dot) == 4
        v_s = suspState.zs_dot(:);
    else
        v_s = zeros(4,1);
    end
    if isfield(suspState, 'zu_dot') && numel(suspState.zu_dot) == 4
        v_u = suspState.zu_dot(:);
    else
        v_u = zeros(4,1);
    end

    v_rel = v_s - v_u;   % 댐퍼 상대속도

    %% Semi-active hybrid skyhook + groundhook (per-wheel)
    %   F_des = -(alpha*c_sky*v_s + (1-alpha)*c_gnd*v_u)
    %   plant 댐퍼 F_d = -c*v_rel 매칭 → c = (alpha*c_sky*v_s + (1-alpha)*c_gnd*v_u)/v_rel
    %   수동제약: c>=0 가능할 때만 적용 (force·v_rel 부호 일치)
    c_cmd = c_min * ones(4,1);
    for i = 1:4
        num = alpha * c_sky * v_s(i) + (1-alpha) * c_gnd * v_u(i);  % 원하는 댐퍼력 크기항
        if abs(v_rel(i)) > 1e-3
            c_try = num / v_rel(i);
            if c_try > 0                  % 수동댐퍼로 구현 가능한 방향
                c_cmd(i) = c_try;
            else
                c_cmd(i) = c_min;         % 에너지 주입 불가 → 최소 감쇠
            end
        else
            c_cmd(i) = c_min;             % 상대속도≈0 특이점 회피
        end
    end

    %% 제한
    dampingCmd = min(max(c_cmd, c_min), c_max);

end
