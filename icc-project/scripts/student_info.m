function info = student_info()
%STUDENT_INFO 학생 정보 — 채점 시 매칭에 사용. **반드시 수정해서 제출.**

    info.student_id   = '202525344';
    info.name         = '정지우';
    info.team_members = {};

    info.course = '자동제어특론 - 2026 봄';

    info.ai_usage = 'Claude Code (Anthropic) + Codex used for: LQR controller design (bicycle model, augmented I-action), Q/R tuning, ABS/CDC design, MATLAB debugging';

    %% 검증 (수정 금지)
    if contains(info.student_id, 'TODO_FILL')
        warning('[student_info] 학번이 기입되지 않았습니다 — 채점 시 감점 + 매칭 불가');
    end
    if contains(info.name, 'TODO_FILL')
        warning('[student_info] 이름이 기입되지 않았습니다');
    end
end
