function [terminalTime, terminalState, nextPhaseIndex] = execute_flight(tstart, q_ini)  
    global tfinal
    
    % ode45�@�̃��g���C�񐔁H�H
    refine = 4;
    % ���Ό덷
    relval = 1e-12;
    % ��Ό덷
    absval = 1e-12;
    
    % ode45�Ŕ�������������������
    myEvent = @(t, q) events1(q); % �C�x���g�֐����`�D�[���ɂȂ�ϐ��ƕ������w��D
    myOde = @(t, q) f1(q); % ode�ŉ����������������`�D
    options = odeset('RelTol', relval, 'AbsTol', absval, 'Events', myEvent, 'Refine', refine, 'Stats', 'off'); %ode45�̃I�v�V������ݒ�D
    
    % ode45�Ŕ���������������
    [tout, qout, te, qe, ie] = ode45(myOde, [tstart tfinal], q_ini, options);
    
    % ���ʂ�ۑ�
    [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie);
    nextPhaseIndex = detectNextPhase(ie);
    calc_touchDownPos(nextPhaseIndex);

end

function [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie)
    %���̕ۑ��ϐ�
    global tout_ac qout_ac lout_ac gout_ac 
    global teout_ac qeout_ac ieout_ac phaseout_ac
    global  L_0F L_0H
    global gamma_F_td gamma_H_td
    
    nt = length(tout);

    tout_ac = [tout_ac; tout(2:nt)];
    qout_ac = [qout_ac; qout(2:nt, :)];
    
    lout_ac = [lout_ac; ones(nt - 1, 1) * L_0H, ones(nt - 1, 1) * L_0F];
    gout_ac = [gout_ac; ones(nt - 1, 1) * gamma_H_td, ones(nt - 1, 1) * gamma_F_td];

    teout_ac = [teout_ac; te(1)];
    qeout_ac = [qeout_ac; qe(1, :)];
    ieout_ac = [ieout_ac; ie(1)];

    phaseout_ac = [phaseout_ac; ones(nt - 1, 1) * 1];

    %���̏I��
    terminalTime = tout(end);
    terminalState = qout(end,:);

end

function nextPhaseIndex = detectNextPhase(ie)
    
    % �ǂ̃C�x���g���N���������H
    switch length(ie)
    case 0
        % �ǂ̃C�x���g���������Ă��Ȃ�
        % disp('no event occured @flight')
        nextPhaseIndex = 20;
    case 1
        % �P��̃C�x���g������
        if ie(1) == 1
            % disp('hind leg touch down @flight')
            nextPhaseIndex = 2;
        elseif ie(1) == 2
            % disp('fore leg touch down @flight')
            nextPhaseIndex = 4;
        elseif ie(1) == 3
            disp('fall down @flight')
            nextPhaseIndex = 30;
        else
            disp('unknown error @flight')
            nextPhaseIndex = 30;
        end

    case 2
        % ������2�̃C�x���g������
        if ie(1) == 1 && ie(2) == 2
            % disp('hind & fore leg touch down @flight')
            nextPhaseIndex = 3;
        else
            disp('fall down @flight')
            nextPhaseIndex = 30;
        end

    case 3
        disp('unknown error @flight')
        nextPhaseIndex = 30;
    end

end % detectNextPhase

function calc_touchDownPos(nextPhaseIndex)
    global qout_ac lout_ac gout_ac 
    global l_b
    global xH_toe xF_toe

    if nextPhaseIndex == 2
        % ����hind leg stance
        xH_toe = qout_ac(end, 1) - l_b * cos(qout_ac(end, 3)) + lout_ac(end, 1) * sin(gout_ac(end, 1));
    elseif  nextPhaseIndex == 3
        % ����Double leg stance
        xH_toe = qout_ac(end, 1) - l_b * cos(qout_ac(end, 3)) + lout_ac(end, 1) * sin(gout_ac(end, 1));
        xF_toe = qout_ac(end, 1) + l_b * cos(qout_ac(end, 3)) + lout_ac(end, 2) * sin(gout_ac(end, 2));
    elseif nextPhaseIndex == 4
        % ����Fore leg stance
        xF_toe = qout_ac(end, 1) + l_b * cos(qout_ac(end, 3)) + lout_ac(end, 2) * sin(gout_ac(end, 2));
    end
end