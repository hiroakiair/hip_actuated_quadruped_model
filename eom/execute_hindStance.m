function [terminalTime, terminalState, nextPhaseIndex, liftOffFlag] = execute_hindStance(tstart, q_ini, liftOffFlag)
    global tfinal
    
    % ode45�@�̃��g���C�񐔁H�H
    refine = 4;
    % ���Ό덷
    relval = 1e-12;
    % ��Ό덷
    absval = 1e-12;
    
    % ode45�Ŕ�������������������
    myEvent = @(t, q) events2(q); % �C�x���g�֐����`�D�[���ɂȂ�ϐ��ƕ������w��D
    myOde = @(t, q) f2(q); % ode�ŉ����������������`�D
    options = odeset('RelTol', relval, 'AbsTol', absval, 'Events', myEvent, 'Refine', refine, 'Stats', 'off'); %ode45�̃I�v�V������ݒ�D
    
    % ode45�Ŕ���������������
    [tout, qout, te, qe, ie] = ode45(myOde, [tstart tfinal], q_ini, options);
    
    % ���ʂ�ۑ�
    [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie);
    nextPhaseIndex = detectNextPhase(ie);
    calc_touchDownPos(nextPhaseIndex);
    liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag);

end % execute_hindStance

function [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie)
    %���̕ۑ��ϐ�
    global tout_ac qout_ac lout_ac gout_ac 
    global teout_ac qeout_ac ieout_ac phaseout_ac
    global L_0F
    global gamma_F_td
    global xH_toe
    
    global l_b % body
    
    nt = length(tout);

    tout_ac = [tout_ac; tout(2:nt)];
    qout_ac = [qout_ac; qout(2:nt, :)];
    
    theta = qout(2:nt, 3);
    xb = qout(2:nt, 1) - l_b * cos(theta);
    yb = qout(2:nt, 2) - l_b * sin(theta);
    Pb = xH_toe * ones(nt - 1, 1) - xb;
    Qb = 0 - yb;
    LBb = sqrt(Pb.^2 + Qb.^2);
    GBb = atan2(Pb, -Qb);
    
    lout_ac = [lout_ac; LBb, ones(nt - 1, 1) * L_0F];
    gout_ac = [gout_ac; GBb, ones(nt - 1, 1) * gamma_F_td];

    teout_ac = [teout_ac; te(1)];
    qeout_ac = [qeout_ac; qe(1, :)];
    ieout_ac = [ieout_ac; ie(1)];

    phaseout_ac = [phaseout_ac; ones(nt - 1, 1) * 2];

    %���̏I��
    terminalTime = tout(end);
    terminalState = qout(end,:);

end

function  nextPhaseIndex = detectNextPhase(ie)
    % �ǂ̃C�x���g���N���������H
    switch length(ie)
    case 0
        % disp('no event occured @hindStance')
        nextPhaseIndex = 20;
    case 1

        if ie(1) == 1
            % disp('fore leg touch down @hindStance')
            nextPhaseIndex = 3;
        elseif ie(1) == 2
            % disp('hind leg lift off @hindStance')
            nextPhaseIndex = 1;
        elseif ie(1) == 3
            % disp('fall down @hindStance')
            nextPhaseIndex = 30;
        else
            % disp('unknown error @hindStance')
            nextPhaseIndex = 30;
        end

    case 2

        if ie(1) == 1 && ie(2) == 2
            % disp('fore leg touch down & hind leg lift off@hindStance')
            nextPhaseIndex = 4;
        else
            % disp('fall down @hindStance')
            nextPhaseIndex = 30;
        end

    case 3
        % disp('unknown error@hindStance')
        nextPhaseIndex = 30;
    end
end %detectNextPhase

function calc_touchDownPos(nextPhaseIndex)
    global qout_ac lout_ac gout_ac 
    global l_b
    global xF_toe

    if  nextPhaseIndex == 3
        % ����Double leg stance
        xF_toe = qout_ac(end, 1) + l_b * cos(qout_ac(end, 3)) + lout_ac(end, 2) * sin(gout_ac(end, 2));
    elseif nextPhaseIndex == 4
        % ����Fore leg stance
        xF_toe = qout_ac(end, 1) + l_b * cos(qout_ac(end, 3)) + lout_ac(end, 2) * sin(gout_ac(end, 2));
    end
end

function liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag)
    if nextPhaseIndex == 1
        % ����Flight
        liftOffFlag.hind = true;
    elseif nextPhaseIndex == 4
        % ����Fore leg stance
        liftOffFlag.hind = true;
    end
end % update_liftOffFlag

