function [terminalTime, terminalState, nextPhaseIndex, liftOffFlag] = execute_foreStance(tstart, q_ini, liftOffFlag)
    global tfinal
    
    % ode45　のリトライ回数？？
    refine = 4;
    % 相対誤差
    relval = 1e-12;
    % 絶対誤差
    absval = 1e-12;
    
    % ode45で微分方程式を解く準備
    myEvent = @(t, q) events4(q); % イベント関数を定義．ゼロになる変数と方向を指定．
    myOde = @(t, q) f4(q); % odeで解く微分方程式を定義．
    options = odeset('RelTol', relval, 'AbsTol', absval, 'Events', myEvent, 'Refine', refine, 'Stats', 'off'); %ode45のオプションを設定．

    % ode45で微分方程式を解く
    [tout, qout, te, qe, ie] = ode45(myOde, [tstart tfinal], q_ini, options);
    
    % 結果を保存
    [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie);
    nextPhaseIndex = detectNextPhase(ie);
    calc_touchDownPos(nextPhaseIndex);
    liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag);

end % execute_foreStance

function [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie)
    %解の保存変数
    global tout_ac qout_ac lout_ac gout_ac 
    global teout_ac qeout_ac ieout_ac phaseout_ac
    global L_0H
    global gamma_H_td
    global xF_toe
    
    global l_b % body
    
    nt = length(tout);

    tout_ac = [tout_ac; tout(2:nt)];
    qout_ac = [qout_ac; qout(2:nt, :)];
    
    theta = qout(2:nt, 3);
    xf = qout(2:nt, 1) + l_b * cos(theta);
    yf = qout(2:nt, 2) + l_b * sin(theta);
    
    Pf = xF_toe * ones(nt - 1, 1) - xf;
    Qf = 0 - yf;
    LBf = sqrt(Pf.^2 + Qf.^2);
    GBf = atan2(Pf, -Qf);
    
    lout_ac = [lout_ac; ones(nt - 1, 1) * L_0H, LBf];
    gout_ac = [gout_ac; ones(nt - 1, 1) * gamma_H_td, GBf];

    teout_ac = [teout_ac; te(1)];
    qeout_ac = [qeout_ac; qe(1, :)];
    ieout_ac = [ieout_ac; ie(1)];

    phaseout_ac = [phaseout_ac; ones(nt - 1, 1) * 3];

    %解の終了
    terminalTime = tout(end);
    terminalState = qout(end,:);

end

function  nextPhaseIndex = detectNextPhase(ie)
    % どのイベントが起こったか？
    switch length(ie)
    case 0
%         disp('no event occured @phase4')
        nextPhaseIndex = 20;
    case 1
        if ie(1) == 1
            % disp('hind leg touch down @phase4')
            nextPhaseIndex = 3;
        elseif ie(1) == 2
            % disp('fore leg lift off @phase4')
            nextPhaseIndex = 1;
        elseif ie(1) == 3
            disp('fall down @phase4 1')
            nextPhaseIndex = 30;
        else
            disp('unknown error @phase4 1')
            nextPhaseIndex = 30;
        end

    case 2
        if ie(1) == 1 && ie(2) == 2
            % disp('hind leg touch down & fore leg lift off @phase4')
            nextPhaseIndex = 2;
        else
            disp('fall down @phase4 2')
            nextPhaseIndex = 30;
        end

    case 3
        disp('unknown error @phase4 2')
        nextPhaseIndex = 30;
    end
end % detectNextPhase


function calc_touchDownPos(nextPhaseIndex)
    global qout_ac lout_ac gout_ac 
    global l_b
    global xH_toe

    if nextPhaseIndex == 2
        % 次はhind leg stance
        xH_toe = qout_ac(end, 1) - l_b * cos(qout_ac(end, 3)) + lout_ac(end, 1) * sin(gout_ac(end, 1));
    elseif nextPhaseIndex == 3
        % 次はDouble leg stance
        xH_toe = qout_ac(end, 1) - l_b * cos(qout_ac(end, 3)) + lout_ac(end, 1) * sin(gout_ac(end, 1));
    end
end % calc_touchDownPos

function liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag)
    if nextPhaseIndex == 1
        % fore lift off
        liftOffFlag.fore = true;
    elseif nextPhaseIndex == 2
        % fore lift off
        liftOffFlag.fore = true;
    end
end % update_liftOffFlag