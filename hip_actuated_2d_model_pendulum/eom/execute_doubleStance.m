function [terminalTime, terminalState, nextPhaseIndex, liftOffFlag] = execute_doubleStance(tstart, q_ini, liftOffFlag)
    global tfinal
    
    % ode45　のリトライ回数？？
    refine = 4;
    % 相対誤差
    relval = 1e-12;
    % 絶対誤差
    absval = 1e-12;

    % ode45で微分方程式を解く準備
    myEvent = @(t, q) events3(q); % イベント関数を定義．ゼロになる変数と方向を指定．
    myOde = @(t, q) f3(q); % odeで解く微分方程式を定義．
    options = odeset('RelTol', relval, 'AbsTol', absval, 'Events', myEvent, 'Refine', refine, 'Stats', 'off'); %ode45のオプションを設定．

    % ode45で微分方程式をとく
    [tout, qout, te, qe, ie] = ode45(myOde, [tstart tfinal], q_ini, options);
    
    % 結果を保存
    [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie);
    nextPhaseIndex = detectNextPhase(ie);
    liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag);

end % execute_doubleStance

function [terminalTime, terminalState] = accumulate(tout, qout, te, qe, ie)
    %解の保存変数
    global tout_ac qout_ac lout_ac gout_ac 
    global teout_ac qeout_ac ieout_ac phaseout_ac
    global xH_toe xF_toe
    
    global l_b % body
    
    nt = length(tout);

    tout_ac = [tout_ac; tout(2:nt)];
    qout_ac = [qout_ac; qout(2:nt, :)];

    theta = qout(2:nt, 3);
    xb = qout(2:nt, 1) - l_b * cos(theta);
    yb = qout(2:nt, 2) - l_b * sin(theta);
    xf = qout(2:nt, 1) + l_b * cos(theta);
    yf = qout(2:nt, 2) + l_b * sin(theta);

    Pb = xH_toe * ones(nt - 1, 1) - xb;
    Qb = 0 - yb;
    LBb = sqrt(Pb.^2 + Qb.^2);
    GBb = atan2(Pb, -Qb);
    Pf = xF_toe * ones(nt - 1, 1) - xf;
    Qf = 0 - yf;
    LBf = sqrt(Pf.^2 + Qf.^2);
    GBf = atan2(Pf, -Qf);

    lout_ac = [lout_ac; LBb, LBf];
    gout_ac = [gout_ac; GBb, GBf];

    teout_ac = [teout_ac; te(1)];
    qeout_ac = [qeout_ac; qe(1, :)];
    ieout_ac = [ieout_ac; ie(1)];

    phaseout_ac = [phaseout_ac; ones(nt - 1, 1) * 4];

    terminalTime = tout(end);
    terminalState = qout(end,:);
end % accumulate

function  nextPhaseIndex = detectNextPhase(ie)
    % どのイベントが起こったか？
    switch length(ie)
    case 0
        % disp('no event occured @phase3')
        nextPhaseIndex = 20;
    case 1

        if ie(1) == 1
            % disp('hind leg lift off @phase3')
            nextPhaseIndex = 4;
        elseif ie(1) == 2
            % disp('fore leg lift off @phase3')
            nextPhaseIndex = 2;
        elseif ie(1) == 3
            disp('fall down @phase3 1')
            nextPhaseIndex = 30;
        else
            disp('unknown error @phase3')
            nextPhaseIndex = 30;
        end

    case 2

        if ie(1) == 1 && ie(2) == 2
            % disp('fore & hind leg lift off @phase3')
            nextPhaseIndex = 1;
        else
            disp('fall down @phase3 2')
            nextPhaseIndex = 30;
        end

    case 3
        disp('unkown error@phase3')
        nextPhaseIndex = 30;
    end 
end % detectNextPhase

function liftOffFlag = update_liftOffFlag(nextPhaseIndex, liftOffFlag)
    if nextPhaseIndex == 1
        % 次はFlight
        liftOffFlag.hind = true;
        liftOffFlag.fore = true;
    elseif nextPhaseIndex == 2
        % 次はHind leg stance
        liftOffFlag.fore = true;
    elseif nextPhaseIndex == 4
        % 次はFore leg stance
        liftOffFlag.hind = true;
    end
end % update_liftOffFlag