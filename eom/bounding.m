function bounding(q_initial)
    global L_0H

    % 初期値設定
    [tstart_, q_ini_, phaseIndex, liftOffFlag] = initialize_state(q_initial);
    
    for i_phase = 1:50
        switch phaseIndex
            case 1
                disp('flight')
                [tstart_, q_ini_, phaseIndex] = execute_flight(tstart_, q_ini_);
            case 2
                disp('hind stance')
                [tstart_, q_ini_, phaseIndex, liftOffFlag] = execute_hindStance(tstart_, q_ini_, liftOffFlag);
            case 3
                disp('double stance')
                [tstart_, q_ini_, phaseIndex, liftOffFlag] = execute_doubleStance(tstart_, q_ini_, liftOffFlag);
            case 4
                disp('fore stance')
                [tstart_, q_ini_, phaseIndex, liftOffFlag] = execute_foreStance(tstart_, q_ini_, liftOffFlag);
        end
    end
end