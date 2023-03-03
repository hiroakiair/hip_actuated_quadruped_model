function [tstart_, q_ini_, phaseIndex, liftOffFlag] = initialize_state(q_initial)
    tstart_ = 0;
    q_ini_ = q_initial;

    liftOffFlag.hind = false;
    liftOffFlag.fore = false;
    
    % ‰Šú’l‚¨‚©‚µ‚©‚Á‚½‚Æ‚«
    y_ini = q_initial(2);
    phaseIndex = 1;
    if y_ini < 0
        phaseIndex = 22;
    end
end