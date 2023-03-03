function [value, isterminal, direction] = events2(y)
    %微分方程式を解く設定
    global l_b % body
    global gamma_F_td L_0F L_0H %leg
    global k_H b_H L_0H k_F b_F L_0F 
    global xH_toe xF_toe
    global tau_nF tau_nH
    
    % Hind leg stanceの終端イベント
    xg = y(1);
    yg = y(2);
    theta = y(3);
    
    fore_toeHight = yg + l_b .* sin(theta) - L_0F * cos(gamma_F_td);
    
    HipB = [xg - l_b .* cos(theta); yg - l_b .* sin(theta)];
    lh_x = xH_toe - HipB(1);
    lh_y = HipB(2);
    lh_length = sqrt(lh_x^2 + lh_y^2) - L_0H;
    
%     hip actuateは力で判定
    %力に関してはラグランジアン風に計算
    q_pre = y(1:4);
    dq_pre = y(5:8);
    
    xb = q_pre(1) - l_b*cos(q_pre(3));
    dxb = dq_pre(1) + l_b*dq_pre(3)*sin(q_pre(3));
    yb = q_pre(2) - l_b*sin(q_pre(3));
    dyb = dq_pre(2) - l_b*dq_pre(3)*cos(q_pre(3));
    phi_H = atan2(yb,(xb-xH_toe));
    
    L_H = sqrt((xb-xH_toe)^2+yb^2);
    F_LH = k_H*(L_0H-L_H) - b_H*((xb-xH_toe)*dxb + yb*dyb)/L_H;
    F_TH = tau_nH/L_H;
   
    F_H_verical = F_LH*sin(phi_H)-F_TH*cos(phi_H);

    %判定
%     value = [fore_toeHight; lh_length ;yg];
%     isterminal = [1; 1; 1];
%     direction = [-1; 1; 0];
    
    value = [fore_toeHight; F_H_verical ;yg];
    isterminal = [1; 1; 1];
    direction = [-1; -1; 0];
end