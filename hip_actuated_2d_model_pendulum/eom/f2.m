function dqdt = f2(q)
    global M J_1 l_b % body
    global m J_2 l_ab % viscera
    global k_ab b_ab  % stiffness of abdominal oblique muscle
    global g
   
    global k_H b_H L_0H
    global tau_nH 
    global xH_toe
    
    param = [M J_1 l_b m J_2 l_ab k_ab b_ab g];
    
    q_pre = q(1:4);
    dq_pre = q(5:8);
    
    %一般化力
    xb = q_pre(1) - l_b*cos(q_pre(3));
    dxb = dq_pre(1) + l_b*dq_pre(3)*sin(q_pre(3));
    yb = q_pre(2) - l_b*sin(q_pre(3));
    dyb = dq_pre(2) - l_b*dq_pre(3)*cos(q_pre(3));
    phi_H = atan2(yb,(xb-xH_toe));
    
    L_H = sqrt((xb-xH_toe)^2+yb^2);
    
    F_LH = k_H*(L_0H-L_H) - b_H*((xb-xH_toe)*dxb + yb*dyb)/L_H;
    F_TH = tau_nH/L_H;
   
    F_H = [F_LH*cos(phi_H)+F_TH*sin(phi_H);F_LH*sin(phi_H)-F_TH*cos(phi_H)];
    tau_H = cross([-l_b*cos(q_pre(3)); -l_b*sin(q_pre(3));0],[F_H;0]);
    
    Q = [F_H;tau_H(3);0];
    
    %慣性行列
    MassMatrix = myMassMatrix(q_pre, param);
    %コリオリ・重力
    f_cg = myCoriGrav(q_pre, dq_pre, param);
    
    %加速度
    ddq = MassMatrix\(-f_cg+Q);
    
    dqdt = [dq_pre; ddq];