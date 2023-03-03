function dqdt = f1(q)

    global M J_1 l_b % body
    global m J_2 l_ab % viscera
    global k_ab b_ab  % stiffness of abdominal oblique muscle
    global g
    
    global L_0H L_0F gamma_H_td gamma_F_td
    
    
    param = [M J_1 l_b m J_2 l_ab k_ab b_ab g];
    
    q_pre = q(1:4);
    dq_pre = q(5:8);
    
    %一般化力
    Q = [0;0;0;0];
    
    %慣性行列
    MassMatrix = myMassMatrix(q_pre, param);
    %コリオリ・重力
    f_cg = myCoriGrav(q_pre, dq_pre, param);
    
    
    %加速度
    ddq = MassMatrix\(-f_cg+Q);
    
    dqdt = [dq_pre; ddq];
    
end