function evaluation(t,y)
    global M J_pitch_1 J_roll_1 J_yaw_1 l_bl l_bw % body
    global m J_pitch_2 J_roll_2 J_yaw_2 l_vl l_vw % viscera
    global k b l_0 % stiffness of abdominal oblique muscle
    global g
    global k_Fleg k_Hleg b_Fleg b_Hleg A_F A_H  L0_F L0_H T% leg
    global t_final
    global phi_FR phi_FL phi_HR phi_HL
 
    %床反力
    N_FR = []; N_FL = []; N_HR = []; N_HL = [];
    %仕事
    W_FR = []; W_FL = []; W_HR = []; W_HL = [];
    %
    roll = []; pitch = []; yaw = [];
    
    %脚の運動関数
    omega = 2*pi/T;
    r_FR = [[]]; r_FL=[[0;0;0]]; r_HR=[[0;0;0]]; r_HL=[[0;0;0]]; dr_FR = [[0;0;0]]; dr_FL=[[0;0;0]]; dr_HR=[[0;0;0]]; dr_HL=[[0;0;0]];
    r_vFR = [[0;0;0]]; r_vFL=[[0;0;0]]; r_vHR=[[0;0;0]]; r_vHL=[[0;0;0]]; dr_vFR = [[0;0;0]]; dr_vFL=[[0;0;0]]; dr_vHR=[[0;0;0]]; dr_vHL=[[0;0;0]];

    p_work_FR = 0;
    p_work_FL = 0;
    p_work_HR = 0;
    p_work_HL = 0;
    n_work_FR = 0;
    n_work_FL = 0;
    n_work_HR = 0;
    n_work_HL = 0;
    intergral_interval_positive = find((floor(t_final/T)-1)*T<t & t<floor(t_final/T)*T).';
    

    for i = intergral_interval_positive
        q_pre = y(i,1:10).';
        dq_pre = y(i,11:20).';
        roll(i) = q_pre(2);
        pitch(i) = q_pre(3);
        yaw(i) = q_pre(4);
        
        [r_FR_pre,r_FL_pre,r_HR_pre,r_HL_pre,dr_FR_pre,dr_FL_pre,dr_HR_pre,dr_HL_pre,r_vFR_pre,r_vFL_pre,r_vHR_pre,r_vHL_pre,dr_vFR_pre,dr_vFL_pre,dr_vHR_pre,dr_vHL_pre,R_1,R_2] = calculate_r_dr(q_pre,dq_pre);
        
        r_FR(:,i) = r_FR_pre;
        r_FL(:,i) = r_FL_pre;
        r_HR(:,i) = r_HR_pre;
        r_HL(:,i) = r_HL_pre;
        dr_FR(:,i) = dr_FR_pre;
        dr_FL(:,i) = dr_FL_pre;
        dr_HR(:,i) = dr_HR_pre;
        dr_HL(:,i) = dr_HL_pre;
                
        r_vFR(:,i) = r_vFR_pre;
        r_vFL(:,i) = r_vFL_pre;
        r_vHR(:,i) = r_vHR_pre;
        r_vHL(:,i) = r_vHL_pre;
        dr_vFR(:,i) = dr_vFR_pre;
        dr_vFL(:,i) = dr_vFL_pre;
        dr_vHR(:,i) = dr_vHR_pre;
        dr_vHL(:,i) = dr_vHL_pre;
        
        L_FR = L0_F + A_F*sin(omega*t(i)+phi_FR);
        L_FL = L0_F + A_F*sin(omega*t(i)+phi_FL);
        L_HR = L0_H + A_H*sin(omega*t(i)+phi_HR);
        L_HL = L0_H + A_H*sin(omega*t(i)+phi_HL);
        dL_FR = omega*A_F*cos(omega*t(i)+phi_FR);
        dL_FL = omega*A_F*cos(omega*t(i)+phi_FL);
        dL_HR = omega*A_H*cos(omega*t(i)+phi_HR);
        dL_HL = omega*A_H*cos(omega*t(i)+phi_HL);
        
        %床反力
        N_FR(i) = F(r_FR, dr_FR, L_FR, dL_FR, k_Fleg, b_Fleg);
        N_FL(i) = F(r_FL, dr_FL, L_FL, dL_FL, k_Fleg, b_Fleg);
        N_HR(i) = F(r_HR, dr_HR, L_HR, dL_HR, k_Hleg, b_Hleg);
        N_HL(i) = F(r_HL, dr_HL, L_HL, dL_HL, k_Hleg, b_Hleg);
        
        W_FR(i) = dL_FR*N_FR(i);
        W_FL(i) = dL_FL*N_FL(i);
        W_HR(i) = dL_HR*N_HR(i);
        W_HL(i) = dL_HL*N_HL(i); 
        
        
        %仕事の計算
        if dL_FR > 0
            p_work_FR = p_work_FR + N_FR(i)*dL_FR;
        else
            n_work_FR = n_work_FR - N_FR(i)*dL_FR;
        end
        if dL_FL > 0
            p_work_FL = p_work_FL + N_FL(i)*dL_FL;
        else
            n_work_FL = n_work_FL - N_FL(i)*dL_FL;
        end
        if dL_HR > 0
            p_work_HR = p_work_HR + N_HR(i)*dL_HR;
        else
            n_work_HR = n_work_HR - N_HR(i)*dL_HR;
        end
        if dL_HL > 0
            p_work_HL = p_work_HL + N_HL(i)*dL_HL;
        else
            n_work_HL = n_work_HL - N_HL(i)*dL_HL;
        end
    end

    
    pW = [sum(p_work_FR), sum(p_work_FL), sum(p_work_HR), sum(p_work_HL)];
    nW = [sum(n_work_FR), sum(n_work_FL), sum(n_work_HR), sum(n_work_HL)];
    
    angle = [range(roll)*360/(2*pi), range(pitch)*360/(2*pi), range(yaw)*360/(2*pi)];
    
    
    %raw data
    figure();
    xlabel('Time (s)');
    ylabel(' \roll_body (deg) ');
    plot(t(intergral_interval_positive), roll(intergral_interval_positive)*360/(2*pi));
    ylim([-20 20])
    title([' Roll Angle ']);
    filename = append('fig/roll_',num2str(k),'.png');
    saveas(gcf,filename)
    
    figure();
    xlabel('Time (s)');
    ylabel(' \pitch_body (deg) ');
    plot(t(intergral_interval_positive), pitch(intergral_interval_positive)*360/(2*pi));
    ylim([-8 0])
    title([' Pitch Angle ']);
    filename = append('fig/pitch_',num2str(k),'.png');
    saveas(gcf,filename)
    
    figure();
    xlabel('Time (s)');
    ylabel(' \yaw_body (deg) ');
    plot(t(intergral_interval_positive), yaw(intergral_interval_positive)*360/(2*pi));
    title([' Yaw Angle ']);
    filename = append('fig/yaw_',num2str(k),'.png');
    saveas(gcf,filename)
    
    
    %角度
    figure();
    x = categorical({'Roll', 'Pitch', 'Yaw'});
    x = reordercats(x,{'Roll', 'Pitch', 'Yaw'});
    bar(x,angle) 
    ylim([0 40])
    ylabel(" Angle Range [deg] ");
    filename = append('fig/Angle_Range_',num2str(k),'.png');
    saveas(gcf,filename)
    
    fontsize=15; % 変更したいフォントサイズにする．
    h1=gca; % 直前に作成した図をhとしてset関数で変更できるようにしています．
    set(h1,'fontsize',fontsize)
    
    
    %仕事
    figure();
    x = categorical({'FR', 'FL', 'HR', 'HL'});
    x = reordercats(x,{'FR', 'FL', 'HR', 'HL'});
    bar(x,pW) 
    ylim([0 7.5*1.0e+05])
    ylabel(" Positive Work [W] ");
    
    fontsize=15; % 変更したいフォントサイズにする．
    h1=gca; % 直前に作成した図をhとしてset関数で変更できるようにしています．
    set(h1,'fontsize',fontsize)
    filename = append('fig/Positive_work_',num2str(k),'.png');
    saveas(gcf,filename)

end