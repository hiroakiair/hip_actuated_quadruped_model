function N = F_for_3d(r, r_joint, R_1, k_leg, b_leg, alpha, body, L0, roll)

    global alpha_up alpha_dn
    alpha_rem = rem(alpha,2*pi);
    
    if (alpha_dn < alpha_rem) && (alpha_rem < alpha_up)
            L = min(L0, r_joint(3)*cos(roll)/cos(alpha));
            r_joint(3)
            delta_l = L0-L
            f = k_leg*delta_l;
            [-f*cos(alpha); 0; f*sin(alpha)];
            N = R_1.'*[-f*cos(alpha); 0; f*sin(alpha)];
    else 
        N = [0;0;0];
    end
end