function N = F(r, dr, L, dL, k_leg, b_leg)

    if r(3) < L
        N = max([0, -k_leg*(r(3) - L) -b_leg*(dr(3) - dL)]);
    else
        N = 0;
    end
        
end