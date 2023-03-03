function [value, isterminal, direction] = events1(y)
    %”÷•ª•û’ö®‚ğ‰ğ‚­İ’è
    global l_b % body
    global gamma_F_td gamma_H_td L_0F L_0H %leg
    

    yg = y(2);
    theta = y(3);

    hind_toeHight = yg - l_b .* sin(theta) - L_0H * cos(gamma_H_td);
    fore_toeHight = yg + l_b .* sin(theta) - L_0F * cos(gamma_F_td);

    value = [hind_toeHight; fore_toeHight; yg];
    isterminal = [1; 1; 1];
    direction = [-1; -1; 0];
end