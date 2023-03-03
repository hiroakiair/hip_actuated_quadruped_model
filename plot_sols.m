function plot_sols(t,y)
%     close all;
    figure();
    plot(t,y(:,1));
    xlabel('Time (s)');
    ylabel('x_1 (m)');
    
    figure();
    plot(t,y(:,2));
    xlabel('Time (s)');
    ylabel('y_1 (m)');
    
    figure();
    plot(t,y(:,3)*360/(2*pi));
    xlabel('Time (s)');
    ylabel(' \theta_p (deg) ');
    
    figure();
    plot(t,y(:,4)*360/(2*pi)-y(:,3)*360/(2*pi));
    xlabel('Time (s)');
    ylabel(' \theta_s - \theta_p (deg) ');
    
    figure();
    plot(t,y(:,5));
    xlabel('Time (s)');
    ylabel('dx_1 (m)');
    
    figure();
    plot(t,y(:,6));
    xlabel('Time (s)');
    ylabel('dy_1 (m)');
    
    figure();
    plot(t,y(:,7)*360/(2*pi));
    xlabel('Time (s)');
    ylabel(' d\theta_p (deg) ');
    
    figure();
    plot(t,y(:,8)*360/(2*pi)-y(:,7)*360/(2*pi));
    xlabel('Time (s)');
    ylabel(' d\theta_s (deg) ');
    
end