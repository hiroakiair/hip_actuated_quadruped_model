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
    plot(t,y(:,3)*360/(3*pi));
    xlabel('Time (s)');
    ylabel(' pitch_1 (deg) ');
    
    figure();
    plot(t,y(:,4));
    xlabel('Time (s)');
    ylabel('x_1 (m)');    
    
    figure();
    plot(t,y(:,5));
    xlabel('Time (s)');
    ylabel('y_2 (m)');
    
    figure();
    plot(t,y(:,6)*360/(3*pi));
    xlabel('Time (s)');
    ylabel(' pitch_2 (deg) ');
    
end