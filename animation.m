function animation(t,y,l,g)
    global l_b % body
    global l_ab % viscera
    global l_0F l_0H  % stiffness of abdominal oblique muscle
 

    global L_0F L_0H
    
    %座標
    xb = y(:,1) - l_b.*cos(y(:,3));
    yb = y(:,2) - l_b.*sin(y(:,3));
    
    xf = y(:,1) + l_b.*cos(y(:,3));
    yf = y(:,2) + l_b.*sin(y(:,3));
    
    xab = y(:,1) - l_ab.*cos(y(:,4));
    yab = y(:,2) - l_ab.*sin(y(:,4));
    
    xb_foot = xb + l(:,1).*sin(g(:,1));
    yb_foot = yb - l(:,1).*cos(g(:,1));
    xf_foot = xf + l(:,2).*sin(g(:,2));
    yf_foot = yf - l(:,2).*cos(g(:,2));
    

    
    %セットアップ
    figure; hold on
    title(sprintf('Trajectory\nTime: %0.2f sec', t(1)), 'Interpreter', 'Latex');
    xlabel('x', 'Interpreter', 'Latex')
    ylabel('y', 'Interpreter', 'Latex')
    grid minor  % Adding grid lines
    axis equal  % Equal axis aspect ratio
%     view(0,0);  % Setting viewing angle
    
    %プロット
    % Create file name variable
%     filename = append('animation_k',num2str(k),'.gif')
     filename = append('animation_k','.gif')
    % Plotting with no color to set axis limits
    
    x_body = [xf, xb];
    y_body = [yf, yb];
    
    x_viscera = [y(:,1), xab];
    y_viscera = [y(:,2), yab];
     
    x_F = [xf, xf_foot];
    y_F = [yf, yf_foot];
    
    x_H = [xb, xb_foot];
    y_H = [yb, yb_foot];
    
    
    %グラフのinitialize
    body = plot(x_body(1,:),y_body(1,:));
    viscera = plot(x_viscera(1,:),y_viscera(1,:));
    Foot_f = plot(x_F(1,:),y_F(1,:));
    Foot_h = plot(x_H(1,:),y_H(1,:));
    ground = yline(0);
%     xlim([-0.5 8.0])
%     ylim([-0.1 0.7])
%     zlim([-0.2 0.8])
    
    t_pre = 0;
    for i = 1:length(t)
        if or(i == 1, t(i) - t_pre > 0.01)
            t_pre = t(i);
            %座標の更新

            %更新
            body.XData = x_body(i,:);
            body.YData = y_body(i,:);
            viscera.XData = x_viscera(i,:);
            viscera.YData = y_viscera(i,:);
            Foot_f.XData = x_F(i,:);
            Foot_f.YData = y_F(i,:);
            Foot_h.XData = x_H(i,:);
            Foot_h.YData = y_H(i,:);

            % Updating the title
            title(sprintf('Trajectory\nTime: %0.2f sec', t(i)),...
            'Interpreter','Latex');

            % Delay
    %         pause(1.0);

            % Saving the figure
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);

            if i == 1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                'DelayTime',0.01);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append',...
                'DelayTime',0.01);
            end
        end
    end
end