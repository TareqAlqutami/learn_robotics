disp('Initializing figures...');
max_iter = length(out.tout);
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

% Initialize video
myVideo = VideoWriter('quad_video2'); %open video file
myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
open(myVideo)

% first iteration
i = 1;
time = out.state.Xe.Time(i);
pos = out.state.Xe.Data(:,i);
pos_cmd = [out.cmd.X.x_cmd.Data(i), out.cmd.X.y_cmd.Data(i), out.cmd.X.z_cmd.Data(i)];
thrust_mag = out.thrust_mag.Data(i)/4; % scale for visualizaition
thrust_sp = unit(out.thrust_sp.Data(i,:))*thrust_mag;
thrust_dir_x = out.thrust_unit_dir.Data(:,1,i);
thrust_dir_y = out.thrust_unit_dir.Data(:,2,i);
thrust_dir_z = out.thrust_unit_dir.Data(:,3,i);

hold(h_3d, 'on')
% plot origin axes
h_origin = quiver3(h_3d,[0,0,0],[0,0,0],[0,0,0],[0.1,0,0],[0,0.1,0],[0,0,-0.1],'r','LineWidth',2);
% title
h_title = title(h_3d,sprintf('time: %4.2f',time));

% thrust mag and dir
h_th = quiver3(h_3d,pos(1),pos(2),pos(3),thrust_sp(1),thrust_sp(2),thrust_sp(3),'g','LineWidth',2);


% thrust unit dir project to x-y-z
h_th_x = quiver3(h_3d,pos(1),pos(2),pos(3),thrust_dir_x(1),thrust_dir_x(2),thrust_dir_x(3),'b');
h_th_y = quiver3(h_3d,pos(1),pos(2),pos(3),thrust_dir_y(1),thrust_dir_y(2),thrust_dir_y(3),'b');
h_th_z = quiver3(h_3d,pos(1),pos(2),pos(3),thrust_dir_z(1),thrust_dir_z(2),thrust_dir_z(3),'b');

% positon current
h_p   = plot3(h_3d,pos(1),pos(2),pos(3), 'b.','markerSize',15);
% position cmd
h_p_sp = plot3(h_3d,pos_cmd(1),pos_cmd(2),pos_cmd(3), 'rx','markerSize',15);

% acc_cmd vector
% ---
hold(h_3d, 'off')

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
pause(0.01);

% rest of iterations
for i = 900:5:3000
    time = out.state.Xe.Time(i);
    pos = out.state.Xe.Data(:,i);
    pos_cmd = [out.cmd.X.x_cmd.Data(i), out.cmd.X.y_cmd.Data(i), out.cmd.X.z_cmd.Data(i)];
    thrust_mag = out.thrust_mag.Data(i)/3;
    thrust_sp = unit(out.thrust_sp.Data(i,:))*thrust_mag;
    thrust_dir_x = out.thrust_unit_dir.Data(:,1,i);
    thrust_dir_y = out.thrust_unit_dir.Data(:,2,i);
    thrust_dir_z = out.thrust_unit_dir.Data(:,3,i);
    prev_time = time;
    o = -[0.5,0.5,0.5];
    origin_offset_x = [0.8 0 0];
    origin_offset_y = [0 0.8 0];
    origin_offset_z = [0 0 -0.8];
    set(h_origin, ...
        'XData', o,'YData', o, 'ZData', o,...
        'UData', origin_offset_x,'VData', origin_offset_y, 'WData', origin_offset_z);
    set(h_th, ...
        'XData', pos(1),'YData', pos(2), 'ZData', pos(3),...
        'UData', thrust_sp(1),'VData', thrust_sp(2), 'WData', thrust_sp(3));  
    set(h_th_x, ...
        'XData', pos(1),'YData', pos(2), 'ZData', pos(3),...
        'UData', thrust_dir_x(1),'VData', thrust_dir_x(2), 'WData', thrust_dir_x(3));
    set(h_th_y, ...
        'XData', pos(1),'YData', pos(2), 'ZData', pos(3),...
        'UData', thrust_dir_y(1),'VData', thrust_dir_y(2), 'WData', thrust_dir_y(3));
    set(h_th_z, ...
        'XData', pos(1),'YData', pos(2), 'ZData', pos(3),...
        'UData', thrust_dir_z(1),'VData', thrust_dir_z(2), 'WData', thrust_dir_z(3));

    set(h_p, ...
        'XData', pos(1), ...
        'YData', pos(2), ...
        'ZData', pos(3));

    set(h_p_sp, ...
        'XData', pos_cmd(1), ...
        'YData', pos_cmd(2), ...
        'ZData', pos_cmd(3));

    set(h_title, 'String', sprintf('time: %4.2f',time))

    set(h_3d,'XLim',[-1.7,1.7])
    set(h_3d,'YLim',[-1.7,1.7])
    set(h_3d,'ZLim',[-1,3])
    
    drawnow;

    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    dt = time - prev_time; % Update simulation time
%     pause(dt);

end

close(myVideo)