function animation = drone_Animation(x, y, z, roll, pitch, yaw)
    % This Animation code is for QuadCopter. Written by Jitendra Singh 
    %% Define design parameters
    D2R = pi/180;
    %R2D = 180/pi;
    b = 1;              % the length of total square cover by whole body of quadcopter in meter
    a = b/3;            % the legth of small square base of quadcopter (b/3)
    H = b/3;            % hight of drone in Z direction (b/3)
    H_m = H/2;          % hight of motor in z direction (H_m = H/2)
    r_p = b/5;          % radius of propeller
    %% Conversions
    ro = 45*D2R;                        % angle by which rotate the base of quadcopter
    Ri = [cos(ro), -sin(ro), 0;         % rotation matrix to rotate the coordinates of base
          sin(ro),  cos(ro), 0;
                0,        0, 1];         
    base_co = [-a/2,  a/2, a/2, -a/2;   % Coordinates of Base 
               -a/2, -a/2, a/2,  a/2;
                  0,    0,   0,    0];
    base = Ri*base_co;                  % rotate base Coordinates by 45 degree 
    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1, length(to));
    %% Define Figure plot
    %fig1 = figure('pos', [0 50 800 600]);
    figure(1);
    grid on;
    xlabel('X (m)', 'FontSize', 16);
    ylabel('Y (m)', 'FontSize', 16);
    zlabel('Z (m)', 'FontSize', 16);
    hg = gca;
    view(-128, 32);
    xlim([-9, 11]); ylim([-18, 2]); zlim([0, 25]);
    %axis equal;
    hold(gca, 'on');
    %% Design Different parts
    % design the base square
    drone(1) = patch(base(1, :), base(2, :), base(3, :), 'c');
    drone(2) = patch(base(1, :), base(2, :), base(3, :) + H, 'c');
    %alpha(drone(1:2), 0.7);
    % design 2 parpendiculer legs of quadcopter 
    [xcylinder, ycylinder, zcylinder] = cylinder([H/5 H/5]);
    drone(3) =  surface(b*zcylinder - b/2, ycylinder, xcylinder + H/2, 'facecolor', 'k');
    drone(4) =  surface(ycylinder, b*zcylinder - b/2, xcylinder + H/2, 'facecolor', 'k'); 
    %alpha(drone(3:4), 0.6);
    % design 4 cylindrical motors 
    drone(5) = surface(xcylinder + b/2, ycylinder, H_m*zcylinder + H/2, 'facecolor', 'k');
    drone(6) = surface(xcylinder - b/2, ycylinder, H_m*zcylinder + H/2, 'facecolor', 'k');
    drone(7) = surface(xcylinder, ycylinder + b/2, H_m*zcylinder + H/2, 'facecolor', 'k');
    drone(8) = surface(xcylinder, ycylinder - b/2, H_m*zcylinder + H/2, 'facecolor', 'k');
    %alpha(drone(5:8), 0.7);
    % design 4 propellers
    drone(9)  = patch(xp + b/2, yp, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);
    drone(10) = patch(xp - b/2, yp, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);
    drone(11) = patch(xp, yp + b/2, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);
    drone(12) = patch(xp, yp - b/2, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);
    %alpha(drone(9:12), 0.3);
    %% create a group object and parent surface
    combinedobject = hgtransform('parent', hg);
    set(drone, 'parent', combinedobject)
    % drawnow
    for i = 1:length(x)
        plot3(x(1:i), y(1:i), z(1:i), 'b', 'LineWidth', 1);
        translation = makehgtform('translate', [x(i), y(i), z(i)]);
        %set(combinedobject, 'matrix', translation);
        rotation1 = makehgtform('xrotate', (pi/180)*(roll(i)));
        rotation2 = makehgtform('yrotate', (pi/180)*(pitch(i)));
        rotation3 = makehgtform('zrotate', yaw(i));
        %scaling = makehgtform('scale', 1 - i/20);
        set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);
        %movieVector(i) = getframe(fig1);
        %delete(b);
        drawnow;
        %drawnow limitrate;
        %pause(0.2);
    end
end