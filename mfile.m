close all;
clearvars;
clc;
%% GLOBAL VARIABLES
global U_global W_;
U_global = [0, 0, 0, 0, 0];
W_ = diag([0, 0, 0]);
%% ODE
tspan = [0 100];
[t, x] = ode45(@quadrotor, tspan, zeros(24, 1));
%% DESIRED TRAJECTORY TO PLOT
n_des = [1 + 8*sin(0.1*t), -8 + 8*cos(0.1*t), 2 + 0.2*t];
psi_des = 0.2 + 0*t;
%% CONTROL SIGNALS PLOT
figure(3);
subplot(4,1,1);
plot(U_global(:, 1), U_global(:, 2));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('U_1 (N)', 'FontSize', 14);
subplot(4,1,2);
plot(U_global(:, 1), U_global(:, 3));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('U_2 (N)', 'FontSize', 14);
subplot(4,1,3);
plot(U_global(:, 1), U_global(:, 4));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('U_3 (N)', 'FontSize', 14);
subplot(4,1,4);
plot(U_global(:, 1), U_global(:, 5));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('U_4 (Nm)', 'FontSize', 14);
%% ERROR x, y, z, psi PLOT
figure(2);
subplot(4,1,1);
plot(t, n_des(:, 1) - x(:, 1));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('e_x (m)', 'FontSize', 14);
subplot(4,1,2);
plot(t, n_des(:, 2) - x(:, 2));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('e_y (m)', 'FontSize', 14);
subplot(4,1,3);
plot(t, n_des(:, 3) - x(:, 3));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('e_z (m)', 'FontSize', 14);
subplot(4,1,4);
plot(t, psi_des - x(:, 6));
grid on;
xlabel('Time (s)', 'FontSize', 14);
ylabel('e_\psi (rad)', 'FontSize', 14);
%% FLY TRAJECTORY PLOT
figure(1);
plot3(n_des(:, 1), n_des(:, 2), n_des(:, 3), 'r--', 'LineWidth', 1.5);
hold on;
% plot3(x(:, 1), x(:, 2), x(:, 3), 'b', 'LineWidth', 1.5);
% grid on;
% xlabel('X (m)', 'FontSize', 16);
% ylabel('Y (m)', 'FontSize', 16);
% zlabel('Z (m)', 'FontSize', 16);
% view(-128, 32);
% legend('Quỹ đạo đặt', 'Quỹ đạo bay');
uav_animation(x(:, 1), x(:, 2), x(:, 3), x(:, 4), x(:, 5), x(:, 6));
%% FUNCTION
function dx = quadrotor(t, x)
    %% PARAMETERS
    % Quadrotor UAV parameters
    m = 0.486;
    g = 9.810;
    Mm = diag([m, m, m]);
    Gg = [0, 0, -m*g]';
    Ir = diag([2.830e-3, 2.830e-3, 0]);
    l = diag([0.250, 0.250, 1]);
    kp = 3.230e-7;
    kd = 2.980e-5;
    I = diag([4.856e-3, 4.856e-3, 8.801e-3]);
    cdn = diag([5.560e-4, 5.560e-4, 6.350e-4]);
    kdn = diag([1e-4, 1e-4, 1e-4]);
    W_to_U = [ kd,  kd, kd,  kd;
                0,  kd,  0, -kd;
              -kd,   0, kd,   0;
               kp, -kp, kp, -kp];
    % Control parameters
    A1 = diag([1, 1, 1]);
    A2 = diag([4, 4, 4]);
    A3 = diag([5, 5, 5]);
    A4 = diag([2, 2, 2]);
    B1 = diag([2, 2, 2]);
    B2 = diag([8, 8, 8]);
    B3 = diag([1, 1, 1]);
    B4 = diag([10, 10, 10]);
    %D1 = diag([2, 2, 2]);
    %D2 = diag([8, 8, 8]);
    %D3 = diag([1, 1, 1]);
    %D4 = diag([10, 10, 10]);
    K1 = diag([0.1, 0.1, 0.1]);
    K2 = diag([1, 1, 1]);
    K3 = diag([3, 3, 3]);
    K4 = diag([5, 5, 5]);
    %% DESIRED TRAJECTORY
    n_des = [1 + 8*sin(0.1*t), -8 + 8*cos(0.1*t), 2 + 0.2*t]';
    dn_des = [0.8*cos(0.1*t), -0.8*sin(0.1*t), 0.2]';
    d2n_des = [-0.08*sin(0.1*t), -0.08*cos(0.1*t), 0]';
    d3n_des = [-0.008*cos(0.1*t), 0.008*sin(0.1*t), 0]';
    psi_des = 0.2;
    %% DISTURBANCES
    % Position disturbances
    n_dis = [0.1*sin(t), 0.1*sin(t), 0.1*cos(t)]';
    % Attitude disturbances
    w_dis = [0.4, 0.4, 0.4*sin(t)]';
    %% NONLINEAR FUNCTIONS
    % Translational function f(n)
    FD = -cdn*x(7:9);   
    % Rotational function f(w)
    w = x(10:12);
    Gf = kdn*(w.^2);
    global W_;
    tM = Ir*W_*[x(11), x(10), 0]';
    %% TRANSLATIONAL BIOINSPIRED
    % Vs1
    E1 = n_des - x(1:3);
    dVs1 = -(A1 + abs(diag(E1)))*x(13:15) + B1*E1;
    % Vs2
    E2 = dn_des + K1*x(13:15) - x(7:9);
    dVs2 = -(A2 + abs(diag(E2)))*x(16:18) + B2*E2;
    %% TRANSLATIONAL CONTROLLER
    Un = d2n_des - (Mm^-1)*FD + K1*dVs1 + E1 + K2*x(16:18);
    theta_atan = (Un(1)*cos(psi_des) + Un(2)*sin(psi_des))/(Un(3) + g);
    theta_des = atan(theta_atan);
    phi_atan = cos(theta_des)*(Un(1)*sin(psi_des) - Un(2)*cos(psi_des))/(Un(3) + g);
    phi_des = atan(phi_atan);
    U1 = m*sqrt(Un(1)^2 + Un(2)^2 + (Un(3) + g)^2);
    F = [0, 0, U1]';
    %% TRANSLATIONAL DYNAMICS
    Jr = R_M(x(4), x(5), x(6));
    d2p = (Mm^-1)*(Jr*F + Gg + FD) + n_dis;
    %% ROTATIONAL BIOINSPIRED
    % Vs3
    w_des = [phi_des, theta_des, psi_des]';
    E3 = w_des - x(4:6);
    dVs3 = -(A3 + abs(diag(E3)))*x(19:21) + B3*E3;    
    % Vs4
    d2Vs1 = -diag(diag(sign(E1))*(dn_des - x(7:9)))*x(13:15) - (A1 + abs(diag(E1)))*dVs1 + B1*(dn_des - x(7:9));
    dUn = d3n_des - (-(Mm^-1)*cdn*d2p) + K1*d2Vs1 + (dn_des - x(7:9)) + K2*dVs2;
    dtheta_des = (1/(1 + theta_atan^2))*((dUn(1)*cos(psi_des) + dUn(2)*sin(psi_des))/(Un(3) + g) - theta_atan*dUn(3)/(Un(3) + g));
    dphi_des = (1/(1 + phi_atan^2))*(cos(theta_des)*(dUn(1)*sin(psi_des) - dUn(2)*cos(psi_des))/(Un(3) + g) - phi_atan*dUn(3)/(Un(3) + g) - sin(theta_des)*dtheta_des*phi_atan/cos(theta_des));
    dpsi_des = 0;
    E4 = [dphi_des, dtheta_des, dpsi_des]' + K3*x(19:21) - x(10:12);
    dVs4 = -(A4 + abs(diag(E4)))*x(22:24) + B4*E4;
    %% ROTATIONAL CONTROLLER
    d2w_des = [0, 0, 0]';
    Uw = d2w_des - (I^-1)*(cross(-w, I*w) + Gf + tM) + K3*dVs3 + E3 + K4*x(22:24);
    U234 = (l^-1)*I*Uw;
    tC = l*U234;
    %% ROTATIONAL DYNAMICS
    d2w = (I^-1)*(cross(-w, I*w) + Gf + tM + tC) + w_dis;
    %% ROTOR SPEED W_ CALCULATE
    U = [U1; U234];
    W = sqrt((W_to_U^-1)*U);
    W_ = diag([W(1)-W(2)+W(3)-W(4), W(1)-W(2)+W(3)-W(4), W(1)-W(2)+W(3)-W(4)]);
    %% t AND U_global TO PLOT
    global U_global;
    U_global = [U_global; [t, U1, U234']];
    %% OUTPUT dx
    dx = [x(7:12); d2p; d2w; dVs1; dVs2; dVs3; dVs4];
end
%% ROTATION MATRIX
function Jr = R_M(phi, theta, psi)
    Jr = [0, 0, cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi);
          0, 0, cos(phi)*sin(psi)*sin(theta) - sin(phi)*cos(psi);
          0, 0, cos(phi)*cos(theta)                             ];
end