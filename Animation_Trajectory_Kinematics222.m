clc
clear all
close all

% Joint Lengths
A1     = 0.3; % Link 1 Length
A2     = 0.5; % Link 2 Length

%% Trajectory Planning
t0 = 0;
tf = 3;

% Position for the Kinematics
Fin_P  = [0.1; 1; 0]; % Final Desired Position
Ini_P  = [2; 1; 0]; % Initial Position of the Robot

X0 = Ini_P(1);
Y0 = Ini_P(2);
Z0 = Ini_P(3);

Xf = Fin_P(1);
Yf = Fin_P(2);
Zf = Fin_P(3);

% X-Y-Z Axis Initial and Final Velocity
vX0 = 0;
vXf = 0;

vY0 = 0;
vYf = 0;

vZ0 = 0;
vZf = 0;

%% Calculating variables ao, a1, a2, a3

% For X_Axis
MX = [ 1 t0 t0^2 t0^3;
      0 1 2*t0 3*t0^2;
      1 tf tf^2 tf^3;
      0 1 2*tf 3*tf^2];

bX = [X0; vX0; Xf; vXf];

aX = inv(MX) * bX;

a0X = aX(1);
a1X = aX(2);
a2X = aX(3);
a3X = aX(4);

% For Y_Axis
MY = [ 1 t0 t0^2 t0^3;
      0 1 2*t0 3*t0^2;
      1 tf tf^2 tf^3;
      0 1 2*tf 3*tf^2];

bY = [Y0; vY0; Yf; vYf];

aY = inv(MY) * bY;

a0Y = aY(1);
a1Y = aY(2);
a2Y = aY(3);
a3Y = aY(4);

% For Z_Axis
MZ = [ 1 t0 t0^2 t0^3;
      0 1 2*t0 3*t0^2;
      1 tf tf^2 tf^3;
      0 1 2*tf 3*tf^2];

bZ = [Z0; vZ0; Zf; vZf];

aZ = inv(MZ) * bZ;

a0Z = aZ(1);
a1Z = aZ(2);
a2Z = aZ(3);
a3Z = aZ(4);

% Time for Trajectory

dt = 0.005; % Sampling Time
N = (tf - t0)/dt; % Total numbers of points in Trajectory
Time = linspace(t0, tf, N);

% X Axis
X_T = a0X + a1X * Time + a2X * Time.^2 + a3X * Time.^3; 
vXd = a1X + 2 * a2X * Time + 3 * a3X * Time.^2;

% Y Axis
Y_T = a0Y + a1Y * Time + a2Y * Time.^2 + a3Y * Time.^3;
vYd = a1Y + 2 * a2Y * Time + 3 * a3Y * Time.^2;

% Z Axis
Z_T = a0Z + a1Z * Time + a2Z * Time.^2 + a3Z * Time.^3;
vZd = a1Z + 2 * a2Z * Time + 3 * a3Z * Time.^2;

%% Inverse Kinematics

it = 100; % Iterations for inverse kinematics
Cur_P  = [1.8; 0; 0]; % Initial position when starting the inverse kinematics
JointV = [0; 0; 1.5708; 1]; % Initial Joint Values for the Inverse Kinematics

for k = 1 : N

    Des_P  = [X_T(k); Y_T(k); Z_T(k)];

    Err_P  = (Des_P - Cur_P) / it; % Error factor inverse kinematics

    % Inverse Kinematics Main Loop for Joint 2 ~ 4
    for i = 1 : it

        J = IK_Jacobian(JointV);

        JointV = JointV + pinv(J) * Err_P;

    end

    Des_JV([1 2 3 4], k) = JointV; % Saving all the joint values

    Des_JV(1, k) = Z_T(k); % Joint 1 displacement is directly equal to the Z trajectory.

    % Forward Kinematics

    T01 = [1 0 0 A1; 0 1 0 0; 0 0 1 Des_JV(1); 0 0 0 1];
    T12 = [cos(Des_JV(2, k)) -sin(Des_JV(2, k)) 0 A2*cos(Des_JV(2, k)); sin(Des_JV(2, k)) cos(Des_JV(2, k)) 0 A2*sin(Des_JV(2, k)); 0 0 1 0; 0 0 0 1];
    T23 = [cos(Des_JV(3, k)) 0 sin(Des_JV(3, k)) 0; sin(Des_JV(3, k)) 0 -cos(Des_JV(3, k)) 0; 0 1 0 0; 0 0 0 1];
    T34 = [1 0 0 0; 0 1 0 0; 0 0 1 Des_JV(4, k); 0 0 0 1];

    % Forward Kinematics
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    Cur_P = T04([1 2 3],4); % Updating the Current Position

end

%% Plotting The Robot

f1 = figure(1);
f1.Position = [0 0 3000 3000];
i = 1;

    frame1(i) = getframe(gcf); % Saving frames for making video

    i = i + 1;

    axis off

for k = 1 : 5 : N

    frame1(i) = getframe(gcf); % Saving frames for video
    i = i + 1;

    axis off

    %% Forward Kinematics

    T01 = [1 0 0 A1; 0 1 0 0; 0 0 1 Des_JV(1, k); 0 0 0 1];
    T12 = [cos(Des_JV(2, k)) -sin(Des_JV(2, k)) 0 A2*cos(Des_JV(2, k)); sin(Des_JV(2, k)) cos(Des_JV(2, k)) 0 A2*sin(Des_JV(2, k)); 0 0 1 0; 0 0 0 1];
    T23 = [cos(Des_JV(3, k)) 0 sin(Des_JV(3, k)) 0; sin(Des_JV(3, k)) 0 -cos(Des_JV(3, k)) 0; 0 1 0 0; 0 0 0 1];
    T34 = [1 0 0 0; 0 1 0 0; 0 0 1 Des_JV(4, k); 0 0 0 1];

    % Forward Kinematics
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    X1 = T01(1,4); Y1 = T01(2,4); Z1 = T01(3,4);
    X2 = T02(1,4); Y2 = T02(2,4); Z2 = T02(3,4);
    X3 = T03(1,4); Y3 = T03(2,4); Z3 = T03(3,4);
    X4 = T04(1,4); Y4 = T04(2,4); Z4 = T04(3,4);

    %% Isometric View

    subplot(1,2,1)
    hold off

    plot3(Z1, Y1, 0, 'r.','MarkerSize',30)

    W = fill3([-2 -2 2 2], [1 1 1 1], [0 2 2 0],'b'); % Wall
    set(W,'FaceAlpha',0.1);

    hold on


    % Now plotting the first solid Link 1 in blue color
    plot3([Z1 Z1], [Y1 Y1], [0 X1], 'b','LineWidth',5)
    plot3([Z1 Z2], [Y1 Y2], [X1 X2], 'b','LineWidth',5)
    plot3([Z3 Z4], [Y3 Y4], [X3 X4], 'g','LineWidth',5)
    plot3(Z2, Y2, X2, 'r.','MarkerSize',50)
    plot3(Z1, Y1, X1, 'r.','MarkerSize',50)
    % Now plotting black dot at end-effector
    plot3(Z4, Y4, X4, 'k.','MarkerSize',50)

    % Figure Axis Limitation
    xlim([-2 2])
    ylim([-2 2])
    zlim([-0.1 2])

    % Labels
    t=title('Robot Animation - Isomteric View');
    t.FontSize=28;
    xlabel('Z Axis'); % The actual x-axis of figure is the z-axis of our chosen coordinate
    ylabel('Y Axis');
    zlabel('X Axis'); % The actual z-axis of figure is the x-axis 
    axis('square')

    %% Front View
    subplot(1,2,2)
    hold off
    
    plot([1 1],[0 2.2],'m',LineWidth=3); % Line representing the wall
    

    hold on

    plot(Y1, X1, 'r.','MarkerSize',40)
    plot(Y2, X2, 'r.','MarkerSize',40)
    plot([Y1 Y1], [0 X1], 'b','LineWidth',4)
    plot([Y1 Y2], [X1 X2], 'b','LineWidth',4)
    plot([Y3 Y4], [X3 X4], 'g','LineWidth',4)
    plot(Y4, X4, 'k.','MarkerSize',50)

    xlim([-2 2])
    ylim([0 2.5])
    zlim([-0.1 2])

    % Labels
    t2=title('Robot Animation - Side View');
    t2.FontSize=28;

    txt= '\leftarrow Wall';
    t3 = text(1,2,txt);
    t3.FontSize=18;
    xlabel('Y Axis'); % The actual x-axis of figure is the y-axis of robot
    ylabel('X Axis');

    axis('square')

    pause(0.01);

end

%% Saving the graphs as Video
Video1 = VideoWriter('Robot_Ani11111');
Video1.FrameRate = 10;
open(Video1);

for i=1:length(frame1)
    writeVideo(Video1, frame1(i));

end

close(Video1);

%% Plotting trajectory data
% Velocity Plot
figure(2)
subplot(3,1,1)
plot(Time, vXd)
xlabel('Time - sec')
ylabel('X Velocity m/sec')
title('Desired End-Effector Velocity')

subplot(3,1,2)
plot(Time, vYd)
xlabel('Time - Seconds')
ylabel('Y Velocity meter/sec')
title('Desired Y Velocity')

subplot(3,1,3)
plot(Time, vZd)
xlabel('Time - Seconds')
ylabel('Z Velocity meter/sec')
title('Desired Z Velocity')

% End-Effector Position Plot
figure(3)
subplot(3,1,1)
plot(Time, X_T)
xlabel('Time - Seconds')
ylabel('X Position - meter')
title('Desired End-Effector Position ')

subplot(3,1,2)
plot(Time, Y_T)
xlabel('Time - Seconds')
ylabel('Y Position meter')
title('Desired Y Position')

subplot(3,1,3)
plot(Time, Z_T)
xlabel('Time - Seconds')
ylabel('Z Position meter')
title('Desired Z Position')

% Joint Angles Plot 
figure(4)
subplot(2,2,1)
plot(Time, Des_JV(1,:))
xlabel('Time - Seconds')
ylabel('Prismatic Joint - D1 meter')
title('Desired Joint 1 Position')

subplot(2,2,2)
plot(Time, Des_JV(2,:))
xlabel('Time - Seconds')
ylabel('Joint 2 - Theta Rad')
title('Desired Joint 2 Angles')

subplot(2,2,3)
plot(Time, Des_JV(3,:))
xlabel('Time - Seconds')
ylabel('Joint 3 - Theta Rad')
title('Desired Joint 3 Angles')

subplot(2,2,4)
plot(Time, Des_JV(4,:))
xlabel('Time - Seconds')
ylabel('Prismatic Joint - D4 meter')
title('Desired Joint 4 Position')
