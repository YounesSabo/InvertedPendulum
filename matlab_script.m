%% define parameters and create tcp/ip object
close all, clc, clear all 
arduino = tcpclient('127.0.0.1', 6013, 'Timeout', 60);  
addpath('matlab_tools')
%% define parameters and modes
T_sample = 0.05;
n_samples = 200;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%% Free respons
clc;close all

reset_system(arduino)
mode = OPEN_LOOP;
angle = 0*pi/180;
Measurements = [];

% We'll make N measurements
N = 5; 

reset_system(arduino)
w = 0;
set_mode_params(arduino, mode, w, [])
% w is the force [N] on the cart
% w has to be chosen in [-10,10]
for i = 1:2
    reset_system(arduino)
    w = 1;
    Measurements = [Measurements; get_response(arduino, w, n_samples)];
end
%-------------------------------------------------------------------------
% Results of the free respons
close all; clc
% Taking the mean of the results
y_theta = sum(Measurements(3:3:end,:))/N;
y_x = sum(Measurements(2:3:end-1,:))/N;
u = sum(Measurements(1:3:end-2,:))/N;

figure; plot( y_x); title("x"); xlabel('t')
figure; plot( y_theta); title("\theta","Interpreter","tex"); xlabel('t')
figure; plot(ts, u); title("u"); xlabel('t')

% We can clearly see that there is noise on the measurements!

%% Controller PID
clc; close all; clear y_theta u y_x

% Data for the measurement
T_sample = 0.05;
n_samples = 300;
ts = (0:n_samples-1)*T_sample;

% Data for the controller in python
Gain_d = 90;
zerosd = [0.8185    0.7788];

mode = CLASSICAL;
Angle = 0;
w = 0;
set_mode_params(arduino, mode, w,[Angle, Gain_d, zerosd, n_samples]);
input('press enter')
W = 6*ones(1,10);
Y = [];
for w = W
    reset_system(arduino)
    Y = [Y; get_response(arduino, w, n_samples)];
end
disp('Measurements are done')
%-------------------------------------------------------------------------
% The results
u = sum(Y(1:3:end-2,:))/length(W);
y_x = sum(Y(2:3:end-1,:))/length(W);
y_theta = sum(Y(3:3:end,:))/length(W); 


figure(1); 
plot(ts, y_theta,'b');
title("Measured \theta",'Interpreter','tex','FontSize',16);
xlabel('Time [s]','Interpreter','tex','FontSize',15); 
ylabel('\theta [rad]','Interpreter','tex','FontSize',15)
legend('Mean \theta ','FontSize',12)

figure(2); plot(ts, y_x); 
title("Measured x",'Interpreter','tex','FontSize',16);
xlabel('Time [s]','Interpreter','tex','FontSize',15);
ylabel('x [m]','Interpreter','tex','FontSize',15) 
legend('Mean x ','FontSize',12)


figure(3); plot(ts, u); 
title("Measured u",'Interpreter','tex','FontSize',16);
xlabel('Time [s]','Interpreter','tex','FontSize',15);
ylabel('x [m]','Interpreter','tex','FontSize',15)
legend('Mean u ','FontSize',12)

%% STATE FEEDBACK
close all; clc;clear Y

mode = STATE_SPACE;
T_sample = 0.05;
n_samples = 500;
ts = (0:n_samples-1)*T_sample;

% Data for the measurement
Angle = 0;
w = 0;

% Data for python
K = [-2.3660,-3.1772,26.5077,6.5749];
Kcl = [-0.4226, 0];

% Adding the parameters: [angle of the pendulum, The closed loop gain, 
% amount of samples, K]
reset_system(arduino)
set_mode_params(arduino, mode, w,[Angle, Kcl(1) , n_samples, K]);
input('press enter')

W = 6*ones(1,1);
Y = [];
for w = W
reset_system(arduino)
    Y = [Y; get_response(arduino, w, n_samples)];
end
disp('Measurements are done')

%------------------------------------------------------------------------
% Results
figure(1); hold on
plot(Y(4,:),'b-','LineWidth',1.5,'DisplayName','Observed position')
plot(Y(2,:),'r-','LineWidth',1.5,'DisplayName','Real position')
yline(mean(Y(2,:)),'c-','LineWidth',2,'DisplayName','Average real position')
yline(mean(Y(4,:)),'k--','LineWidth',2,'DisplayName','Average observed position')
legend('Interpreter','latex')
ylabel('Position [m]','Interpreter','LaTeX')
xlabel('Time [s]','Interpreter','LaTeX')

figure(2); hold on
plot(Y(6,:),'b-','LineWidth',1.5,'DisplayName','Observed angle')
plot(Y(3,:),'r-','LineWidth',2,'DisplayName','Real angle')
yline(mean(Y(3,:)),'c-','LineWidth',2,'DisplayName','Average real angle')
yline(mean(Y(6,:)),'k--','LineWidth',2,'DisplayName','Average observed angle')
legend('Interpreter','latex')
ylabel('Angle [rad]','Interpreter','LaTeX')
xlabel('Time [s]','Interpreter','LaTeX')

%% EXTENDED FEEDBACK
clc; close all; clear Y

reset_system(arduino)

% Data for the measurement
T_sample = 0.05;
n_samples = 500;
ts = (0:n_samples-1)*T_sample;

% Data for the python
mode = 3;
Angle = 0.0;
Position = 0;
w = 0;
K = [-11.282,-8.4680,41.9837,11.0042,-0.2235];
Ki = K(end);
Ke = K(1:end-1);
Kp = -7.1772;
Kcorr = [-18.4593,-8.4857,42.1184,11.0273];

% Using PI or I
% 0 : using I
% 1 : using PI

% Adding the parameters for I : [angle of the pendulum, Position of the cart,
% 0,Ki ,amount of samples, Ke]

% Adding the parameters: [angle of the pendulum, Position of the cart,
% 1,Ki ,amount of samples,Kp, Kcorr]

Params_I = [Angle, Position,0, Ki,n_samples, Ke];
Params_PI = [Angle, Position, 1,Ki,n_samples,Kp, Kcorr]

set_mode_params(arduino, mode, w,Params_PI);
input('press enter')

W = 6*ones(1,1);
Y = [];
for w = W
reset_system(arduino)
Y = [Y; get_response(arduino, w, n_samples)];
end
disp('Measurements are done')

% Results
figure(1); hold on
plot(Y(4,:),'b-','LineWidth',1.5,'DisplayName','Observed position')
plot(Y(2,:),'r-','LineWidth',1.5,'DisplayName','Real position')
yline(mean(Y(2,100:end)),'c-','LineWidth',2,'DisplayName','Average real position')
yline(mean(Y(4,100:end)),'k--','LineWidth',2,'DisplayName','Average observed position')
legend('Interpreter','latex')
ylabel('Position [m]','Interpreter','LaTeX')
xlabel('Time [s]','Interpreter','LaTeX')


figure(2); hold on
plot(Y(6,:),'b-','LineWidth',1.5,'DisplayName','Observed angle')
plot(Y(3,:),'r-','LineWidth',2,'DisplayName','Real angle')
yline(mean(Y(3,:)),'c-','LineWidth',2,'DisplayName','Average real angle')
yline(mean(Y(6,:)),'k--','LineWidth',2,'DisplayName','Average observed angle')
legend('Interpreter','latex')
ylabel('Angle [rad]','Interpreter','LaTeX')
xlabel('Time [s]','Interpreter','LaTeX')