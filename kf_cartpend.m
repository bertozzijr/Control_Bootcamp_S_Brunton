clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = -1; % pendulum up (s=1)

% y = [x; dx; theta; dtheta];
A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C = [1 0 0 0];  

D = zeros(size(C,1),size(B,2));

%%  Augment system with disturbances and noise
Vd = .1*eye(4);  % disturbance covariance
Vn = 1;       % noise covariance

BF = [B Vd 0*B];  % augment inputs to include disturbance and noise

sysC = ss(A,BF,C,[0 0 0 0 0 Vn]);  % build big state space system... with single output

sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));  % system with full state output, disturbance, no noise

%%  Build Kalman filter
[L,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter
Kf = (lqr(A',C',Vd,Vn))';   % alternatively, possible to design using "LQR" code

sysKF = ss(A-L*C,[B L],eye(4),0*[B L]);  % Kalman filter estimator

%%  Estimate linearized system in "down" position (Gantry crane)
dt = .01;
t = dt:dt:50;

uDIST = randn(4,size(t,2));
uNOISE = randn(size(t));
u = 0*t;
u(100:120) = 100;     % impulse
u(1500:1520) = -100;  % impulse

uAUG = [u; Vd*Vd*uDIST; uNOISE];

[y,t] = lsim(sysC,uAUG,t);
[xtrue,t] = lsim(sysFullOutput,uAUG,t);


[x,t] = lsim(sysKF,[u; y'],t);

plot(t,xtrue,'-',t,x,'--','LineWidth',2)

figure
plot(t,y)
hold on
plot(t,xtrue(:,1),'r')
plot(t,x(:,1),'k--')