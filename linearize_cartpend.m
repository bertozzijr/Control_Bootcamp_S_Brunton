clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

C = eye(4);
sys = ss(A,B,C,0*B);

%%

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 1.5];
    [yL,t,xL] = initial(sys,y0,tspan);
    [t,yNL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);
elseif(s==1)
    y0 = [0; 0; pi+.0001; 0];
    [yL,t,xL] = initial(sys,y0-[0; 0; pi; 0],tspan);
    [t,yNL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);
else    
end

figure
% plot(t,yL);
plot(t,yL+ones(10001,1)*[0; 0; pi; 0]');
hold on
plot(t,yNL);

figure
for k=1:100:length(t)
    drawcartpend_bw(yNL(k,:),m,M,L);
end