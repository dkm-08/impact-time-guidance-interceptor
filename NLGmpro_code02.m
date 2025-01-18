clc
close all
clear

N=3;
Tol=1e-12;
tspan=[0 50];
tf = 30;
xt=0;
yt=0;


Vm=280;
x0=-7000;
y0=0;
% Get initial R and theta
[R0,Theta0] = GetRandTheta(xt,yt,x0,y0)

Gamma0=50;
Sigma0 = Gamma0 - Theta0;

%Get Control parameters
[b1,f1,tf_min1,tf_max1]=GetParameters(R0,Vm,Sigma0,tf)
% Solve Kinematic eqaution
[T,X,T1, SIGMA, A] = Interception(R0, Theta0, Gamma0, x0,y0,N,Vm,f1,b1,tspan,Tol);
% Get Plots
plotresults(T,X,x0,y0,T1,A,SIGMA,b1,f1,xt,yt)

x0=7000;
y0=0;
% Get initial R and theta
[R0,Theta0] = GetRandTheta(xt,yt,x0,y0) 
Gamma0=240;
Sigma0 = Gamma0 - Theta0;
%Get Control parameters
[b2,f2,tf_min2,tf_max2]=GetParameters(R0,Vm,Sigma0,tf)

% Solve Kinematic eqaution
[T,X,T1, SIGMA, A] = Interception(R0, Theta0, Gamma0, x0,y0,N,Vm,f2,b2,tspan,Tol);
% Get Plots
plotresults(T,X,x0,y0,T1,A,SIGMA,b2,f2,xt,yt)

x0=6000;
y0=4000;
% Get initial R and theta
[R0,Theta0] = GetRandTheta(xt,yt,x0,y0) 
Gamma0=-90;
Sigma0 = Gamma0 - Theta0;
%Get Control parameters
[b3,f3,tf_min3,tf_max3]=GetParameters(R0,Vm,Sigma0,tf)
% Solve Kinematic eqaution
[T,X,T1, SIGMA, A] = Interception(R0, Theta0, Gamma0, x0,y0,N,Vm,f3,b3,tspan,Tol);
% Get Plots
plotresults(T,X,x0,y0,T1,A,SIGMA,b3,f3,xt,yt)


function [R0,theta0] = GetRandTheta(xt,yt,x0,y0)
 theta0=atan2d((yt-y0),(xt-x0));
 R0=sqrt((yt-y0)^2+(xt-x0)^2);
end

function plotresults(T,X,x0,y0,T1,A,SIGMA,b,f,xt,yt)
R=X(:,1);
THETA=rad2deg(X(:,2));
GAMMA=rad2deg(X(:,3));
SIGMA=rad2deg(SIGMA);
Xt=X(:,4);
Yt=X(:,5);


% Trajectory plot
figure(1)
plot(Xt,Yt,'LineWidth',2)
hold on
plot(x0,y0,'ro','MarkerSize',10,'LineWidth',2)
plot(xt,yt,'ks','MarkerSize',10,'LineWidth',2)
xlabel('X (m)','FontSize',14)
ylabel('Y (m)','FontSize',14)
title('Trajectory','FontSize',16)
legend('Trajectory','Launch Point','Target','FontSize',12,'Location','best')
axis equal
grid on
hold on

figure(2)
plot(T,R,'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Range','FontSize',14)
grid on
hold on

% Lateral acceleration plot
figure(3)
plot(T1,A,'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Lateral acceleration (m/s^2)','FontSize',14)
title('Lateral acceleration','FontSize',16)
grid on
hold on

% Control parameters and heading error variation plot
figure(4)
yyaxis left
plot(T1,SIGMA,'LineWidth',2)
ylabel('\delta (rad)','FontSize',14)
yyaxis right
plot(T,b*ones(size(T)),'--','LineWidth',2)
hold on
plot(T,f*ones(size(T)),'-.','LineWidth',2)
ylabel('Control parameters b, f','FontSize',14)
xlabel('Time (sec)','FontSize',14)
title('Control parameters and \delta','FontSize',16)
legend('\delta','b','f','FontSize',12)
grid on
end

function [b,f,tf_min,tf_max]=GetParameters(R0,Vm,Sigma0,tf)
tf_min = R0 / Vm;
tf_max = (R0/Vm) * ( (1-tand(abs(Sigma0))-secd(Sigma0)) / (log(1-sind(abs(Sigma0)))) );

f = (tf_max - tf) / (tf_max - tf_min);
b = log(1 - sind(abs(Sigma0))) / (f - 1);
end

function [T,X,T1, SIGMA, A] = Interception(R0, Theta0, Gamma0, x0,y0,N,Vm,f,b,tspan,Tol)
  
 A=[];
 T1=[];
 SIGMA=[];
 

x0=[R0  deg2rad(Theta0) deg2rad(Gamma0) x0 y0];
At=Tol;
AT= [At At At At At];
options = odeset('RelTol',At,'AbsTol',AT,'Events',@Deeqtn);
[T,X] = ode45(@DEsolver,tspan,x0,options);


function [dx]=DEsolver(t,x)
dx=zeros(5,1);
sigma=x(3)-x(2);
if sigma<0.0001
    sigma=0;
end
dx(1)=-Vm*cos(sigma);
dx(2)=(-Vm*sin(sigma))/x(1);

a = -sign(sigma) * (((b*Vm^2/R0)*(1-sin(abs(sigma))) + (Vm^2*(sin(abs(sigma)))/x(1))));


dx(3)= a/Vm;
dx(4)= Vm*cos(x(3));
dx(5)= Vm*sin(x(3));
A=[A;a];
T1=[T1;t];
SIGMA=[SIGMA;sigma];
end
 

end