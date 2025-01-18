clc
close all
clear

N=3;
Tol=1e-10;
Vm=280;
R0=7000;
Theta0=0;
Gamma0=50;
Sigma0 = Gamma0 - Theta0;
 
tf_min = R0 / Vm;
tf_max = (R0/Vm) * ( (1-tand(abs(Sigma0))-secd(Sigma0)) / (log(1-sind(abs(Sigma0)))) );
tf = 28.4;
f = (tf_max - tf) / (tf_max - tf_min);
b = log(1 - sind(abs(Sigma0))) / (f - 1);

x0=0;
y0=0;
tspan=[0 50];
[T,X,T1, SIGMA, A] = Interception(R0, Theta0, Gamma0, x0,y0,N,Vm,f,b,tspan,Tol);


R=X(:,1);
THETA=rad2deg(X(:,2));
GAMMA=rad2deg(X(:,3));
SIGMA=rad2deg(SIGMA);
Xt=X(:,4);
Yt=X(:,5);


% Trajectory plot
figure
plot(Xt,Yt,'LineWidth',2)
hold on
plot(x0,y0,'ro','MarkerSize',10,'LineWidth',2)
plot(max(Xt),0,'ks','MarkerSize',10,'LineWidth',2)
xlabel('X (m)','FontSize',14)
ylabel('Y (m)','FontSize',14)
title('Trajectory','FontSize',16)
legend('Trajectory','Launch Point','Target','FontSize',12)
axis equal
grid on

% Lateral acceleration plot
figure
plot(T1,A,'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Lateral acceleration (m/s^2)','FontSize',14)
title('Lateral acceleration','FontSize',16)
grid on

% Control parameters and heading error variation plot
figure
yyaxis left
plot(T1,deg2rad(SIGMA),'LineWidth',2)
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
if sigma<=0.0001
    sigma=0;
end
dx(1)=-Vm*cos(sigma);
dx(2)=(-Vm*sin(sigma))/x(1);

a = -sign(sigma) * (((b*Vm^2/R0)*(1-sin(abs(sigma))) + (Vm^2*(sin(abs(sigma)))/x(1))));

%a = N*Vm*dx(2);
if abs(a)>100
    a=100*sign(a);
end
dx(3)= a/Vm;
dx(4)= Vm*cos(x(3));
dx(5)= Vm*sin(x(3));
A=[A;a];
T1=[T1;t];
SIGMA=[SIGMA;sigma];
end
 

end