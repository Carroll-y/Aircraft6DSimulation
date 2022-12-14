% 给定基准状态参数和输入指令
% 调用Aircraft6DSimulation仿真
% 返回输入时间曲线和状态时间曲线

clear
clc
close all

%% 基准状态参数
x0 = [75   %V0=75m/s
      0.6659*pi/180    %alpha0=0.6659deg
      0    %beta0=0deg
      
      0    %p0=0deg/s
      0    %q0=0deg/s
      0    %r0=0deg/s

      0    %phi0=0deg
      0.6659*pi/180    %theta0=0.6659deg
      0    %psi0=0deg
      
      0    %x0=0m
      0    %y0=0m
      -3000];   %h=3000m

u0 = [-0.23791*pi/180  %升降舵偏角-0.23791°
     0                 %副翼偏角0°
     0];               %方向舵偏转0°

% 基准状态时间
t1 = 10;


%% 输入指令
%以下状态5选1，仿真其中某种状态时需手动将其他状态注释或直接更改参数

% % (1) 基准状态
% TF = t1; %仿真时间10s
% t2 = 0;  %指令结束时间
% e = 0;   %升降舵输入指令值
% a = 0;   %副翼输入指令值
% r = 0;   %方向舵输入指令值

% (2) 升降舵相对基准状态上偏5度，并保持2秒，然后回到基准舵偏角，仿真时长 60s
TF = t1+60;     %仿真时间
t2 = t1+2;      %指令结束时间
e = 5*pi/180;   %升降舵输入指令值
a = 0;          %副翼输入指令值
r = 0;          %方向舵输入指令值

% % (3) 副翼相对基准状态偏转正2度，仿真时长30s
% TF = t1+30;     %仿真时间
% t2 = TF;        %指令结束时间
% e = 0;          %升降舵输入指令值
% a = 2*pi/180;   %副翼输入指令值
% r = 0;          %方向舵输入指令值

% % (4) 方向舵相对基准状态偏转负2度，仿真时长30s
% TF = t1+30;     %仿真时间
% t2 = TF;        %指令结束时间
% e = 0;          %升降舵输入指令值
% a = 0;          %副翼输入指令值
% r = -2*pi/180;  %方向舵输入指令值

% % (5) 升降舵、副翼及方向舵相对基准状态同时偏转正2度，并保持2秒，然后都回到基准偏角，仿真时长60s；
% TF = t1+60;       %仿真时间
% t2 = t1+2;        %指令结束时间
% e = 2*pi/180;     %升降舵输入指令值
% a = 2*pi/180;     %副翼输入指令值
% r = 2*pi/180;     %方向舵输入指令值


%% run the model
dt = 0.1;  %仿真步长
out = sim('Aircraft6DSimulation.slx');


%% 读取数据
t = out.simX.Time;

d_e   = out.simU.Data(:,1) * 180/pi; %模型中z轴向下，delta_e为反向，这里转正
d_a   = out.simU.Data(:,2) * 180/pi;
d_r   = out.simU.Data(:,3) * 180/pi;

V     = out.simX.Data(:,1);
alpha = out.simX.Data(:,2) * 180/pi;
beta  = out.simX.Data(:,3) * 180/pi;

p     = out.simX.Data(:,4) * 180/pi;
q     = out.simX.Data(:,5) * 180/pi;
r     = out.simX.Data(:,6) * 180/pi;

phi   = out.simX.Data(:,7) * 180/pi;
theta = out.simX.Data(:,8) * 180/pi;
psi   = out.simX.Data(:,9) * 180/pi;

x_g   = out.simX.Data(:,10);
y_g   = out.simX.Data(:,11);
z_g   = -out.simX.Data(:,12); %输出的z轴方向向下，这里转化为高度


%% plot the input
figure("Name","控制参数")
subplot(3,1,1)
plot(t,d_e)
xlabel('t(s)')
ylabel('delta_e(deg)')
grid on 

subplot(3,1,2)
plot(t,d_a)
xlabel('t(s)')
ylabel('delta_a(deg)')
grid on 

subplot(3,1,3)
plot(t,d_r)
xlabel('t(s)')
ylabel('delta_r(deg)')
grid on 


%% plot the reslults
figure('Name','状态参数')
subplot(4,3,1)
plot(t,V)
xlabel('t(s)')
ylabel('V(m/s)')
grid on 

subplot(4,3,2)
plot(t,alpha)
xlabel('t(s)')
ylabel('alpha(deg)')
grid on

subplot(4,3,3)
plot(t,beta)
xlabel('t(s)')
ylabel('beta')
grid on

subplot(4,3,4)
plot(t,p)
xlabel('t(s)')
ylabel('p(deg/s)')
grid on

subplot(4,3,5)
plot(t,q)
xlabel('t(s)')
ylabel('q(deg/s)')
grid on

subplot(4,3,6)
plot(t,r)
xlabel('t(s)')
ylabel('r(deg/s)')
grid on

subplot(4,3,7)
plot(t,phi)
xlabel('t(s)')
ylabel('phi(deg)')
grid on

subplot(4,3,8)
plot(t,theta)
xlabel('t(s)')
ylabel('theta(deg)')
grid on

subplot(4,3,9)
plot(t,psi)
xlabel('t(s)')
ylabel('psi(deg)')
grid on

subplot(4,3,10)
plot(t,x_g)
xlabel('t(s)')
ylabel('x_g(m)')
grid on

subplot(4,3,11)
plot(t,y_g)
xlabel('t(s)')
ylabel('y_g(m)')
grid on

subplot(4,3,12)
plot(t,z_g)
xlabel('t(s)')
ylabel('z_g(m)')
grid on