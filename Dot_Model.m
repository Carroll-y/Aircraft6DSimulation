function [XDOT] = Dot_Model(X,U)

%% 定义变量
% 飞行参数
V = X(1);
alpha = X(2);
beta = X(3);

% 角速度分量
p = X(4);
q = X(5);
r = X(6);

% 偏转角
phi = X(7);
theta = X(8);
psi = X(9);

% 位置
x = X(10);
y = X(11);
z = X(12);

% 输入参数
d_e = U(1); %delta_e 升降舵偏转
d_a = U(2); %delta_a 副翼偏转 
d_r = U(3); %delta_r 方向舵偏转


%% 给定常数
% 机身参数
m = 4510; %mass(kg)
b = 19.81; %翼展(m) 
c = 1.981; %平均气动弦长(m)
S = 39.02; %机翼面积(m^2)

I_x = 26190; %(N.m^2)
I_y = 33460; %(N.m^2)
I_z = 47920; %(N.m^2)
I_xz = 1490; %(N.m^2)

% 发动机参数
T = 5605.18; %推力(N)
varphi = 0; %安装角(°)

% 大气密度(kg/m^3)
rho = 0.9096;

% 重力加速度
g = 9.81; %重力加速度(m/s^2)

% 气动系数参数
A = 0.052;
C_z0 = -0.38;
C_za = -5.660;
C_zq = -19.97;
C_zde = -0.608; %C_z^delta_e
C_x0 = -0.041;
C_m0 = 0.008;
C_ma = -1.310;
C_mq = -34.2;
C_mde = -1.74; %C_m^delta_e

C_ybe = -0.6; %C_y^beta
C_yp = -0.2;
C_yr = 0.4;
C_ydr = 0.15; %C_y^delta_r
C_lbe = -0.08; %C_l^beta
C_lp = -0.5;
C_lr = 0.06;
C_lda = -0.15; %C_l^delta_a
C_ldr = 0.015; %C_l^delta_r
C_nbe = 0.1;
C_np = -0.06;
C_nr = -0.18;
C_ndr = -0.12; %C_n^delta_r
C_nda = -0.001; %C_n^delta_a


%% 中间变量
%动压
Q = 0.5*rho*V^2; 

% 素的分量
u = V*cos(alpha)*cos(beta);
v = V*sin(beta);
w = V*sin(alpha)*cos(beta);

% 转动惯量系数
Gamma = I_x*I_z - I_xz^2;
c1 = ((I_y - I_z)*I_z - I_xz^2)/Gamma;
c2 = ((I_x - I_y + I_z)*I_xz)/Gamma;
c3 = I_z/Gamma;
c4 = I_xz/Gamma;
c5 = (I_z - I_x)/I_y;
c6 = I_xz/I_y;
c7 = 1/I_y;
c8 = (I_x*(I_x - I_y) + I_xz^2)/Gamma;
c9 = I_x/Gamma;


%% 气动系数
C_z = C_z0 + C_za*alpha + 0.5*c/V*C_zq*q + C_zde * d_e; 
C_x = C_x0 - A*C_z0^2 - 2*A*C_z0*(C_za*alpha + 0.5*c/V*C_zq*q + C_zde*d_e);
C_y = C_ybe*beta + 0.5*b/V*(C_yp*p + C_yr*r) + C_ydr*d_r;

C_l = C_lbe*beta + 0.5*b/V*(C_lp*p + C_lr*r) + C_lda*d_a + C_ldr*d_r;
C_m = C_m0 + C_ma*alpha + 0.5*c/V*C_mq*q + C_mde*d_e;
C_n = C_nbe*beta + 0.5*b/V*(C_np*p + C_nr*r) + C_nda*d_a + C_ndr*d_r;


%% 力\力矩（体轴）
%力(气动力    +推力           +重力）
Fx = Q*S*C_x + T*cos(varphi) - m*g*sin(theta);
Fy = Q*S*C_y + 0             + m*g*cos(theta)*sin(phi);
Fz = Q*S*C_z - T*sin(varphi) + m*g*cos(theta)*cos(phi);

%气动力矩
L = Q*S*b*C_l;
M = Q*S*c*C_m;
N = Q*S*b*C_n;


%% Dot
% Vdot,alphadot,betadot
VDot     = ( Fx*cos(alpha)*cos(beta) + Fy*sin(beta) + Fz*sin(alpha)*sin(beta))/m;
alphaDot = (-Fx*sin(alpha) + Fz*cos(alpha))/(m*V*cos(beta)) + q - tan(beta)*(p*cos(alpha) + r*sin(alpha));
betaDot  = (-Fx*cos(alpha)*sin(beta) + Fy*cos(beta) - Fz*sin(alpha)*sin(beta))/(m*V) + p*sin(alpha) - r*cos(alpha);

% udot,vdot,wdot
% uDot = VDot*cos(alpha)*cos(beta)-alphaDot*V*sin(alpha)*cos(beta)-betaDot*V*cos(alpha)*sin(beta);
% vDot = VDot*sin(beta)+betaDot*V*cos(beta);
% wDot = VDot*sin(alpha)*cos(beta)+alphaDot*V*cos(alpha)*cos(beta)-betaDot*V*sin(alpha)*sin(beta);

% pdot,qdot,rdot
pDot = (c1*r + c2*p)*q + c3*L + c4*N;
qDot = c5*p*r - c6*(p^2 - r^2) + c7*M;
rDot = (c8*p - c2*r)*q + c4*L + c9*N;

% phidot,thetadot,psidot
phiDot   = p + tan(theta)* (q*sin(phi) + r*cos(phi));
thetaDot = q*cos(phi) - r*sin(phi);
psiDot   = (q*sin(phi) + r*cos(phi))/cos(theta);

% xdot,ydot,zdot
xDot = u*cos(theta)*cos(psi) + v* (sin(theta)*sin(phi)*cos(psi) - cos(phi)*sin(psi)) + w* (sin(theta)*cos(phi)*cos(psi) + sin(phi)*sin(psi));
yDot = u*cos(theta)*sin(psi) + v* (sin(theta)*sin(phi)*sin(psi) + cos(phi)*cos(psi)) + w* (sin(theta)*cos(phi)*sin(psi) + sin(phi)*cos(psi));
zDot = - u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);


%% 输出
XDOT = [VDot
        alphaDot
        betaDot

        pDot
        qDot
        rDot

        phiDot
        thetaDot
        psiDot

        xDot
        yDot
        zDot];
