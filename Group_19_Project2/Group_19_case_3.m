A=[-520 -220 0 0 0
220 -500 -999994 0 2e8
0 1 0 0 0
0 0 66667 -0.1667 -1.3333e7
0 0 0 1 0]
B=[1000;
0 
0
0
0];
C=[0 0 0 0 1];
sysc= ss(A,B,C,0);

%Number of states
n=5;
% Detection, Processing and merging times
T_d = 4e-03;
T_p = 10e-03;
T_m = 42e-03;

% number of cores
Y = 4;

% Sensing time for single core
T_sensing = T_d + (6 * T_p) + (T_m);

% Computation and Actuation time

T_computation = 1.5e-03;
T_actuation = 0.5e-03;



tau_t = (T_sensing + T_computation + T_actuation);

% period of camera frame arrival
fh = 1/30;  % 30 fps

% Sensing to Actuation Delay
tau = (ceil(tau_t / fh))*fh;


% sampling period is equal to delay / number of processing cores
h=tau/Y;



%% discretization with delay
%%sysd= c2d(sysc, h); phi = sysd.a; Gamma = sysd.b; C = sysd.c;
%%Gamma0 = inv(A)*(expm(A*(h-tau))-expm(A*0))*B;
%%Gamma1 = inv(A)*(expm(A*h)-expm(A*(h-tau)))*B;
 
sysc= ss(A,B,C,0);
sysd= c2d(sysc, h); Ad = sysd.a; Bd= sysd.b; Cd = sysd.c;
n = 5;
%Frames processed
gamma = 4;
%% augmented system with delay
phi = [Ad Bd zeros(n,gamma-1);
zeros(gamma-1,n) zeros(gamma-1,1) eye(gamma-1, gamma -1);
zeros(1,n) zeros(1,1) zeros(1,gamma-1) ];
Gamma = [zeros(n+gamma-1,1);1];
C = [Cd zeros(1,gamma)];
%% augmentation for the integral tracking action
phi_aug= [phi zeros(n+gamma,1);
C 1];
Gamma_aug= [Gamma; 0];
C_aug= [C 0];
%% LQR tracking controller
Q = [zeros(n+gamma, n+gamma) zeros(n+gamma,1);zeros(1,n+gamma) 10^10];
[X,L,G] = dare(phi_aug, Gamma_aug, Q, 0.01);
K = -G;
x0 = [0;0;0.00;0;0;];
z0 = [x0; zeros(gamma,1)];
x(1) = C*z0;
time(1) = 0;
e(1) = C*z0;
for i=1:50
if i==1
time(1) = 0;
else
time(i) = time(i-1) + h;
end
y(i) = C*z0;
if i>20
Road(i) = 0.002;
else
Road(i) = 0;
end
e(i+1) = e(i) + y(i) -Road(i);
u = K*[z0;e(i)];
z_1 = phi*z0 + Gamma*u;
z0 = z_1;
end
plot(time, y,'b', time, Road,'r')
