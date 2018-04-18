% Callback function drifting stabilization controller

% Filter Parameters:
wf=100; %[rad/s] cut-off frequency of the filter of wz,beta,V_opt
Ts=0.01; %[s]
s=tf('s');
F=1/(1+s/wf);
FilterDTF=c2d(F,Ts,'tustin');

% Eqpoint Parameters:
delta_eqpoint=-10*pi/180; %[rad]
Fxr_eqpoint=1.25488; %[N]

Vx_eqpoint=1.5; %[m/s]
Vy_eqpoint=-0.6100; %[rad]
r_eqpoint=1.24259; %[rad/s]

% Controller state feedback matrix (A+BK) with
% d_x=[d_Vx,d_Vy,d_r],d_u=d_delta,d_Fxr]
K=[-0.1667   -0.2873    0.6444
   -0.8841   -9.7626    8.4897];

% Transmission and motor parameters:
Fdrag=2; %[N]
Rw=0.049; %[m]
Kt=1/340.34; %[Nm/A]
Imax=10; %[A] max motor current (set in VESC GUI)
tau=0.1159; %transmission gear ratio

% Control Inputs Saturation Limits
delta_max=35*pi/180; %[rad]
delta_min=-delta_max; %[rad]

