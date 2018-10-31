% Callback function drifting stabilization controller

%% Filter Parameters:
wf=250; %[rad/s] cut-off frequency of the filter of wz,beta,V_opt
Ts=0.01; %[s]
s=tf('s');
F=1/(1+s/wf);
FilterDTF=c2d(F,Ts,'tustin');
Num_Filter=FilterDTF.Num{1};
Den_Filter=FilterDTF.Den{1};
 
%% Eqpoint Parameters:
delta_eqpoint=-20*pi/180; %[rad]
% Fxr_eqpoint=2.09; %[N] moquette with solid rear axle
Fxr_eqpoint=3.006; %[N] moquette with rear differential

Vx_eqpoint=1.0; %[m/s]

% Vy_eqpoint=-0.7055; %[m/s] moquette with solid rear axle
% r_eqpoint=1.848; %[rad/s] moquette with solid rear axle

Vy_eqpoint=-0.7880; %[m/s] moquette with rear differential
r_eqpoint=2.31807; %[rad/s] moquette with rear differential

%% Controller state feedback matrix (A-BK) with
% d_x=[d_Vx,d_Vy,d_r],d_u=d_delta,d_Fxr]

K_Vx_delta=-0.8304;
K_Vy_delta=-0.9889;
K_r_delta=0.6146;

K_Vx_Fxr=1.2987;
K_Vy_Fxr=-6.6663;
K_r_Fxr=0.8143;


%% Transmission and motor parameters:
Fdrag=1.75; %[N] moquette with rear differential
Rw=0.049; %[m]
Kt=1/340.34; %[Nm/A]
Imax=13; %[A] max motor current (set in VESC GUI)
% tau=0.1159; %transmission gear ratio with solid rear axle
tau=0.09799; %transmission gear ratio with rear differential

%% Control Inputs Saturation Limits
delta_max=45*pi/180; %[rad]
delta_min=-delta_max; %[rad]


%% Open Loop Control Sequence Parameters:
% PHASE 1 (lasts from 0 to dt1):
dt1=0.5; %[s]
Throttle_Ref_1=1; 

% PHASE 2 (lasts dt2 s):
dt2=1; %[s]
Throttle_Ref_2=0.5;

% PHASE 3 (lasts dt3 s):
dt3=3; %[s] 
Steer_Ref_3=20*pi/180; %[rad]
Throttle_Ref_3=0.5;

% PHASE 4 (lasts dt4 s: RAMP):
dt4=1; %[s]
Throttle_Ref_4=0.8;

% 

t_start_3=dt1+dt2;%[s]
t_start_4=dt1+dt2+dt3; %[s]
t_end_4=dt1+dt2+dt3+dt4; %[s]

m4=(delta_eqpoint-Steer_Ref_3)/dt4;
q4=delta_eqpoint-m4*(dt1+dt2+dt3+dt4);

