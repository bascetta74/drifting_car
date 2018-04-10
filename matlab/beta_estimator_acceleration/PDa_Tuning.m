% PD acceleration based gamma estimator tuning

%% PARAMETERS
wca=50; %[rad/s]

Tsa=1e-3; %[s] controller's sampling time

% I'm assuming to be placing the zero one decade before wca and the high
% frequency pole one decade after wca
%% controllers parameters
Kda=wca;
Kpa=0.1*wca*Kda;
Ta=1/(wca*10); %[s] filter on the derivative action one decade after wca

Na=1/Ta;


%% Continuous time loop transfer function

s=tf('s');

% sys tf
Ga=1/s^2

% controller TF
Ra=Kpa+Kda*s/(1+s*Ta)

% Loop TF
La=Ra*Ga;

% margin plot
figure('name','PDa margin continuous time');
margin(La);grid;
legend('Ra');

%% Discrete time regulator's tf

Rda=c2d(Ra,Tsa,'tustin');

figure('name','PDa bode Ra,Rda');
bode(Ra,Rda);grid;
legend('Ra','Rda');

%% Discrete Time PDa TF Num and Den
% the discretization of the continuous time PD transfer function (with
% filter on the derivative action) has been obtained with the tustin
% transformation

% note that the leading coefficient of the denominator has to be equal to 1
% (it is needed by the discrete TF simulink block)

Num=[(Kpa*(2+Na*Tsa)+2*Kda*Na)/(2+Na*Tsa),(-2*Kpa+Na*Tsa*Kpa-2*Kda*Na)/(2+Na*Tsa)];
Den=[1,(Na*Tsa-2)/(Na*Tsa+2)];

Rda_NumDen=tf(Num,Den,Tsa);

fprintf('\n Rda with c2d command, tustin transformation:\n');
Rda
fprintf('\n Rda Num Den:\n');
Rda_NumDen

figure('name','PDa bode R');
bode(Ra,Rda,Rda_NumDen);grid;
legend('Ra','Rda','Rda NumDen');
%% output coefficients
fprintf('\n Num Rda = \n');
Num
Num_PDa1=Num(1)
Num_PDa2=Num(2)
fprintf('\n den Rda= \n');
Den
Den_PDa1=Den(1)
Den_PDa2=Den(2)





