% sys tf=1/s^2

s=tf('s');
G=1/s^2;
% PD with filter on the derivative action
% target bw: 100 Hz
Kp=2*pi*100;
Td=1; %[s]
N=1000;

Kd=Td*Kp;

R=Kp+Kd*s/(s/N+1)

L=R*G;
margin(L)

%% discretization
Ts=0.01; %[s]

Gd=c2d(G,Ts);

Rd=c2d(R,Ts);
Rdf=c2d(R,Ts,'tustin');

Ld=Rd*Gd;
margin(Ld)

%% trasf bilineare alla FdT tempo continuo

N=100;
z=tf('z',Ts);
Rpd=Kp+Kd*(2/Ts*(z-1)/(z+1))/((2/Ts*(z-1)/(z+1))/N+1)