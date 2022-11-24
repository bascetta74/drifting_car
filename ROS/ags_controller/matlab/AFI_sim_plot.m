close all

figure,plot(t,Fy_ref,'r--', t,Fyf),grid,xlabel('Time [s]'),ylabel('Front lateral force [N]'),...
    legend('Fy ref','Fy act')

if exist('beta_est')
    figure,plot(t,beta,'r--', t,beta_est),grid,xlabel('Time [s]'),ylabel('Sideslip angle [rad]'),...
    legend('beta','beta est')
end