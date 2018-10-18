% beta_estimator_velocity_driftingcar_callaback function

ds_nonzero_threshold=1e-3; %[m] threshold below which ds is considered to be zero

theta_offset=5; %[rad]

% VELOCITY BETA ESTIMATOR
p=0.1; %[m]
Kp=500;

% ACCELERATION BETA ESTIMATOR
acc_integrator_initial_condition=[0,0,0,0.1]; %[x,y,gamma,v]
Num_PDa=[2.8403 -2.7708]*1e5;
Den_PDa=[1 0.1111];