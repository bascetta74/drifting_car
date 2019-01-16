clear all
close all

addpath ./fblin_law ./trajectory ./vehicle_model

% Feedback linearization parameters
P = 0.5;

% Vehicle initial state
x0 = 0;
y0 = 0;
theta0 = 0;

% Trajectory parameters
R = 0.25;
w = 1;
