function out = fblin_output_singletrack_spaliviero(x,y,psi,beta,P)

% Coordinate transformation
xp = x+P*cos(beta+psi);
yp = y+P*sin(beta+psi);

out = [xp; yp];
