function out = fblin_outputInverse_singletrack_spaliviero(xp,yp,psi,beta,P)

% Coordinate transformation
x = xp -P*cos(beta+psi);
y = yp -P*sin(beta+psi);

out = [x; y];
