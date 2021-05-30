function [phi,theta,psi]=rotmat2zyz(R)
phi=atan2(R(2,3),R(1,3));
theta=atan2(sqrt(R(1,3)^2 + R(2,3)^2),R(3,3));
psi=atan2(R(3,2),-R(3,1));
end