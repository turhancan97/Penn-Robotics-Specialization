function [endeff] = computeMiniForwardKinematics(rads1,rads2)

alfa1=0.5*(rads1+rads2)+pi;
alfa=-alfa1+pi/2;

beta=0.5*(rads1-rads2);

l2=2;
l1=1;

r=sqrt(l2^2-(l1*sin(beta))^2)-l1*cos(beta);


endeff = [r*sin(alfa),r*cos(alfa)]