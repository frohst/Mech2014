clear all
close all
clc
%% Constants
a=[0,0.1,0.05];%x,y,z
b=[0,0,0];
c=[0,0,0];

d=[0,0,0.05];
e=[0,0,0];
f=[0,0,0];

d772=[60/0.17,0.3/9.807,64E-3]; %degree/second, N*m of torque, kg mass
d770=[60/0.06,0.11/9.807,47E-3];

maxRotation=105*pi/180; %range of servos
thetaMount=-maxRotation/2; %orientation of servo1. see diagrams for labeling and measurement baselines.
phiMount=-maxRotation/2;
%% Variables
knee=0.15;

%% solved terms

legExist=false;
%solve for leg such that it can lift itself off the ground on two legs. if
%leg <=0, or DNE legExist=false
leg=((0.22125*2+d772(3)+2*d770(3))*9.807*0.1*sqrt(2)-d772(2)-d770(2)-d770(3)*9.807*knee)/(9.807*((0.22125*2+d772(3)+2*d770(3))+d772(3)+2*d770(3)));
if leg>0
    legExist=true;
end

if (legExist)
    c=[.1,.1+0.1,0];
    f=[-.1,-.1,0];
    
    %solves the angles needed to place c where it is. theta relative to x
    %axis, phi is relative to -perp of theta.
    syms thetaSym phiSym
    [phi,theta]=vpasolve([knee*cos(thetaSym)+(leg-knee)*(sin(thetaSym+phiSym))==hypot(c(1)-a(1),c(2)-a(2)),knee*sin(thetaSym)-(leg-knee)*(cos(phiSym+thetaSym))==c(3)-a(3)],[thetaSym,phiSym],[thetaMount,maxRotation+thetaMount;phiMount,maxRotation+phiMount]);
    b=[knee*cos(theta)*c(1)/hypot(c(1)-a(1),c(2)-a(2))+a(1),knee*cos(theta)*c(2)/hypot(c(1)-a(1),c(2)-a(2))+a(2),knee*sin(theta)+a(3)];
    
    [phi2,theta2]=vpasolve([knee*cos(thetaSym)+(leg-knee)*(sin(thetaSym+phiSym))==hypot(f(1)-d(1),f(2)-d(2)),knee*sin(thetaSym)-(leg-knee)*(cos(phiSym+thetaSym))==f(3)-d(3)],[thetaSym,phiSym],[thetaMount,maxRotation+thetaMount;phiMount,maxRotation+phiMount]);
    e=[knee*cos(theta)*f(1)/hypot(f(1)-d(1),f(2)-d(2))+d(1),knee*cos(theta)*f(2)/hypot(f(1)-d(1),f(2)-d(2))+d(2),knee*sin(theta)+d(3)];
end