clc;
clear;

% Inserting D-H convention parameters
a1 = 0.07; 
alpha1 = -pi/2; 
d1 = 0.352;
a2 = 0.360; 
alpha2 = 0;
d2 = 0;
a3 = 0; 
alpha3 = -pi/2;
d3 = 0;
a4 = 0;
alpha4 = pi/2;
d4= 0.38;
a5 = 0; 
alpha5 = -pi/2; 
d5 = 0.30;
a6 = 0; 
alpha6 = 0; 
d6 = 0;

% Inserting joint limits for Arms
t1_min = -pi; t1_max = pi;
t2_min = -pi; t2_max = (1.919 - pi/2);
t3_min = -4.014; t3_max = 0.873;
t4_min = -3.491; t4_max = 3.491;
t5_min = -2.007; t5_max = 2.007;
t6_min = (-6.981 + pi/2); t6_max = (6.981 + pi/2);


% Monte Carlo method
% sampling size
N = 20000;
t1 = t1_min + (t1_max-t1_min)*rand(N,1);
t2 = t2_min + (t2_max-t2_min)*rand(N,1);
t3 = t3_min + (t3_max-t3_min)*rand(N,1);
t4 = t4_min + (t4_max-t4_min)*rand(N,1);
t5 = t5_min + (t5_max-t5_min)*rand(N,1);
t6 = t6_min + (t6_max-t6_min)*rand(N,1);





for i = 1:N
T1 = TransMat(a1,alpha1,d1,t1(i));
T2 = TransMat(a2,alpha2,d2,t2(i));
T3 = TransMat(a3,alpha3,d3,t3(i));
T4 = TransMat(a4,alpha4,d4,t4(i));
T5 = TransMat(a5,alpha5,d5,t5(i));
T6 = TransMat(a6,alpha6,d6,t6(i));
T = T1*T2*T3*T4*T5*T6;
X=T(1,4);
Y=T(2,4);
Z=T(3,4);
plot3(X,Y,Z,'.')
hold on;
end

%view(3);
%title('Isometric view');
%xlabel('x (m)');
%ylabel('y (m)');
%zlabel('z (m) ');
%view(2); % top view
%title(' Top view');
%xlabel('x (m)');
%ylabel('y (m)');
view([1 0 0]); % y-z plane
title('Side view, Y-Z');
ylabel('y (m)');
zlabel('z (m)');


function [ T ] = TransMat( a,b,c,d )
T = [ cos(d) -sin(d)*cos(b) sin(d)*sin(b) a*cos(d); sin(d) cos(d)*cos(b) -cos(d)*sin(b) a*sin(d); 0 sin(b) cos(b) c; 0 0 0 1];
end
