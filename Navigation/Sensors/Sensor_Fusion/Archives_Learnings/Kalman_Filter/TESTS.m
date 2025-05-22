clear all
clc

% Kalman filtering according to simple example presentation:
A = [1.1269 -0.4940 0.1129;1.0000 0 0;0 1.0000 0];
B = [-0.3832;0.5919;0.5191];
%C = [1 0 0];
C = [1 0 0;2 0 1];
D = 0;

Ts = -1;
sys = ss(A,[B B],C,D,Ts,'InputName',{'u' 'w'},'OutputName','y');

Q = 2.3; 
R = [1.2 0;0 2.1]; 
%R = 1;

[kalmf,L,~,Mx,Z] = kalman(sys,Q,R);
kalmf = kalmf(1:2,:);

sys.InputName = {'u','w'};
sys.OutputName = {'yt'}; % Vectorial comprising yt_1 and yt_2
%sys.OutputName = {'yt_1','yt_2'}; 
vIn = sumblk('y = yt + v');
%vIn1 = sumblk('y_1 = yt_1 + v');

kalmf.InputName = {'u','y_1','y_2'};
% kalmf.InputName = {'u','y'};
% kalmf.OutputName = {'ye_1','ye_2'};
kalmf.OutputName = {'ye'};

SimModel = connect(sys,vIn,kalmf,{'u','w','v'},{'yt','ye'});
% SimModel = connect(sys,vIn,kalmf,{'u','w','v'},{'yt','ye_1','ye_2'});

t = (0:10)';
u = sin(t/5);

rng(10,'twister');
w = sqrt(Q)*randn(length(t),1);
% v = sqrt(R)*randn(length(t),1);
v = sqrt(R)*randn(length(t),2)';

out = lsim(SimModel,[u,w,v']);
yt = out(:,1);
ye = out(:,2);
y = yt + v;

clf
subplot(211), plot(t,yt,'b',t,ye,'r--'),
xlabel('Number of samples'), ylabel('Output')
title('Kalman filter response')
legend('True','Filtered')
subplot(212), plot(t,yt - y,'g',t,yt - ye,'r--'),
xlabel('Number of samples'), ylabel('Error')
legend('True - measured','True - filtered')







