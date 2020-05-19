% /******************************************************************************\
% * Copyright (C) 2020 . Luozu, All rights reserved.                            *
% * Version: 2.0																*
% * Last Revised: 2020-03-05													*
% * Eritor: Luozu																*
% \******************************************************************************/
clear;
close all;
clc;
% close all;
L1 = Link('d', 0, 'a', 0, 'alpha', -pi/2,'qlim', [-2.6,2.6]);
L2 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-2,2]);
L3 = Link('d', 0.55, 'a', 0.045, 'alpha', -pi/2,'qlim', [-2.8,2.8]);
L4 = Link('d', 0, 'a', -0.045, 'alpha', pi/2,'qlim', [-0.9,3.1]);
L5 = Link('d', 0.3, 'a', 0, 'alpha', -pi/2,'qlim', [-4.76,1.24]);
L6 = Link('d', 0, 'a', 0, 'alpha', pi/2,'qlim', [-1.6,1.6]);
L7= Link('d', 0.06, 'a', 0, 'alpha',0,'qlim', [-3,3]);
b=isrevolute(L1);  %Link 类函数
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);   %SerialLink 类函数
robot.name='barrett wam 机器人';

J=[0 0 0 0 0 0 0];
theta=[0 0 0 0 0 0 0];
load('ALL_QV0305.mat')

% ---------------draw circle-------------------
x = 0.0;  % 圆心
z = 0.1; % 圆心
r = 0.25; % R
t = 0:(2*pi/50):2*pi; % 圆滑性设置
x=x+r*cos(t);
y = -0.7*ones(1,size(t,2));
z=z+r*sin(t);
Pos(:,1) = x;
Pos(:,2) = y;
Pos(:,3) = z;

Time = t(1:50);
Q = Qvector(:,:,4);
PosTrack = zeros(50,3);
for i=1:size(Q,1)
    robot.plot(Q(i,:));
    hold on
    T = robot.fkine(Q(i,:));
    PosTrack(i,:) = transl(T);
    plot3(PosTrack(i,1),PosTrack(i,2),PosTrack(i,3),'k-o');
    hold on
end
