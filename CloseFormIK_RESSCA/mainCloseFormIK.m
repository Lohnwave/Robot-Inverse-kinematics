% /******************************************************************************\
% * Copyright (C) 2020 . Luozu, All rights reserved.                            *
% * 带连杆偏移的7自由度Barrett WAM 逆运动学封闭式解析解： mian                     *
% * Version: 1.0																*
% * Last Revised: 2020-02-29													*
% * Eritor: Luozu																*
% \******************************************************************************/
clear;
clc;
close all;
Optimizer_Name={'IKOptimizer'};
% Optimizer_Name={'SCA','CGSCA','RES_SCA'};
% draw Sine
% t = 0:(2*pi/50):2*pi;
% Pos = zeros(size(t,2),3);
% Pos(:,1) = 0.7*ones(1,size(t,2)) - 0.6*t/(2*pi);
% Pos(:,2) = -0.422*ones(1,size(t,2));
% Pos(:,3) = 0.2*ones(1,size(t,2)) - 0.2*sin(t);
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
% plot(Circle1,Circle2,'Color','k','linewidth',1);


yaw = 0;
pitch = 0;
roll = pi / 2;
% 欧拉角转换四元素
Tool_Orient = angle2quat(yaw, pitch, roll,'XYZ');  % matlab 四元素顺序默认ZYX ！！！ Eigen: W xyz
% Tool_Orient = [0.70710678118654746,0,0,0.70710678118654746];
curQ = [0,0,0,0,0,0,0];
curPhi = 0;
Qvector = zeros(50,7,3);
Phivector = zeros(3,50,1);
for k=1: size(Optimizer_Name,2)
    disp(['Optimizer_Name: ', Optimizer_Name{k}]);
    for i=1:50
        disp(['i= ', num2str(i)]);
        BSOFUNC = Optimizer_Name{k};
        tic;
        [Q,phi,allgbestval,sign] = feval(BSOFUNC,Pos(i,:),Tool_Orient, curQ,curPhi);
        Time(i,k)=toc;
        if sign==1
            Qvector(i,:,k) = Q;
            Phivector(k,i) = phi;
            error(k,i,:) = allgbestval;
            curQ = Q;
            curPhi = phi;
        end
    end
    file_name= [Optimizer_Name{k},'_Optimizer_IK.mat'];
    save (file_name);
end
%     file_name= ['ALL_Optimizer_IK.mat'];
%     save (file_name);