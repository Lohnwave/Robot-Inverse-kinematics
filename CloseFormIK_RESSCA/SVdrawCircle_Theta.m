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

% load('ALL_Optimizer_IK0305.mat')
load('ALL_QV0305.mat')
% load F:\00-大论文\03-WAM逆运动学\01-1208-WAM_Ikine_HandGet_UDP\Ikine_HandGet\Ikine_HandGet\Qvector.txt

 %% 
% Q = Qvector(:,:,3);
% for i=1:size(Q,1)
%     robot.plot(Q(i,:));
%     hold on
%     T = robot.fkine(Q(i,:));
%     Pos = transl(T);
%     plot3(Pos(1),Pos(2),Pos(3),'k-o');
%     hold on
% end
Optimizer_Name = {'SCA','CGSCA','RES-SCA','GS'};
%% 绘制关节角变换
for i=1:size(Qvector,3)
    figure(i)
    plot(Qvector(:,:,i),'LineWidth',2);
    hl=legend('\theta1','\theta2','\theta3','\theta4','\theta5','\theta6','\theta7');
    set(hl,'FontSize',12,'FontWeight','normal')
    h=figure(i);
    h_axis=get(h,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    set(gcf,'color','w');   % 背景白色
    xlabel('Time','fontsize',16);
    ylabel(['\theta ','\rm/ rad'],'fontsize',16);
    % Rp = a + b.*exp(c.*(f).^-1);
%     title(['NSEI: ','\itRp','=','\ita','+','\itbe','^{c/f}'],'fontsize',20,'FontName','Times New Roman');  
    title([Optimizer_Name{i}],'fontsize',20,'FontName','Times New Roman'); 
    %     hl=legend('test');   %右上角的标   
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    print('-dtiff','-r600',[Optimizer_Name{i},'SVtheta','.tiff']);
end


