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
Q = Qvector(:,:,3);
PosTrack = zeros(50,3);
for i=1:size(Q,1)
%     robot.plot(Q(i,:));
%     hold on
    T = robot.fkine(Q(i,:));
    PosTrack(i,:) = transl(T);
%     plot3(PosTrack(1),PosTrack(2),PosTrack(3),'k-o');
%     hold on
end
Eline = 2;
Tline = 1;
h1 = figure(1);

plot3(x,y,z,'r--','LineWidth',Eline);
hold on
plot3(PosTrack(:,1),PosTrack(:,2),PosTrack(:,3),'k-d','LineWidth',Tline);
ylim([-0.7-0.0005 -0.7+0.0005]);
grid on
    h_axis=get(h1,'Children');
    set(h_axis,'LineWidth',1.5);
%     legend1 = legend('Expected path','Actual path');
    legend1 = legend('\fontname{宋体}期望','\fontname{宋体}实际');
    set(legend1,...
    'Position',[0.669401787207326 0.722619049889701 0.155357141207371 0.0904761882055374]);
    set(legend1,'FontSize',12,'FontWeight','normal')
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    set(gcf,'color','w');   % 背景白色
    xlabel(['X','\rm/ m'] ,'fontsize',16);  %x坐标  字体12 
    %ylabel('log(Average Function Value Error)','fontsize',16);  
    ylabel(['Y','\rm/ m'],'fontsize',16);  
    zlabel(['Z','\rm/ m'],'fontsize',16);  
%     title('Closed-Form solution','fontsize',20,'FontName','Times New Roman');  
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    title('轨迹跟踪','fontsize',16,'FontName','宋体');
%     print('-dtiff','-r600',[num2str(1),'SVTrack','中文.tiff']);
    
h2 = figure(2);
plot(x(1:50),'r--','LineWidth',Eline);
hold on 
plot(PosTrack(:,1),'k-d','LineWidth',Tline);
% legend('Expected path','Actual path');
legend1=legend('\fontname{宋体}期望','\fontname{宋体}实际');
set(legend1,'FontSize',12,'FontWeight','normal')
xlim([0 50]);
grid on
hold on
    h_axis=get(h2,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    xlabel('Time','fontsize',16);  %x坐标  字体12 
    ylabel(['X','\rm/ m'] ,'fontsize',16);  %x坐标  字体12 
    set(gcf,'color','w');   % 背景白色
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    title(['\fontname{Times New Roman}X','\fontname{宋体}轴分量'],'fontsize',16);
    print('-dtiff','-r600',[num2str(2),'SVTrack','中文.tiff']);
    
h3 = figure(3);
plot(y(1:50),'r--','LineWidth',Eline);
hold on 
plot(PosTrack(:,2),'k-d','LineWidth',Tline);
% legend('Expected path','Actual path');
legend1=legend('\fontname{宋体}期望','\fontname{宋体}实际');
set(legend1,'FontSize',12,'FontWeight','normal')
axis([[0 50] [-0.7-0.0005 -0.7+0.0005]]);
hold on
grid on
    h_axis=get(h3,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    set(gcf,'color','w');   % 背景白色
    xlabel('Time','fontsize',16);  %x坐标  字体12 
    ylabel(['Y','\rm/ m'] ,'fontsize',16);  %x坐标  字体12 
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    title(['\fontname{Times New Roman}Y','\fontname{宋体}轴分量'],'fontsize',16);
    print('-dtiff','-r600',[num2str(3),'SVTrack','中文.tiff']);
    
h4 = figure(4);
plot(z(1:50),'r--','LineWidth',Eline);
hold on 
plot(PosTrack(:,3),'k-d','LineWidth',Tline);
% legend('Expected path','Actual path');
legend1=legend('\fontname{宋体}期望','\fontname{宋体}实际');
set(legend1,'FontSize',12,'FontWeight','normal')
xlim([0 50]);
hold on
grid on
    h_axis=get(h4,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    set(gcf,'color','w');   % 背景白色
    xlabel('Time','fontsize',16);  %x坐标  字体12 
    ylabel(['Z','\rm/ m'] ,'fontsize',16);  %x坐标  字体12 
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    title(['\fontname{Times New Roman}Z','\fontname{宋体}轴分量'],'fontsize',16);
    print('-dtiff','-r600',[num2str(4),'SVTrack','中文.tiff']);
    
h5 = figure(5);
error = Pos(1:50,:) - PosTrack;

plot(error,'LineWidth',2);
title('Track Error','fontsize',12);
xlabel('Time','fontsize',12);
ylabel('Error(m)','fontsize',12); 
legend('X-Direction','Y-Direction','Z-Direction');
    h_axis=get(h5,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %设置字体
    set(gcf,'color','w');   % 背景白色
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
% grid on
% print('-dtiff','-r600',['TrackError','SV','.tiff']);


Q = Qvector(:,:,1);
PosTrack1 = zeros(50,3);
for i=1:size(Q,1)
%     robot.plot(Q(i,:));
%     hold on
    T = robot.fkine(Q(i,:));
    PosTrack1(i,:) = transl(T);
%     plot3(PosTrack(1),PosTrack(2),PosTrack(3),'k-o');
%     hold on
end
Q = Qvector(:,:,2);
PosTrack2 = zeros(50,3);
for i=1:size(Q,1)
%     robot.plot(Q(i,:));
%     hold on
    T = robot.fkine(Q(i,:));
    PosTrack2(i,:) = transl(T);
%     plot3(PosTrack(1),PosTrack(2),PosTrack(3),'k-o');
%     hold on
end
Q = Qvector(:,:,4);
PosTrack3 = zeros(50,3);
for i=1:size(Q,1)
%     robot.plot(Q(i,:));
%     hold on
    T = robot.fkine(Q(i,:));
    PosTrack3(i,:) = transl(T);
%     plot3(PosTrack(1),PosTrack(2),PosTrack(3),'k-o');
%     hold on
end
ME1 = abs(Pos(1:50,:)-PosTrack1); % SCA
ME2 = abs(Pos(1:50,:)-PosTrack2); % CGSCA
ME3 = abs(error);                 % RES-SCA
ME4 = abs(Pos(1:50,:)-PosTrack3); % GS

Max(1,:) = max(ME1);
Max(2,:) = max(ME2);
Max(3,:) = max(ME3);
Max(4,:) = max(ME4);

Min(1,:) = min(ME1);
Min(2,:) = min(ME2);
Min(3,:) = min(ME3);
Min(4,:) = min(ME4);

MeanXYZ(1,:) = mean(ME1);
MeanXYZ(2,:) = mean(ME2);
MeanXYZ(3,:) = mean(ME3);
MeanXYZ(4,:) = mean(ME4);

Error(1,:) = sum(ME1,2);
Error(2,:) = sum(ME2,2);
Error(3,:) = sum(ME3,2);
Error(4,:) = sum(ME4,2);
% figure(6)
% plot(Error');
meanE(1) = sum(sum(ME1,2))/50;
meanE(2) = sum(sum(ME2,2))/50;
meanE(3) = sum(sum(ME3,2))/50;
meanE(4) = sum(sum(ME4,2))/50;