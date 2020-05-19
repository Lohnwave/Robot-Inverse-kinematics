% /******************************************************************************\
% * Copyright (C) 2020 . Luozu, All rights reserved.                            *
% * Version: 2.0																*
% * Last Revised: 2020-03-05													*
% * Eritor: Luozu																*
% \******************************************************************************/
clear;
close all;
clc;
load('ALL_Optimizer_IK0305.mat')

%% ---��������------
Fitness(1,:) = mean(error(1,:,:));
Fitness(2,:) = mean(error(2,:,:));
Fitness(3,:) = mean(error(3,:,:));
figure(4)
plot(Fitness(1,:),'r-o','LineWidth',2);
hold on 
plot(Fitness(2,:),'b-<','LineWidth',2);
hold on
plot(Fitness(3,:),'k-*','LineWidth',2);
legend1=legend('SCA','CGSCA','RES-SCA');
set(legend1,'FontSize',12,'FontWeight','normal')
h=figure(4);
    h_axis=get(h,'Children');
    set(h_axis,'LineWidth',1.5);
    set(gca,'FontSize',12, 'FontName','Times New Roman');  %��������
    set(gcf,'color','w');   % ������ɫ
    xlabel('\fontname{����}����','fontsize',16);
    ylabel(['\fontname{����}ƽ��','\fontname{Times New Roman}\Delta\theta \rm / rad'],'fontsize',16);
    % Rp = a + b.*exp(c.*(f).^-1);
%     title(['NSEI: ','\itRp','=','\ita','+','\itbe','^{c/f}'],'fontsize',20,'FontName','Times New Roman');  
    title('\fontname{����}��������','fontsize',16,'FontName','Times New Roman'); 
    %     hl=legend('test');   %���Ͻǵı�   
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4.49 3.37]);
    print('-dtiff','-r600',['Convergence_curve_SV','����.tiff']);

