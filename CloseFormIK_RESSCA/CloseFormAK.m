% /******************************************************************************\
% * Copyright (C) 2019 . Luozu, All rights reserved.                            *
% * 带连杆偏移的7自由度Barrett WAM 逆运动学封闭式解析解： 逆运动学求解器            *
% * Version: 1.0																*
% * Last Revised: 2020-03-04													*
% * Eritor: Luozu																*
% \******************************************************************************/
function [Q, sign] = CloseFormAK(Tool_P,Tool_Orient, phi)
% Tool_pos = [x, y, z]
% Orient: Quaterniond
    sign = 1;
    L1 = 0.55;
    L2 = 0.3;
    L3 = 0.06;
    L1offset = 0.045;
    L2offset = 0.045;
    DH_alpha = [ -pi / 2, pi / 2, -pi / 2, pi / 2, -pi / 2, pi / 2, 0 ];
	DH_a = [ 0, 0, 0.045, -0.045, 0, 0, 0 ];
	DH_d = [ 0, 0, 0.55, 0, 0.3, 0, 0.06 ];
	theta_L = [ -2.6, -2, -2.8, -0.9, -4.76, -1.6, -3 ];
	theta_U = [ 2.6, 2, 2.8, 3.1, 1.24, 1.6, 3 ];
    % 四元素转旋转矩阵
    Tool_Orient=quatnormalize(Tool_Orient); %单位化
    Tool_R=quat2dcm(Tool_Orient); % matlab 的四元素与Eigen的四元素转旋转矩阵有误
%     TRz = Tool_R(:,3);
    TRz = Tool_R(:,1)';
    % 腕部位置
    Pw = Tool_P - TRz*L3; % 腕部位置
    d = sqrt(dot(Pw,Pw)); % 腕部到基座距离
    % 考虑到连杆偏移 实际连杆
    NL1 = sqrt(L1^2+L1offset^2);
    NL2 = sqrt(L2^2+L2offset^2);
    
    % 通过结构判断距离d下是否有解
    if (d>(NL1+NL2))
        disp(['phi= ',num2str(phi),'无解']);
        sign = 0;
    else
        % L1,L2几何角度计算
        alpha1 = acos((d^2+NL1^2-NL2^2)/(2*d*NL1));
        alpha2 = acos((d^2+NL2^2-NL1^2)/(2*d*NL2));
        % 冗余圆
        Rc = NL1*sin(alpha1); % 冗余圆直径
        dc = NL1*cos(alpha1); % 冗余圆心到基座距离
        
        % 计算腕部与Z轴旋转矩阵R_w_z
        unitZ = [0,0,1]; 
        u = cross(unitZ,Pw/d)/norm(cross(unitZ,Pw/d));
        alpha = acos(dot(unitZ,Pw/d)/(norm(unitZ)*norm(Pw/d)));
        q = [cos(alpha/2), sin(alpha/2)*u(1), sin(alpha/2)*u(2), sin(alpha/2)*u(3)]; % 四元素
        q = quatnormalize(q);
        Rwz=quat2dcm(q);
        % 冗余圆
        C = [Rc*cos(phi), Rc*sin(phi), dc];
        % 根据腕部姿态旋转冗余圆
        C = C*Rwz;% ///////????
        E_C = C/norm(C);
        N_CW = cross(C, Pw);% 叉积求冗余圆与腕部的法向量
        E_N_CW = cross(C,N_CW)/norm(cross(C,N_CW));
        % 肘部上臂偏移点
        Pe1 = E_C*(L1^2/NL1)+(E_N_CW*(L1*L1offset/NL1));
        
        Q(1) = atan2(Pe1(2),Pe1(1));
        Q(2) = acos(Pe1(3)/L1);
        
        j3_up = cross(cross(Pe1,unitZ),Pe1);
        Q(3) = acos(dot(j3_up/norm(j3_up),(C-Pe1)/L1offset));
        if dot(Pe1,cross((C-Pe1)/L1offset,j3_up))>0
           Q(3) = -Q(3); 
        end
        Q(3) = Q(3) - pi;
        Q(4) = alpha2 + atan(L2offset/L2) + alpha1 + atan(L1offset/L1);
        
		T01 = DHmatrix(DH_alpha(1), DH_a(1), DH_d(1), Q(1));
		T12 = DHmatrix(DH_alpha(2), DH_a(2), DH_d(2), Q(2));
		T23 = DHmatrix(DH_alpha(3), DH_a(3), DH_d(3), Q(3));
		T34 = DHmatrix(DH_alpha(4), DH_a(4), DH_d(4), Q(4));
        
        T04 = T01 * T12 * T23 * T34;
        R04 = T04(1:3,1:3);
        R04 = transpose(R04); % equal A'
        
        P_tool = R04*(Tool_P-Pw)';
        
        Q(5) = atan2(P_tool(2), P_tool(1));
        Q(6) = pi/2 - atan2(P_tool(3),sqrt(P_tool(1)^2+P_tool(2)^2));
        
        T45 = DHmatrix(DH_alpha(5), DH_a(5), DH_d(5), Q(5));
        T56 = DHmatrix(DH_alpha(6), DH_a(6), DH_d(6), Q(6));
        T06 = T04 * T45 * T56;
        T06Rx = T06(1:3,1);
%         TRx = Tool_R(:,1);
        TRx = Tool_R(:,2);
        Q(7) = acos(dot(T06Rx,TRx));
        
        for i=1:7
            while(Q(i)<theta_L(i))
                Q(i) = Q(i) + 2*pi;
            end
            while(Q(i)>theta_U(i))
                Q(i) = Q(i) - 2*pi;
            end
        end
        % 边界检测
        if (sum(Q>theta_U)+sum(Q<theta_L))>0
            sign = 0;
%             disp(['phi= ',num2str(phi),'无解']);
        end
        
    end % end if
end