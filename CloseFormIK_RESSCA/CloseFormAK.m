% /******************************************************************************\
% * Copyright (C) 2019 . Luozu, All rights reserved.                            *
% * ������ƫ�Ƶ�7���ɶ�Barrett WAM ���˶�ѧ���ʽ�����⣺ ���˶�ѧ�����            *
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
    % ��Ԫ��ת��ת����
    Tool_Orient=quatnormalize(Tool_Orient); %��λ��
    Tool_R=quat2dcm(Tool_Orient); % matlab ����Ԫ����Eigen����Ԫ��ת��ת��������
%     TRz = Tool_R(:,3);
    TRz = Tool_R(:,1)';
    % ��λ��
    Pw = Tool_P - TRz*L3; % ��λ��
    d = sqrt(dot(Pw,Pw)); % �󲿵���������
    % ���ǵ�����ƫ�� ʵ������
    NL1 = sqrt(L1^2+L1offset^2);
    NL2 = sqrt(L2^2+L2offset^2);
    
    % ͨ���ṹ�жϾ���d���Ƿ��н�
    if (d>(NL1+NL2))
        disp(['phi= ',num2str(phi),'�޽�']);
        sign = 0;
    else
        % L1,L2���νǶȼ���
        alpha1 = acos((d^2+NL1^2-NL2^2)/(2*d*NL1));
        alpha2 = acos((d^2+NL2^2-NL1^2)/(2*d*NL2));
        % ����Բ
        Rc = NL1*sin(alpha1); % ����Բֱ��
        dc = NL1*cos(alpha1); % ����Բ�ĵ���������
        
        % ��������Z����ת����R_w_z
        unitZ = [0,0,1]; 
        u = cross(unitZ,Pw/d)/norm(cross(unitZ,Pw/d));
        alpha = acos(dot(unitZ,Pw/d)/(norm(unitZ)*norm(Pw/d)));
        q = [cos(alpha/2), sin(alpha/2)*u(1), sin(alpha/2)*u(2), sin(alpha/2)*u(3)]; % ��Ԫ��
        q = quatnormalize(q);
        Rwz=quat2dcm(q);
        % ����Բ
        C = [Rc*cos(phi), Rc*sin(phi), dc];
        % ��������̬��ת����Բ
        C = C*Rwz;% ///////????
        E_C = C/norm(C);
        N_CW = cross(C, Pw);% ���������Բ���󲿵ķ�����
        E_N_CW = cross(C,N_CW)/norm(cross(C,N_CW));
        % �ⲿ�ϱ�ƫ�Ƶ�
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
        % �߽���
        if (sum(Q>theta_U)+sum(Q<theta_L))>0
            sign = 0;
%             disp(['phi= ',num2str(phi),'�޽�']);
        end
        
    end % end if
end