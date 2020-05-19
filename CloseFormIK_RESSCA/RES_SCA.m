% /******************************************************************************\
% * Copyright (C) 2019 . Luozu, All rights reserved.                            *
% * 带连杆偏移的7自由度Barrett WAM 逆运动学封闭式解析解： 解析解优化器 RES_SCA        *
% * Version: 1.0																*
% * Last Revised: 2020-03-05													*
% * Eritor: Luozu																*
% \******************************************************************************/
function[gbest,phiGbest,allgbestval,flag] = RES_SCA(Tool_P,Tool_Orient,curQ,curPhi)
Max_iteration = 20;
N = 10;
lb = curPhi-3;
if lb < 0
    lb = 0;
end
ub = curPhi+3;
flag = 0;
D = 1;

w = 0.8;
allgbestval=zeros(1,Max_iteration);%%%
Max_FES = Max_iteration*N;
%Initialize the set of random solutions
Boundary_no= size(ub,2); % numnber of boundaries
% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    X=rand(N,D).*(ub-lb)+lb;
end
% If each variable has a different lb and ub
if Boundary_no>1
    for i=1:D
        ub_i=ub(i);
        lb_i=lb(i);
        X(:,i)=rand(N,1).*(ub_i-lb_i)+lb_i;
    end
end

Destination_position=zeros(1,D);
Destination_fitness=inf;

% Convergence_curve=zeros(1,Max_iteration);
Objective_values = zeros(1,size(X,1));
fitcount=0;
% Calculate the fitness of the first set and find the best one
for i=1:size(X,1)
%   fitness
    [solutionQ, sign] = CloseFormAK(Tool_P,Tool_Orient, X(i,:));
    if (sign==1)
        Objective_values(1,i) = sum(abs(curQ-solutionQ));
        flag = 1;
    else
        Objective_values(1,i) = inf;
    end
%
    fitcount=fitcount+1;
    if i==1
        Destination_position=X(i,:);
        Destination_fitness=Objective_values(1,i);
    elseif Objective_values(1,i)<Destination_fitness
        Destination_position=X(i,:);
        Destination_fitness=Objective_values(1,i);
    end
end
pbestVal = Objective_values;
pbestpos = X;
allgbestval(1,1)=Destination_fitness;
%Main loop
t=2; % start from the second iteration since the first iteration was dedicated to calculating the fitness
while t<=Max_iteration && fitcount<=Max_FES
    
    % Eq. (3.4)
    a = 1;

    r1=a-t*((a)/Max_iteration); % r1 decreases linearly from a to 0
    
    % Update the position of solutions with respect to destination
    X_temp = X;
    for i=1:size(X,1) % in i-th solution
        for j=1:size(X,2) % in j-th dimension
            
            % Update r2, r3, and r4 for Eq. (3.3)
            r2=(2*pi)*rand();
            r3=2*rand;
            r4=rand();
            if t < w*Max_iteration
                if r4<0.5
                % Eq. (3.1)
                    X_temp(i,j)= X_temp(i,j)+(r1*sin(r2)*abs(r3*(pbestpos(j)-X_temp(i,j))));
                else
                    % Eq. (3.2)
                    X_temp(i,j)= X_temp(i,j)+(r1*cos(r2)*abs(r3*(pbestpos(j)-X_temp(i,j))));
                end
            else
                if r4<0.5
                % Eq. (3.1)
                    X_temp(i,j)= X_temp(i,j)+(r1*sin(r2)*abs(r3*(Destination_position(j)-X_temp(i,j))));
                else
                    % Eq. (3.2)
                    X_temp(i,j)= X_temp(i,j)+(r1*cos(r2)*abs(r3*(Destination_position(j)-X_temp(i,j))));
                end
            end
        end
        rr1=0.1; %%%modification
		X_temp(i,:)=X_temp(i,:).*(1+rr1.*(t/Max_iteration).*randn+(1-t/Max_iteration).*trnd(1,1,1)); %%%%%%modification
        % Check if solutions go outside the search spaceand bring them back
        Flag4ub=X_temp(i,:)>ub;
        Flag4lb=X_temp(i,:)<lb;
        X_temp(i,:)=(X_temp(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;
        %   fitness
            [solutionQ, sign] = CloseFormAK(Tool_P,Tool_Orient, X_temp(i,:));
            if (sign==1)
                Objective_values(1,i) = sum(abs(curQ-solutionQ));
            else
                Objective_values(1,i) = inf;
            end
        %
        fitcount=fitcount+1;
        if Objective_values(1,i) < pbestVal(1,i)
            pbestVal(1,i) = Objective_values(1,i);
            X(i,:) = X_temp(i,:);
            pbestpos(i,:) = X_temp(i,:);
        end
        
    end
    
    %Update the destination if there is a better solution
    [gbestval,index]=min(pbestVal(1,:));
    if gbestval < Destination_fitness
        Destination_position=X(index,:);
        Destination_fitness=gbestval;
    end
     allgbestval(1,t)=Destination_fitness;
    % Increase the iteration counter
    if fitcount>=Max_FES
       break;  
    end
    if (t==Max_iteration)&&(fitcount<Max_FES)
       t=t-1;
    end 
    t=t+1;
end
allgbestval(1,Max_iteration)=Destination_fitness;
gbestval=Destination_fitness;
% gbest=Destination_position;
 [gbest,~] = CloseFormAK(Tool_P,Tool_Orient, Destination_position);
 phiGbest = Destination_position;
end