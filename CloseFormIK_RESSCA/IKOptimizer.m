% /******************************************************************************\
% * Copyright (C) 2020 . Luozu, All rights reserved.                            *
% * 带连杆偏移的7自由度Barrett WAM 逆运动学封闭式解析解： 解析解优化器              *
% * Version: 1.0																*
% * Last Revised: 2020-02-29													*
% * Eritor: Luozu																*
% \******************************************************************************/
function [Q,phiGbest,error,flag] = IKOptimizer(Tool_P,Tool_Orient, curQ, curPhi)
    minD = 1e10;
    flag = 0;
    Q = zeros(7);
    phiGbest = 0;
    for i = 1:360
%        disp(['度数= ', num2str(i)]);
       phi = i/180 * pi;
       [solutionQ, sign] = CloseFormAK(Tool_P,Tool_Orient, phi);
       if sign==1
           flag = 1;
           diff = 0;
           for j=1:7
               diff = diff + abs(curQ(j)-solutionQ(j));
           end
           if diff < minD
               Q = solutionQ;
               minD = diff;
               phiGbest = i;
           end
       end
%        error(i) = minD;
    end
    error = minD;
    disp(['diff= ', num2str(minD)]);
end