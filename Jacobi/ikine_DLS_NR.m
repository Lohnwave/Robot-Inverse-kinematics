% A Synthetic Inverse Kinematic Algorithm for 7-DOF Redundant Manipulator
% August 1-5, 2018, Conference
% Code editor: LuoZu
% Time: 2019-11-25
function qt = ikine_DLS_NR(robot, tr, varargin)
    
    n = robot.n;
    
    TT = SE3.check(tr);
        
    %  set default parameters for solution
    opt.ilimit = 500;
    opt.rlimit = 100;
    opt.slimit = 100;
    opt.tol = 1e-10;
    opt.lambda = 0.05;% update from 0.1 by luozu 0713
    opt.lambdamin = 0;
    opt.search = false;
    opt.quiet = false;
    opt.verbose = false;
    opt.mask = [1 1 1 1 1 1];
    opt.q0 = zeros(1, n);
    opt.transpose = NaN;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    assert(numel(opt.mask) == 6, 'RTB:ikine:badarg', 'Mask matrix should have 6 elements');
    assert(n >= numel(find(opt.mask)), 'RTB:ikine:badarg', 'Number of robot DOF must be >= the same number of 1s in the mask matrix');
    W = diag(opt.mask); % 行矩阵对角化
    
    qt = zeros(length(TT), n);  % preallocate space for results
    tcount = 0;              % total iteration count
    rejcount = 0;            % rejected step count
    
    q = opt.q0;
    
    failed = false;
    revolutes = robot.isrevolute();
    
    for i=1:length(TT) %1：1
        T = TT(i); % T = TT
        lambda = 0.01; % damping factor in paper is 0.1

        iterations = 0;
        
        if opt.debug
            e = tr2delta(robot.fkine(q), T);
            fprintf('Initial:  |e|=%g\n', norm(W*e));
        end
        
        while true
            % update the count and test against iteration limit
            iterations = iterations + 1;
            % 迭代终止条件1：迭代超时
            if iterations > opt.ilimit
                if ~opt.quiet
                    warning('ikine: iteration limit %d exceeded (pose %d), final err %g', ...
                        opt.ilimit, i, nm);
                end
                failed = true;
                break
            end
            
            e = tr2delta(robot.fkine(q), T); % tr2delta将齐次变换转换为差分运动 e=(dx, dy, dz, dRx, dRy, dRz).
            
            % are we there yet
            % 迭代终止条件2：误差达到阈值
            if norm(W*e) < opt.tol % tol = 1e-10 norm: 求范数
                iterations
                break;
            end
            
            % compute the Jacobian
            J = jacobe(robot, q); % J 6x7
            % 伪逆 J* = J'*inv(J*J')
            JtJ = J'*W*J;
            if ~isnan(opt.transpose)
                % do the simple Jacobian transpose with constant gain 用恒定增益进行简单的雅可比移调
                dq = opt.transpose * J' * e;
            else
                % do the damped least-squares with Newton-Raphson 
                % inv() 矩阵求逆; eye(N) 生成N*N的单位矩阵
                dq = inv(JtJ + lambda * eye(size(JtJ)) ) * J' * W * e;
                
                % compute possible new value of
                qnew = q + dq';
                
                % and figure out the new error
                enew = tr2delta(robot.fkine(qnew), T);
                
                % was it a good update?
                if norm(W*enew) < norm(W*e)
                    % step is accepted
                    if opt.debug
                        fprintf('ACCEPTED: |e|=%g, |dq|=%g, lambda=%g\n', norm(W*enew), norm(dq), lambda);
                    end
                    q = qnew;
                    e = enew;
%                     lambda = lambda/2;
                    rejcount = 0;
                else
                    % step is rejected, increase the damping and retry
                    if opt.debug
                        fprintf('rejected: |e|=%g, |dq|=%g, lambda=%g\n', norm(W*enew), norm(dq), lambda);
                    end
%                     lambda = lambda*2;
                    rejcount = rejcount + 1;
                    if rejcount > opt.rlimit
                        if ~opt.quiet
                            warning('ikine: rejected-step limit %d exceeded (pose %d), final err %g', ...
                                opt.rlimit, i, norm(W*enew));
                        end
                        failed = true;
                        break;
                    end
                    continue;  % try again
                end
            end
            
            % wrap angles for revolute joints
            k = (q > pi) & revolutes;
            q(k) = q(k) - 2*pi;
            
            k = (q < -pi) & revolutes;
            q(k) = q(k) + 2*pi;
            
            nm = norm(W*e);  
            
        end  % end ikine solution for this pose
        qt(i,:) = q';
        tcount = tcount + iterations;
        if opt.verbose && ~failed
            fprintf('%d iterations\n', iterations);
        end
        if failed
            if ~opt.quiet
                warning('failed to converge: try a different initial value of joint coordinates');
            end
            qt = [];
        end
    end
     
    if opt.verbose && length(TT) > 1
        fprintf('TOTAL %d iterations\n', tcount);
    end
end
