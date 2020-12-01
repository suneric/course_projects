% make sure M is in radian, the output E will be in radian too
function E=Kepler_Equation(M, e)
E = Solver_1(M,e,10e-8);
%E = Solver_2(M,e);
end

%Algorithm 3.1 on the book
function E=Solver_1(M,e,tol)
% choose an initial estimate of the root E
if M < pi
   E = M+0.5*e;
else
   E = M-0.5*e;
end
% interatively, at any step, 
for i=1:10000
    % compute Ei with f(Ei) and f'(Ei)
    f_E = E - e*sin(E)-M;
    f_E_d = 1 - e*cos(E);
    % computer ratio of f(Ei)/f'(Ei)
    ratio = f_E/f_E_d;
    % update E
    if abs(ratio) > tol
        E = E-ratio;
    else
        break
    end
end      
end

% by using fzero
function E=Solver_2(M,e)
kepler_eq = @(E) E-e*sin(E)-M;
if M < pi
   E = M+0.5*e;
else
   E = M-0.5*e;
end
% pass the initial guess to fzero
E = fzero(kepler_eq,E);
end