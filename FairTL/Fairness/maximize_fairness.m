function [alpha_max, fairness] = maximize_fairness(h_min, h_max, h_infeasible, fairness_func)
    a_infeasible = alpha_tuple(h_min, h_max, h_infeasible);
    
    % maximize(x) -> minimize(-x)
    fun = @(x)-fairness_func(x);
    
    % Start at edge of feasible region
    x0 = alpha_tuple(h_min, h_max, max(h_infeasible)); 
    
    % No linear constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % Alpha bounded by [0, 1]
    lb = zeros(1, length(h_min));
    ub = ones(1, length(h_min));
    
    function [c, ceq] = infeasible_constraints(a)
        c = [];
        ceq = double(~all(any(a>a_infeasible, 2)));
    end
    nonlcon = @infeasible_constraints;
    
    options = optimoptions('fmincon','Display','off','Algorithm','sqp');
    [x, fairness] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);
    
    % Convert alpha back to lengths, round up to get in feasible region
    alpha_max = dl_tuple(h_min, h_max, x);
    fairness = -fairness;
end


function a = alpha_tuple(h_min, h_max, dl)
    a = (dl - h_min)./(h_max - h_min);
end

function dl = dl_tuple(h_min, h_max, a)
    dl = ceil(a .* (h_max - h_min) + h_min);
end
