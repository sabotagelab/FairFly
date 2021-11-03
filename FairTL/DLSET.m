classdef DLSET
    properties
        infeasibleDLsSmaller        = [];
        infeasibleDLsLessFair       = [];
        infeasibleAlphasLessFair    = [];
        initsize                    = 0;
        min_horizons                = 0;
        max_horizons                = 0;
        ndrones                     = 0;
        fairness_type               = 0;
    end
    
    methods
        function obj = DLSET(min_horizons, max_horizons, fairness_type)
            assert(length(min_horizons)==length(max_horizons));
            obj.initsize        = prod(max_horizons - min_horizons+1);
            obj.min_horizons    = min_horizons;
            obj.max_horizons    = max_horizons;
            obj.ndrones         = length(min_horizons);
            obj.fairness_type   = fairness_type; % 1-f1, 2-f2, 3-f_weighted
        end
        
        function infeasible = is_infeasible(obj,dl)
            i1 = any(all(obj.infeasibleDLsSmaller<=dl, 2));
            % dl is less fair than the feasible t if it equals t at those
            % elts that are less than average alpha, and greater than t at
            % those elts that exceed average alpha
            i2 = false;
            for k = 1:size(obj.infeasibleDLsLessFair,1)
                t   = obj.infeasibleDLsLessFair(k,:);
                at  = obj.alpha_tuple(t);
                ma  = mean(at);
                ixs = find(at < ma);
                ixl = find(at >= ma);
                i2  = all(dl(ixs) == t(ixs)) && all(dl(ixl) >= t(ixl));
                if i2
                    break
                end
            end
            % dl is infeasible if it is infeasible according to either
            % criterion
            infeasible = i1 || i2;
        end
        
        function s = size(obj)
            nbInfeasibles = sum(prod(obj.infeasibleDLsSmaller - obj.min_horizons+1, 2));
            s = obj.initsize - nbInfeasibles;
        end
        
        function nf = fairest(obj, prevFairest)
            if obj.fairness_type == 1
                fairness_func = @fairness1;
            elseif obj.fairness_type == 2
                fairness_func = @fairness2;
            elseif obj.fairness_type == 3
                fairness_func = @fairness_weighted;
            end
            
            if isempty(obj.infeasibleDLsSmaller) || isempty(prevFairest)
                %w = 0.25;
                %cvx_begin quiet
                %    variable a(obj.ndrones)
                %    minimize(w*1/(obj.ndrones-1)*norm(a-sum(a)/...
                %             obj.ndrones, 2) + (1-w)*norm(a));
                %cvx_end
                
                %nf = ceil(a' .*(obj.max_horizons - obj.min_horizons) + obj.min_horizons);
                
                % Above commented code results in just the minimum hrz (a=0)
                if obj.fairness_type == 1
                    nf = obj.max_horizons;
                elseif obj.fairness_type == 2
                    nf = obj.min_horizons;  
                elseif obj.fairness_type == 3
                    nf = obj.min_horizons;
                end
            else
                
                nf = maximize_fairness(obj.min_horizons,...
                                       obj.max_horizons,...
                                       obj.infeasibleDLsSmaller,...
                                       fairness_func);
                                   
                return
                
                %% Original selection that doesn't work fully
                % Pick next fairest
                % if pf is previous fairest (which is infeasible), then
                % next fairest is in
                % {v| pf - 1 < v <= pf + 1} \ Infeasible set so far
                % That's because fnt is convex over DL so:                
                % - no reason to move more than one step along any
                % direction: feasible directions don't increase fairness
                % since we are proceeding from fairest to least fair, and
                % those that decrease it, we just want to move among them
                % the least amount
                % - above covers all directions
                % This set can have 3^D -1  elements...
                %
                % Build C = {v| - allOnes < v <= + allOnes}
                % -allOnes is excluded since that will definitely be
                % infeasible being smaller than prevFairest
                % Because C is huge, we will actually sample it quick and
                % dirty,
                % subject to infeasibility constraints (i.e. we won't
                % consider points in it that are known to be infeasible)
                D       = obj.ndrones;
                ntries  = min(10000, 3^D - 1);
                C       = randi([-1,1], ntries, D);
                C       = C(sum(C,2)~=-D, :); % remove -1 vector
                % Remove repetitions
                C       = unique(C,'rows');
                
                % Offset prevFairest by C's
                check_set = C + prevFairest;
                % Remove any of check_set with indices outside horizon
                check_set = check_set(all(check_set>=obj.min_horizons, 2), :);
                check_set = check_set(all(check_set<=obj.max_horizons, 2), :);
                
                % Remove infeasibles - note we're not controlling for how
                % any DLs are left after this, hoping for the best
                ixfeas = ones(size(check_set,1),1);
                for k=1:size(check_set,1)
                    if obj.is_infeasible(check_set(k,:))
                        ixfeas(k) = 0;
                    end
                end
                check_set = check_set(ixfeas==1,:);
                % Pick fairest
                F = fairness2(check_set, obj.min_horizons, obj.max_horizons);
                [~, ixm] = max(F);
                nf = check_set(ixm,:);
            end
            % Sanity check
            assert(~isempty(nf) && all(nf>0), "Next fairest negative or empty");
        end
        
        function dl = kinda_random_tuple(obj)
            % Return a random length tuple for the purposes of elimination.
            % Might not be totally random, e.g. we can pick it half-way in
            % the DL set to eliminate a lot if it turns out infeasible.
            % Also, dl should be feasible, otherwise no point (I think...)
            dl = zeros(1,obj.ndrones);
            if ~isempty(obj.infeasibleDLsSmaller)
                % One way to make sure we get a feasible random tuple is to
                % pick one of its lengths, l_d, to be greater than the
                % corresponding largest infeasible length.
                maxInfeas = max(obj.infeasibleDLsSmaller, [], 1);
                % ff contains indices of lengths that can be larger than
                % the largest infeasible length yet within horizon max
                ff = find(obj.max_horizons > maxInfeas);
                if ~isempty(ff)
                    % pick one of these at random and set it to max+1
                    d = randi(length(ff));                    
                    dl(d) = maxInfeas(d) + 1;
                    % the rest can be anything between min and max horizons
                    for d=setdiff(1:obj.ndrones, d)
                        dl(d) = randi([obj.min_horizons(d), obj.max_horizons(d)]);
                    end
                else % revert to pure random
                    for d=1:obj.ndrones
                        dl(d) = randi([obj.min_horizons(d), obj.max_horizons(d)]);
                    end
                end
            else
                % This is pure random, no feasibility constraints
                for d=1:obj.ndrones
                    dl(d) = randi([obj.min_horizons(d), obj.max_horizons(d)]);
                end
            end
        end
        
        function a = alpha_tuple(obj,dl)
            a = (dl - obj.min_horizons)./(obj.max_horizons - obj.min_horizons);
        end
        
        function obj = remove_less_fair_dls(obj, dl, fvalue)
            % Declare DLs less fair than dl to be infeasible
            % dl
            %   DL tuple
            % fvalue
            %   fairness value
            obj.infeasibleDLsLessFair       = [obj.infeasibleDLsLessFair; dl];
            obj.infeasibleAlphasLessFair    = [obj.infeasibleAlphasLessFair; obj.alpha_tuple(dl)];
        end
        
        function obj = remove_smaller_dls(obj,dl)
            if ~isempty(obj.infeasibleDLsSmaller)
                keep = any(obj.infeasibleDLsSmaller>dl,2);
                obj.infeasibleDLsSmaller = obj.infeasibleDLsSmaller(keep, :);
            end
            obj.infeasibleDLsSmaller = [obj.infeasibleDLsSmaller; dl];
        end
        
        function fairness = get_fairness_val(obj,dl)
           	if obj.fairness_type == 1
                fairness_func = @fairness1;
            elseif obj.fairness_type == 2
                fairness_func = @fairness2;
            elseif obj.fairness_type == 3
                fairness_func = @fairness_weighted;
            end
            fairness = fairness_func(dl, obj.min_horizons, obj.max_horizons);
        end
        
    end
end


