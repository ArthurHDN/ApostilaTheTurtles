function F = generateFieldEquations(type,p_goal)
    % F = generateFieldEquations(type,p_goal)
    % returns an atractive vector field to p_goal. F will be a struct with
    % F.F: the field function handle
    % F.dFdx: the x partial derivative function handle
    % F.dFdy: the y partial derivative function handle
    % F.dFdz: the z partial derivative function handle
    % F.dFdt: the t partial derivative function handle
    d = 1; c = 1;
    switch type
        case 1
            F.F = @(p,t) (norm(p-p_goal)<=d).*(-c*(p-p_goal)) +...
            (norm(p-p_goal)>d).*(-d*c*(p-p_goal)/(norm(p-p_goal)+1e-6));
            
            F.dFdx = @(p,t) [-c;0;0];
            F.dFdy = @(p,t) [0;-c;0];
            F.dFdz = @(p,t) [0;0;-c];
            F.dFdt = @(p,t) [0;0;0];
        case 2
            A = [-1 0.1 0; -0.1 -1 0; 0 0 -1];
            F.F = @(p,t) (c*A*(p-p_goal));
            F.dFdx = @(p,t) c*[-1;-0.1;0];
            F.dFdy = @(p,t) c*[0.1;-1;0];
            F.dFdz = @(p,t) c*[0;0;-1];
            F.dFdt = @(p,t) c*[0;0;0];
        case 3
            A = [-1 0.4 0; -0.4 -1 0; 0 0 -1];
            F.F = @(p,t) (c*A*(p-p_goal));
            F.dFdx = @(p,t) c*[-1;-0.4;0];
            F.dFdy = @(p,t) c*[0.4;-1;0];
            F.dFdz = @(p,t) c*[0;0;-1];
            F.dFdt = @(p,t) c*[0;0;0];
end