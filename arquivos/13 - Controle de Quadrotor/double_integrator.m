function ad = double_integrator(F,v,p,kv,t)
    % ad = double_integrator(F,v,p,kv)
    % returns the second integrator model input ad
    % Inputs:
    % F is the Vector Field struct
    % v is the robot velocity
    % p is the robot position
    % kv > 0 is a gain
    % t is the simulation time
    JF = [F.dFdx(p,t) F.dFdy(p,t) F.dFdz(p,t)];
    ad = JF*v + kv*(F.F(p,t) - v) + F.dFdt(p,t);
end