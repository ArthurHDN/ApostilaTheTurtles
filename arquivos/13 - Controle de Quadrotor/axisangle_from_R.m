function [n, B] = axisangle_from_R(R)
    % [n, B] = axisangle_from_R(R)
    % returns the axis/angle representation using Euler's rotation theorem
    % when given a rotation matrix as input
    B = acos((trace(R)-1)/2);
    n = (1/(2*sin(B)))* [R(3,2)-R(2,3); R(1,3)-R(3,1);R(2,1)-R(1,2)];
    if B == 0 % When B == 0, n is undefined
        n = [0;0;0]
    end
end