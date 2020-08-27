function R = R_from_euler(euler)
    % R = R_from_euler(euler)
    % generates a Rzyz rotation matrix from euler angles representation
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    Rz1 = [cos(phi) -sin(phi) 0
                sin(phi)  cos(phi) 0
                   0         0     1];
    Ry = [cos(theta) 0 sin(theta)
                     0        1      0
                  -sin(theta) 0 cos(theta)];
    Rz2 = [cos(psi) -sin(psi) 0
                sin(psi)  cos(psi) 0
                   0         0     1];
     R = Rz1 * Ry * Rz2;
end
       