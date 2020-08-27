function Rwr = generateRwr(ar, zb)
    % Rwr = generateRwr(ar, zb)
    % generates de Rwr rotation matrix
    psi = 0;
    wpsi = [cos(psi) sin(psi) 0]';
    zr = ar/norm(ar);
    xr = (wpsi - wpsi'*zb*zb); xr = xr/norm(xr);
    yr = cross(zr,xr);
    Rwr = [xr yr zr];
end
    