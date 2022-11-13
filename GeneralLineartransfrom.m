function [coord_glo, coord_t] = GeneralLineartransfrom(x, y, z, vel, dt, theta_rad)
%     theta_rad = deg2rad(theta);
    lt_x = vel*cos(theta_rad)*dt;
    lt_y = vel*sin(theta_rad)*dt;
    lt_z = 0;
    R = [cos(theta_rad), -sin(theta_rad), 0;
         sin(theta_rad), cos(theta_rad), 0;
         0, 0, 1];
    T = [lt_x;lt_y;lt_z];
    mat = [R, T; 0, 0, 0, 1];
    
    coord_glo = mat*[x; y; z; 1];
    coord_t = T;
end
