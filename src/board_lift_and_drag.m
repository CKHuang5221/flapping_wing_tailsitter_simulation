function [b_fdx, b_fdz, b_flx, b_flz] = board_lift_and_drag
global board_area rho inv_rot_mat body_velocity body_lift_x body_lift_z aoa total_body_v b_flift b_fdrag body_drag_x body_drag_z ...
       V_windwake Cd Cl

total_body_v = body_velocity + [V_windwake 0 0];
Vu =  total_body_v(1);       %body_velocity_x 
Vv =  total_body_v(2);       %body_velocity_y
Vw =  total_body_v(3);       %body_velocity_z
Vuw = sqrt(Vu^2 + Vw^2);

aoa = atan2(Vw,Vu);     %rad

[Cd,Cl] = cal_CdCl(aoa);

b_flift = (1/2) * rho * Vuw^2 * board_area * Cl;
b_fdrag = (1/2) * rho * Vuw^2 * board_area * Cd;

b_fdx = (-1) * b_fdrag * cos(aoa);
b_flx = b_flift*sin(aoa);

b_flz = (-1) * b_flift*cos(aoa);
b_fdz = (-1) * b_fdrag * sin(aoa);


body_lift_x = b_flx ;
body_lift_z = b_flz ;
body_drag_x = b_fdx;
body_drag_z = b_fdz;

end

function [coefficient_D, coefficient_L] = cal_CdCl(attack_angle)
coefficient_D = 0.9566 * cos(2.219 * attack_angle - 3.486) + 1.444;  %rad
coefficient_L = 1.259 * sin(1.328*attack_angle + 0.5004); %rad
end