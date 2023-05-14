%-----------------------attitude_controller-------------------------------%
function [torque_des_L, torque_des_M, torque_des_N] = attitude_controller(quaternion_des,quaternion_cur)
    global inertia_vec angular_vel kapl kadl kapm kadm kapn kadn last_quad_error dt euler euler_desired last_euler_error

    if norm(quaternion_cur - quaternion_des) <= norm(quaternion_cur + quaternion_des)   %eq3.31
        quad_error = quatmultiply( quatconj(quaternion_cur), quaternion_des);
    else
        quad_error = quatmultiply( quatconj(quaternion_cur),(-1)*quaternion_des);
    end

%     quad_error = quatmultiply(quaternion_des,quatconj(quaternion_cur));     %eq3.31
%     quad_error = sign(quad_error(1))*quad_error;

%     torque_des_L = inertia_vec(1) * (kapl*quad_error(2) - kadl*angular_vel(1));              %eq3.32
%     torque_des_M = inertia_vec(2) * (kapm*quad_error(3) - kadm*angular_vel(2));
%     torque_des_N = inertia_vec(3) * (kapn*quad_error(4) - kadn*angular_vel(3));
   
     
     torque_des_L = inertia_vec(1) * (kapl*quad_error(2) + kadl*(quad_error(2)-last_quad_error(2))/dt );              %eq3.32
     torque_des_M = inertia_vec(2) * (kapm*quad_error(3) + kadm*(quad_error(3)-last_quad_error(3))/dt );
     torque_des_N = inertia_vec(3) * (kapn*quad_error(4) + kadn*(quad_error(4)-last_quad_error(4))/dt );
     last_quad_error = quad_error;
    
%      if abs(euler(2)) <= deg2rad(20) 
%      euler_error = euler_desired - euler;
% 
%      torque_des_L = inertia_vec(1) * (kapl*euler_error(1) + kadl*(euler_error(1)-last_euler_error(1))/dt );              %eq3.32
%      torque_des_M = inertia_vec(2) * (kapm*euler_error(2) + kadm*(euler_error(2)-last_euler_error(2))/dt );
%      torque_des_N = inertia_vec(3) * (kapn*euler_error(3) + kadn*(euler_error(3)-last_euler_error(3))/dt );
%      last_euler_error = euler_error;
%      end

end
