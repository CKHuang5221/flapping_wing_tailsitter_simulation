%%Bi X Wing flapping wing model
function  model_out = Bi_X_flapping_wing_model(f_x_desired, torque_des_L, torque_des_M, torque_des_N, position_cur,quaternion_cur)

global euler body_velocity pos_2dot pos_dot angular_vel force_vec torque_vec rot_mat inv_rot_mat
global mass gravity gravity_vec inertia_vec
global dt

% rotation matrix
    rot_mat = quat2rotm(quaternion_cur);
                         
% inverse rotation matrix
    inv_rot_mat = rot_mat';

% translation matrix
    trans_mat = [ 1  sin(euler(1))*tan(euler(2))  cos(euler(1))*tan(euler(2));
                  0                cos(euler(1))               -sin(euler(1));
                  0  sin(euler(1))/cos(euler(2))  cos(euler(1))/cos(euler(2))];

%calculate lift and drag    
    cal_wind_wake(f_x_desired);
    [body_x_drag, body_z_drag, body_x_lift, body_z_lift] = board_lift_and_drag;

% set force
    force_vec(1) = f_x_desired ;   %+ body_x_lift + body_x_drag ;
    force_vec(2) = 0;
    force_vec(3) = body_z_lift + body_z_drag ;
% set torque
    torque_vec(1) = torque_des_L;
    torque_vec(2) = torque_des_M;
    torque_vec(3) = torque_des_N;

% position eqaution
    pos_2dot = ((rot_mat * force_vec' + gravity_vec)/mass)';       % remember transfer force_vec to 1*3 matrix

% euler method of differential equation
    pos_dot = pos_dot + pos_2dot * dt;      %1*3 matrix
    
% velocity in body frame
    body_velocity = (inv_rot_mat*pos_dot')';        %1*3 matrix

% euler method of differential equation
    pos_cur = position_cur + pos_dot * dt;      %1*3 matrix

%_______________________________________rotation_________________________________________________%
    
% anguler velocity equation
%    angular_vel_dot(1) = ((inertia_vec(2)-inertia_vec(3))*angular_vel(2)*angular_vel(3) + torque_vec(1))/inertia_vec(1);
%    angular_vel_dot(2) = ((inertia_vec(3)-inertia_vec(1))*angular_vel(1)*angular_vel(3) + torque_vec(2))/inertia_vec(2);
%    angular_vel_dot(3) = ((inertia_vec(1)-inertia_vec(2))*angular_vel(1)*angular_vel(2) + torque_vec(3))/inertia_vec(3);
    angular_vel_dot(1) = (torque_vec(1))/inertia_vec(1);
    angular_vel_dot(2) = (torque_vec(2))/inertia_vec(2);
    angular_vel_dot(3) = (torque_vec(3))/inertia_vec(3);

% euler method of differential equation
    angular_vel = angular_vel + angular_vel_dot * dt;       %1*3 matrix

% quaternion differentail equation:
    quat_cur = quaternion_cur;

    %store current quaternion in buffer
    buffer_quat_cur(1:4) = quat_cur(1:4); 

    % calculate new quaternion from quaternion buffer
    quat_cur(1) = buffer_quat_cur(1) + (1/2)*dt*( -angular_vel(1)*buffer_quat_cur(2) -angular_vel(2)*buffer_quat_cur(3) -angular_vel(3)*buffer_quat_cur(4) );
    quat_cur(2) = buffer_quat_cur(2) + (1/2)*dt*(  angular_vel(1)*buffer_quat_cur(1) -angular_vel(2)*buffer_quat_cur(4) +angular_vel(3)*buffer_quat_cur(3) );
    quat_cur(3) = buffer_quat_cur(3) + (1/2)*dt*(  angular_vel(1)*buffer_quat_cur(4) +angular_vel(2)*buffer_quat_cur(1) -angular_vel(3)*buffer_quat_cur(2) );
    quat_cur(4) = buffer_quat_cur(4) + (1/2)*dt*( -angular_vel(1)*buffer_quat_cur(3) +angular_vel(2)*buffer_quat_cur(2) +angular_vel(3)*buffer_quat_cur(1) );
    
% augular velocity to euler angle
    euler_dot = (trans_mat * angular_vel')';       % remember transfer back to 1*3 matrix
    
% euler method of differential equation
    euler = euler + euler_dot * dt;            % 1*3 matrix

    if euler(1)>2*pi
        euler(1) = euler(1)-2*pi;
    end
    if euler(2)>2*pi
        euler(2) = euler(2)-2*pi;
    end
    if euler(3)>2*pi
        euler(3) = euler(3)-2*pi;
    end

    if euler(1)<-2*pi
        euler(1) = euler(1)+2*pi;
    end
    if euler(2)<-2*pi
        euler(2) = euler(2)+2*pi;
    end
    if euler(3)<-2*pi
        euler(3) = euler(3)+2*pi;
    end


% output
    model_out = [pos_cur,quat_cur];        %1*7 matrix
end
