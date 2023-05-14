% CONTROL BLOCK DESIGN:
% ______________________________________________________________________________________________________________________________________
%                      position_ref...      error                 force,torque  
% [maneuver_generator]------->---------O------>-----[controller]------->--------[flapping_wing_6dof_dynamic]-------↘---->[datalogger]
%                                      ↑                                                                            ↓
%                                       ↖---------<----------------------------<-----------------------------<-----↙
%                                                  position_current,velocity_current,quaternion_current...etc
% ______________________________________________________________________________________________________________________________________

close all ; clear all ; clc;
global mass board_area gravity rho gravity_vec inertia_vec force_vec torque_vec
global dt total_time
global force_x_desired thrust_force_max      
global rot_mat euler euler_desired last_euler_error
global body_velocity pos_2dot pos_dot angular_vel
global kpp kpd kpi i_error last_error kapl kadl kapm kadm kapn kadn kup khp last_quad_error
global data data_column_index
global max_correct_angle_z max_correct_angle_y define_center_trigger center_of_circle roll_ref heading_ref level_flight_aoa
global correct_angle_x correct_angle_y correct_angle_z gamma

%________________________________parameters_______________________________________%
mass = 0.155;                                                % weight (kg)
board_area = 0.0742 + (34*11*2)/10000 ;                                       % 0.0462(m^2)
gravity = 9.81;                                             % gravity acceleration (m/s^2)
rho = 1.1691;                                               % air density (kg/m^3)
gravity_vec = [0;0;-mass*gravity];                          % gravity vector
inertia_vec = [0.003625 ; 0.000957 ; 0.003675];    % inertia vector [Ixx;Iyy;Izz]
dt = 0.005;                                                 % sec  (200hz update rate)
total_time = 50;                                            % sec
thrust_force_max = 2.1 ;                               % max thrust(N)
max_correct_angle_z = 15;                                   %degree
max_correct_angle_y = 25;                                   %degree
data_column_index = 1;
data = zeros(total_time/dt,15);

%________________________________initail state____________________________________%
force_vec = [0 0 0];
torque_vec = [0 0 0];
pos_2dot = [0 0 0];
body_velocity = [0 0 0];                    % velocity_current (body frame)
pos_dot = [0 0 0];                          % velocity_current (global frame)
angular_vel = [0 0 0];
euler = [0 -pi/2 0];                        % initail state when vertical take off (pitch 90 degree)
rot_mat = eul2rotm([euler(3),euler(2),euler(1)],'ZYX');            % initail rotation matrix
i_error = [0;0;0];
last_error = [0;0;0];
last_quad_error = [0 0 0 0];
define_center_trigger = 0;
center_of_circle = [0 0 0];
roll_ref = 0;
heading_ref = 0;
level_flight_aoa = 0;
correct_angle_x = 0;
correct_angle_y = 0;
correct_angle_z = 0;
gamma = 0;
euler_desired = [0 0 0];
last_euler_error = [0 0 0];
quaternion_current = eul2quat([euler(3),euler(2),euler(1)],'ZYX'); % quaternion initial
position_current = [0 0 0];                 % globalframe
maneuver_output_reference = zeros(1,8);
position_reference = zeros(1,3);
body_x_velocity_reference = zeros(1);
quaternion_reference = zeros(1,4);


%______________________________controller gain____________________________________%
%position controller gain           %attitude controller gain         %force controller gain 
   kpp = [0.6,0.6,0.6]';       kapl =60; kapm = 155; kapn = 45;        kup = 8; khp = 18; 
   kpi = [0.02,0.02,0.02]';    kadl = 20; kadm = 10; kadn = 10;  
   kpd = [0.02,0.02,0.02]';

%_________________________________main loop_______________________________________%
for current_time = 0:dt:total_time
    %generate desired trajectory from maneuver_generator
    if current_time <= 1
        env_index = 0;        % vertical take off
    elseif 1 < current_time && current_time <= 5
        env_index = 1;     % hover
    elseif 5 < current_time && current_time <= 6
        env_index = 2;      % transit to level flight
    elseif 6 < current_time && current_time <= 10
        env_index = 3;      % level flight
    elseif 10 < current_time && current_time <= 40
        env_index = 4;      % bank turn flight
%     elseif 40 < current_time && current_time <= 50
%         env_index = 5;      % transit back
%     elseif 50 < current_time && current_time <= 60
%         env_index = 1;      % hover     
%     elseif 60 < current_time && current_time <= 70
%         env_index = 6;     % landing
    elseif current_time >= total_time
        break;
    else
        break;
    end

    %generate position_refernce...etc
     maneuver_output_reference = maneuver_generator(current_time,position_current,env_index);
     position_reference        = maneuver_output_reference(1:3);
     body_x_velocity_reference = maneuver_output_reference(4);      %u_ref
     quaternion_reference      = maneuver_output_reference(5:8);

    %position_controller calculate quaternion_desired
    if mod(current_time,dt*5)==0        % 40hz update rate from optitrack feedback and 40hz update rate from barometor on naze32
     quaternion_desired = position_controller(position_reference, position_current, quaternion_reference,env_index);
    end
    %attitude_controller calculate torque_desired
     [torque_desired_L, torque_desired_M, torque_desired_N] = attitude_controller(quaternion_desired, quaternion_current);
    
    %Force_controller calculate body_fx_desired
     force_x_desired = Force_controller(position_reference, position_current, body_x_velocity_reference);

    %Bi_X_flapping_wing_model output position_current and quaternion_current
     model_output = Bi_X_flapping_wing_model(force_x_desired, torque_desired_L, torque_desired_M, ...
                                             torque_desired_N, position_current, quaternion_current);
     position_current =  model_output(1:3);
     quaternion_current = model_output(4:7);
    
    % store every data for plotter
     datalogger(current_time,position_current, position_reference, quaternion_current, quaternion_desired,force_x_desired, ...
                torque_desired_L, torque_desired_M, torque_desired_N,body_x_velocity_reference);
end

    %plot trajectory and attitude...etc
     plotter;       
