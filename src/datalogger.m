function datalogger(time_cur,position_cur,position_ref,quaternion_cur,quaternion_des, ...
                    fx_desired,torque_des_L,torque_des_M,torque_des_N,bx_velocity_reference)
    global pos_2dot pos_dot data data_column_index force_vec rot_mat body_velocity body_lift_x body_lift_z ...
           body_drag_x body_drag_z aoa total_body_v V_windwake force_x_desired Cd Cl b_flift b_fdrag...
           define_center_trigger center_of_circle roll_ref heading_ref level_flight_aoa...
           correct_angle_x correct_angle_y correct_angle_z euler gamma angular_vel

    reshape_rot_mat = reshape(rot_mat,[1,9]);

    data(data_column_index,1) = time_cur;
    data(data_column_index,2:4) = position_cur;
    data(data_column_index,5:7) = position_ref;
    data(data_column_index,8:11) = quaternion_cur;
    data(data_column_index,12:15) = quaternion_des;
    data(data_column_index,16:18) = force_vec;
    data(data_column_index,19) = torque_des_L;
    data(data_column_index,20) = torque_des_M;
    data(data_column_index,21) = torque_des_N;
    data(data_column_index,22:24) = pos_2dot;
    data(data_column_index,25:27) = pos_dot;
    data(data_column_index,28) = bx_velocity_reference;
    data(data_column_index,29:37) = reshape_rot_mat;
    data(data_column_index,38:40) = body_velocity;
    data(data_column_index,41) = body_drag_x;
    data(data_column_index,42) = body_drag_z;
    data(data_column_index,43) = body_lift_x;
    data(data_column_index,44) = body_lift_z;
    data(data_column_index,45) = rad2deg(aoa);
    data(data_column_index,46:48) = total_body_v;
    data(data_column_index,49) = V_windwake;
    data(data_column_index,50) = force_x_desired;
    data(data_column_index,51) = Cd;
    data(data_column_index,52) = Cl;
    data(data_column_index,53) = b_flift;
    data(data_column_index,54) = b_fdrag;
    data(data_column_index,55) = define_center_trigger ;
    data(data_column_index,56:58) = center_of_circle;
    data(data_column_index,59) = rad2deg(roll_ref);
    data(data_column_index,60) = rad2deg(heading_ref);
    data(data_column_index,61) = rad2deg(level_flight_aoa);
    data(data_column_index,62) = rad2deg(correct_angle_x) ;
    data(data_column_index,63) = rad2deg(correct_angle_y) ;
    data(data_column_index,64) = rad2deg(correct_angle_z) ;
    data(data_column_index,65:67) = rad2deg(euler);
    data(data_column_index,68) = rad2deg(gamma);
    data(data_column_index,69:71) = rad2deg(angular_vel);
    data_column_index = data_column_index + 1;  
end