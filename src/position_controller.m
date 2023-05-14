%-----------------------position_controller-------------------------------%
%      unit quaternion??????????

function [quaternion_des] = position_controller(position_ref,position_cur,quaternion_ref,envelope_index)
    global dt euler kpp kpd kpi i_error max_correct_angle_z max_correct_angle_y last_error ...
            correct_angle_x correct_angle_y correct_angle_z euler_desired

    p_error = position_ref' - position_cur';        %remember transfer to 3*1 matrix
    d_error = (p_error-last_error)/dt;
    i_error = i_error + p_error*5*dt;

    correct_angle = quternion_to_rotation_matrix(quaternion_ref)' * (kpp.*p_error + kpd.*d_error + kpi.*i_error) ;%eq3.27 tranform matrix because we are using north east "up"
    correct_angle_z = correct_angle(2);
    correct_angle_y = (-1)*correct_angle(3);

    if correct_angle_z > deg2rad(max_correct_angle_z)
        correct_angle_z = deg2rad(max_correct_angle_z);
    end
    if correct_angle_z < -deg2rad(max_correct_angle_z)
        correct_angle_z = -deg2rad(max_correct_angle_z);
    end

    if correct_angle_y > deg2rad(20)
        correct_angle_y = deg2rad(20);
    end
    if correct_angle_y < -deg2rad(max_correct_angle_y)
        correct_angle_y = -deg2rad(max_correct_angle_y);
    end

%    correct_angle_x = (-1)*correct_angle_z * cos(abs(euler(2)))*cos(abs(euler(1)));            %eq3.28

     if( abs(euler(2)) < pi/4 && abs(euler(2)) > 0)    %if we have enought lift to bank turn
         correct_angle_x = (-1)*correct_angle_z * cos(abs(euler(2)))*cos(abs(euler(1)));            %eq3.28
     else
         correct_angle_x = 0;
     end
    qx = [cos((correct_angle_x/2)) , sin((correct_angle_x/2)) , 0 , 0  ];           %eq3.29 ,convert to rad for cos,sin calculate
    qy = [cos((correct_angle_y/2)) , 0 , sin((correct_angle_y/2)) , 0 ];
    qz = [cos((correct_angle_z/2)) , 0 , 0 , sin((correct_angle_z/2))  ];

    quatprod = quatmultiply(quaternion_ref,qz);                           %eq3.30
    quatprod = quatmultiply(quatprod,qy);
    quatprod = quatmultiply(quatprod,qx);
    quaternion_des = quatprod;
    last_error = p_error;
    
    eul_ref = quat2eul(quaternion_ref,'ZYX');
    euler_desired = [correct_angle_x, correct_angle_y, correct_angle_z] + eul_ref;
    

end
