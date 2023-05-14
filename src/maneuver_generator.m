% u_ref is forward flight speed
function modeloutput = maneuver_generator(cur_t, p_current ,envelope_index)

    global dt total_time rot_mat b_flift b_fdrag mass gravity euler pos_dot define_center_trigger center_of_circle roll_ref heading_ref...
            level_flight_aoa gamma target_bearing
    bankturn_radius = 2;
    level_flight_V = 3; %sqrt(pos_dot(1)^2 + pos_dot(2)^2);
    level_flight_aoa = 1.3*exp(-0.2*level_flight_V);
    cl_alpha = 2.9112 ;     % from level_fly_aoa_fitting.m
    rho = 1.196;
    S = 0.0462; %(m^2)

    switch envelope_index
        case 0      %vertical take off
            px_ref = 0;
            py_ref = 0;
            pz_ref = 1;
            u_ref  = 0.5;
            quaternion_ref = eul2quat([0 -pi/2 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);

        case 1      %hover
            px_ref = 0;
            py_ref = 0;
            pz_ref = 1;
            u_ref = 0;
            quaternion_ref = eul2quat([0 -pi/2 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);

        case 2      %transit to level flight            
            px_ref = p_current(1);
            py_ref = 0;
            pz_ref = 1;
            u_ref = 1;
            target_bearing = atan2(py_ref-p_current(2),px_ref-p_current(1));
            quaternion_ref = eul2quat([0 -pi/4 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);

        case 3      %level flight
            px_ref = p_current(1);
            py_ref = 0;
            pz_ref = 1;
            u_ref = level_flight_V*cos(level_flight_aoa);
            quaternion_ref = eul2quat([0 level_flight_aoa*(-1) 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);
    
        case 4      %bank turn flight
%             level_flight_V = 2.7;
%             level_flight_aoa = 1.3*exp(-0.2*level_flight_V);
            direction = 1;  %1 for left bank turn, -1 for right bank turn
            
            % define center of circle
            if define_center_trigger == 0        
                center_of_circle = p_current + bankturn_radius*[cos(0 + direction*(pi/2)), sin(0 + direction*(pi/2)), 0] ;
                define_center_trigger = 1;
            end
            
            % define desired heading
            if p_current(2)-center_of_circle(2) >= 0  % quadrant I and II
                gamma =  atan2(p_current(2)-center_of_circle(2), p_current(1)-center_of_circle(1) );
            else                                      % quadrant III and IV
                gamma =  atan2(p_current(2)-center_of_circle(2), p_current(1)-center_of_circle(1) ) + 2*pi ;
            end
            heading_ref = gamma + direction*(pi/2);
            if heading_ref > 2*pi
                heading_ref = heading_ref-2*pi;
            end

            % define desired roll
            ref = (2*mass)/(bankturn_radius*rho*S*cl_alpha*abs(euler(2)));
            if ref>1
                ref = 1;
            end
            roll_ref = real(asin( ref ));    %rad
            roll_ref_max = deg2rad(20);
            if abs(roll_ref) > roll_ref_max
                roll_ref = roll_ref_max;
            end
            roll_ref = roll_ref*direction*(-1);

            px_ref = center_of_circle(1) + bankturn_radius*cos(gamma);
            py_ref = center_of_circle(2) + bankturn_radius*sin(gamma);
            pz_ref = 1;
            u_ref = level_flight_V*cos(level_flight_aoa);
            quaternion_ref = eul2quat([ heading_ref, level_flight_aoa*(-1), roll_ref],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);

        case 5      %transit back
            px_ref = p_current(1);
            py_ref = p_current(2);
            pz_ref = 1;
            u_ref = 0;
            quaternion_ref = eul2quat([0 (pi/2)*(-1) 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);

        case 6      %landing
            px_ref = p_current(1);
            py_ref = p_current(2);
            pz_ref = 0;
            u_ref = 0;
            quaternion_ref = eul2quat([0 (pi/2)*(-1) 0],'ZYX');
            qw_ref = quaternion_ref(1);
            qx_ref = quaternion_ref(2);
            qy_ref = quaternion_ref(3);
            qz_ref = quaternion_ref(4);
        otherwise
    end

    %output position ref...etc
    modeloutput(1) = px_ref;
    modeloutput(2) = py_ref ;
    modeloutput(3) = pz_ref ;
    modeloutput(4) = u_ref ;
    modeloutput(5) = qw_ref ;
    modeloutput(6) = qx_ref ; 
    modeloutput(7) = qy_ref ; 
    modeloutput(8) = qz_ref ;
end
