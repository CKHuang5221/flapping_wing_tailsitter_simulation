%--------------------------Thrust_controller-------------------------------%
function fx_des = Force_controller(position_ref,position_cur,body_x_velocity_ref)
    global mass gravity euler kup khp thrust_force_max body_velocity

    fx_des =   mass*gravity*sin(euler(2))*(-1) ...
             + mass*kup*(body_x_velocity_ref - body_velocity(1))...    
             + mass*khp* (position_ref(3) - position_cur(3) )*sin( euler(2) )*(-1);  %eq3.34

    if fx_des < 0
        fx_des = 0;
    end
    if fx_des > thrust_force_max
        fx_des = thrust_force_max;
    end
end
