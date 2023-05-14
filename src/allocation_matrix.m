
%--------------------------Thrust_allocation------------------------------%
function [thrust_left,thrust_right] = allocation_matrix(F_desired,torque_desired_L,torque_desired_M,torque_desired_N)    
    F_desired_max = 2*0.95*thrust_max;      %eq3.35
    if F_desired > F_desired_max
        F_desired = F_desired_max;
    end

    thrust_left = 0.5*F_desired + (1/(2*dis_l)) * torque_desired_N;         %eq3.36
    thrust_right = 0.5*F_desired - (1/(2*dis_l)) * torque_desired_N; 

    thrust_min = (1/2)*air_density*pi*rp*rp*(vs_min*vs_min - velocity_u*velocity_u);          %??????????
    if thrust_min < 0       %eq3.37
       thrust_min = 0;
    end

    if thrust_left < thrust_min
        thrust_left = thrust_min;
    end

    if thrust_right < thrust_min
        thrust_right = thrust_min;
    end

end

%--------------------------Servo_allocation-------------------------------%
function [servo_left,servo_right] = servo_allocation()      %??????????

    servo_allocation_matrix_A = [];     %??????????
    servo_allocation_matrix_b = [(torque_desired_L-(Ql-Qr)) , (torque_desired_M-M0)]';      %??????????
    servo_lr = inv(servo_allocation_matrix) * servo_allocation_matrix_b ;      

    servo_left = servo_lr(1);
    servo_right = servo_lr(2);

end

%-----------------------transfer to pwm signal----------------------------%
function [motor0,motor1,servo3,servo4] = transfer_to_pwm(thrust_left,thrust_right,servo_left,servo_right)


end