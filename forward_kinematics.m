function [px, py, pz] = forward_kinematics(leg_1, leg_2, leg_3, base_radius, motor_radius, arm_length)
    % Define the angle coefficient for calculations
    angle_coef = 1;
    
    % Calculate the intermediate values for each leg
    motor_y_1 = base_radius - leg_1 * cos(pi / 4) - motor_radius;
    motor_z_1 = leg_1 * sin(pi / 4);
    
    motor_y_2_dash = base_radius - leg_2 * cos(pi / 4) - motor_radius;
    motor_y_2 = -motor_y_2_dash * sin(deg2rad(30));
    motor_x_2 = motor_y_2_dash * cos(deg2rad(30));
    motor_z_2 = leg_2 * sin(pi / 4);
    
    motor_y_3_dash = base_radius - leg_3 * cos(pi / 4) - motor_radius;
    motor_y_3 = -motor_y_3_dash * sin(deg2rad(30));
    motor_x_3 = -motor_y_3_dash * cos(deg2rad(30));
    motor_z_3 = leg_3 * sin(pi / 4);
    
    motor_x_1 = 0;
    
    % Calculate squared distances for each leg
    dist_1 = motor_x_1^2 + motor_y_1^2 + motor_z_1^2;
    dist_2 = motor_x_2^2 + motor_y_2^2 + motor_z_2^2;
    dist_3 = motor_x_3^2 + motor_y_3^2 + motor_z_3^2;
    
    % Calculate intermediate alpha and beta values for solving the system of equations
    alpha_1 = (motor_x_3 * (dist_2 - dist_1) - motor_x_2 * (dist_3 - dist_1)) / 2;
    beta_1 = motor_x_2 * (motor_z_3 - motor_z_1) - motor_x_3 * (motor_z_2 - motor_z_1);
    det_d = motor_x_3 * (motor_y_2 - motor_y_1) - motor_x_2 * (motor_y_3 - motor_y_1);
    
    alpha_2 = ((motor_y_2 - motor_y_1) * (dist_3 - dist_1) - (motor_y_3 - motor_y_1) * (dist_2 - dist_1)) / 2;
    beta_2 = (motor_y_3 - motor_y_1) * (motor_z_2 - motor_z_1) - (motor_y_2 - motor_y_1) * (motor_z_3 - motor_z_1);
    
    % Coefficients for the quadratic equation
    quad_a = beta_2^2 + det_d^2 + beta_1^2;
    quad_b = 2 * (alpha_2 * beta_2 + alpha_1 * beta_1 - det_d^2 * motor_z_1 - motor_y_1 * det_d * beta_1);
    quad_c = alpha_2^2 + alpha_1^2 - 2 * motor_y_1 * det_d * alpha_1 - det_d^2 * arm_length^2 + det_d^2 * dist_1; 
    
    % Discriminant for the quadratic equation
    discriminant = quad_b^2 - 4 * quad_a * quad_c;
    
    % Solve for z
    pz = (-quad_b + sqrt(discriminant)) / (2 * quad_a);
    % Solve for y
    py = (alpha_1 + beta_1 * pz) / det_d;
    % Solve for x
    px = (alpha_2 + beta_2 * pz) / det_d;   
end
