function [leg_1, leg_2, leg_3] = backwards_kinematics(px, py, pz, base_radius, motor_radius, arm_length)
    % Define the motor angles in degrees
    angle_motor_1 = 0;
    angle_motor_2 = 240;
    angle_motor_3 = 120;

    % Calculate the coordinates of the motors in the rotated frame
    motor_x_1 = px * cos(deg2rad(angle_motor_1)) + py * sin(deg2rad(angle_motor_1));
    motor_y_1 = py * cos(deg2rad(angle_motor_1)) - px * sin(deg2rad(angle_motor_1)) + motor_radius;
    motor_z_1 = pz;

    motor_x_2 = px * cos(deg2rad(angle_motor_2)) + py * sin(deg2rad(angle_motor_2));
    motor_y_2 = py * cos(deg2rad(angle_motor_2)) - px * sin(deg2rad(angle_motor_2)) + motor_radius;
    motor_z_2 = pz;

    motor_x_3 = px * cos(deg2rad(angle_motor_3)) + py * sin(deg2rad(angle_motor_3));
    motor_y_3 = py * cos(deg2rad(angle_motor_3)) - px * sin(deg2rad(angle_motor_3)) + motor_radius;
    motor_z_3 = pz;

    % Quadratic coefficients for each leg
    a_1 = 1;
    b_1 = -2 * motor_z_1 * sin(deg2rad(45)) - 2 * base_radius * cos(deg2rad(45)) + 2 * motor_y_1 * cos(deg2rad(45));
    c_1 = motor_x_1^2 + motor_y_1^2 + motor_z_1^2 + base_radius^2 - 2 * base_radius * motor_y_1 - arm_length^2;

    a_2 = 1;
    b_2 = -2 * motor_z_2 * sin(deg2rad(45)) - 2 * base_radius * cos(deg2rad(45)) + 2 * motor_y_2 * cos(deg2rad(45));
    c_2 = motor_x_2^2 + motor_y_2^2 + motor_z_2^2 + base_radius^2 - 2 * base_radius * motor_y_2 - arm_length^2;

    a_3 = 1;
    b_3 = -2 * motor_z_3 * sin(deg2rad(45)) - 2 * base_radius * cos(deg2rad(45)) + 2 * motor_y_3 * cos(deg2rad(45));
    c_3 = motor_x_3^2 + motor_y_3^2 + motor_z_3^2 + base_radius^2 - 2 * base_radius * motor_y_3 - arm_length^2;

    % Discriminants for the quadratic equations
    discriminant_1 = b_1^2 - 4 * a_1 * c_1;
    discriminant_2 = b_2^2 - 4 * a_2 * c_2;
    discriminant_3 = b_3^2 - 4 * a_3 * c_3;

    % Calculate the lengths of the legs
    leg_1 = (-b_1 - sqrt(discriminant_1)) / 2;
    leg_2 = (-b_2 - sqrt(discriminant_2)) / 2;
    leg_3 = (-b_3 - sqrt(discriminant_3)) / 2;
end
