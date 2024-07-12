% Define the starting, via, and end points for the trajectory
initial_point = [-60, 40, 335];
intermediate_point = [-40, -90, 335];
final_point = [-40, -90, 360];

% Adjust the z-coordinate of each point
initial_point(3) = initial_point(3) + 70;
intermediate_point(3) = intermediate_point(3) + 70;
final_point(3) = final_point(3) + 70;

% Calculate the joint angles for the initial point using backwards kinematics
[joint1_init, joint2_init, joint3_init] = backwards_kinematics(initial_point(1), initial_point(2), initial_point(3), const_r_b, const_r_m, const_l);
initial_joint_positions = [joint1_init, joint2_init, joint3_init];

% Define the end times and velocities for the trajectory
end_times = [1000000, 1000000, 1000000];
end_velocities = [0, 0, 0];

% Define the maximum velocities and accelerations
max_velocities = [3, 7, 3];
max_accelerations = [1, 1, 1];

% Generate the linear trajectory from the initial point to the intermediate point
[linear_times1, linear_velocities1] = moveL(initial_point, intermediate_point, max_velocities, max_accelerations);
% Generate the linear trajectory from the intermediate point to the final point
[linear_times2, linear_velocities2] = moveL(intermediate_point, final_point, max_velocities, max_accelerations);

% Adjust the timing for the second linear trajectory
linear_times2 = linear_times2 + linear_times1(1, 5) + 0.00001;

% Concatenate the times and velocities for the linear trajectories
linear_times = [linear_times1, linear_times2, [1000000, 1000000, 1000000]'];
linear_velocities = [linear_velocities1, linear_velocities2, [0, 0, 0]'];

% Extract the times and velocities for each coordinate axis
time_x = linear_times(1, :);
time_y = linear_times(2, :);
time_z = linear_times(3, :);

velocity_x = linear_velocities(1, :);
velocity_y = linear_velocities(2, :);
velocity_z = linear_velocities(3, :);

% Generate the joint trajectory from the initial point to the intermediate point
[joint_times1, joint_velocities1] = moveJ(initial_point, intermediate_point, max_velocities, max_accelerations, const_l, const_r_b, const_r_m);
% Generate the joint trajectory from the intermediate point to the final point
[joint_times2, joint_velocities2] = moveJ(intermediate_point, final_point, max_velocities, max_accelerations, const_l, const_r_b, const_r_m);

% Adjust the timing for the second joint trajectory
joint_times2 = joint_times2 + joint_times1(1, 5) + 0.01;

% Concatenate the times and velocities for the joint trajectories
joint_times = [joint_times1, joint_times2, [1000000, 1000000, 1000000]'];
joint_velocities = [joint_velocities1, joint_velocities2, [0, 0, 0]'];

% Extract the times and velocities for each joint
time_joint1 = joint_times(1, :);
time_joint2 = joint_times(2, :);
time_joint3 = joint_times(3, :);

velocity_joint1 = joint_velocities(1, :);
velocity_joint2 = joint_velocities(2, :);
velocity_joint3 = joint_velocities(3, :);

% MoveJ function: Generate joint space trajectory
function [times, velocities] = moveJ(start_point, end_point, max_v, max_a, const_l, const_r_b, const_r_m)
    [joint1_start, joint2_start, joint3_start] = backwards_kinematics(start_point(1), start_point(2), start_point(3), const_r_b, const_r_m, const_l);
    start_joint_positions = [joint1_start, joint2_start, joint3_start];
    [joint1_end, joint2_end, joint3_end] = backwards_kinematics(end_point(1), end_point(2), end_point(3), const_r_b, const_r_m, const_l);
    end_joint_positions = [joint1_end, joint2_end, joint3_end];
    [times, velocities] = trajectory_gen(start_joint_positions, end_joint_positions, max_v, max_a);
end

% MoveL function: Generate linear space trajectory
function [times, velocities] = moveL(start_point, end_point, max_v, max_a)
    [times, velocities] = trajectory_gen(start_point, end_point, max_v, max_a);
end

% Trajectory generation function: Calculate times and velocities for a trajectory
function [times, velocities] = trajectory_gen(start_point, end_point, max_v, max_a)
    local_max_v = max_v;
    local_max_a = max_a;

    % Calculate the displacement vector
    displacement = end_point - start_point;

    % Adjust max velocities and accelerations based on displacement direction
    for i = 1:3
        if displacement(i) < 0
            local_max_v(i) = -local_max_v(i);
            local_max_a(i) = -local_max_a(i);
        end 
    end

    % Calculate acceleration and total time for each axis
    for i = 1:3
        if abs(displacement(i)) >= abs(local_max_v(i)^2 / local_max_a(i))
            Ta(i) = local_max_v(i) / local_max_a(i);
            T(i) = (displacement(i) * local_max_a(i) + local_max_v(i)^2) / (local_max_a(i) * local_max_v(i));
        else 
            Ta(i) = sqrt(displacement(i) / local_max_a(i));
            T(i) = 2 * Ta(i);
        end
    end

    % Calculate the maximum acceleration and total time
    max_acceleration_time = max(Ta);
    max_total_time = max(T);

    % Recalculate max accelerations and velocities based on max times
    local_max_a = displacement / (max_acceleration_time * (max_total_time - max_acceleration_time));
    local_max_v = displacement / (max_total_time - max_acceleration_time);

    % Define the end time for the trajectory
    end_time = max_total_time + 0.01;

    % Generate the time and velocity profiles for x, y, and z axes
    traj_x_time = [0, max_acceleration_time + 0.00001, max_total_time - max_acceleration_time + 0.00002, max_total_time + 0.00003, end_time];
    traj_x_velo = [0, local_max_v(1), local_max_v(1), 0, 0];

    traj_y_time = [0, max_acceleration_time + 0.00001, max_total_time - max_acceleration_time + 0.00002, max_total_time + 0.00003, end_time];
    traj_y_velo = [0, local_max_v(2), local_max_v(2), 0, 0];

    traj_z_time = [0, max_acceleration_time + 0.00001, max_total_time - max_acceleration_time + 0.00002, max_total_time + 0.00003, end_time];
    traj_z_velo = [0, local_max_v(3), local_max_v(3), 0, 0];

    % Concatenate the times and velocities for each axis
    times = [traj_x_time; traj_y_time; traj_z_time];
    velocities = [traj_x_velo; traj_y_velo; traj_z_velo];
end
