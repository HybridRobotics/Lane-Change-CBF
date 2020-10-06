dt = 0.01;
simulation_time = 10;
param_sys = ParamVeh();
scenario = 1; % 1 indicates highway, 2 indicates urban road

%% define driving lanes
if scenario == 1
    lane_width = 3.6;
elseif scenario == 2
    lane_width = 3;
end
lanes = StraightLane(3, lane_width, 750);

%% define a controller for a surrounding vehicle, which changes the lane
car4_controller_flag = 2;
car4_direction_flag = -1;
car4_initial_lane = 3;
car4_target_speed = 30;
car4_lim_acc = 0.3 * 9.81;
car4_lim_beta = 15 * pi / 180;
car4_lim_slip_rate = 12 * pi / 180;
controller_goal = LaneChangeSurroundingVehicleGoal((car4_initial_lane+car4_direction_flag-0.5)*lanes.lane_width, car4_target_speed, car4_lim_beta, car4_lim_acc, car4_lim_slip_rate);
param_opt_normal = ParamOptSurroundingVeh(dt);
controller4 = Controller(controller_goal, car4_controller_flag, param_opt_normal, param_sys, lanes, []);

%% define driving scenario
car1 = Vehicle(0, param_sys, [50; 0.5 * lane_width; 0; 24], [0; 0], [], 1, dt, lanes, 0, 0, scenario);
car2 = Vehicle(0, param_sys, [70; 1.5 * lane_width; 0; 28], [0; 0], [], 1, dt, lanes, 0, 0, scenario);
car3 = Vehicle(0, param_sys, [-60; 1.5 * lane_width; 0; 25], [0; 0], [], 1, dt, lanes, 0, 0, scenario);
car4 = Vehicle(2, param_sys, [40; 2.5 * lane_width; 0; car4_target_speed], [0; 0], controller4, 3, dt, lanes, car4_direction_flag, 0, scenario);
other_vehicles = [car1; car2; car3; car4];

%% define a clf_cbf_qp controller
ego_acc_flag = 0;
ego_initial_lane_id = 1; % the initial lane id of ego vehicle
ego_direction_flag = 1; % the direction of the lane chane process of ego vehicle
ego_desired_speed = 27.5;
ego_veh_initial_state = [0; 0.5 * lane_width; 0; ego_desired_speed];
ego_veh_initial_input = [0; 0];
ego_lim_slip_angle = 15 * pi / 180;
ego_lim_acc = 0.3 * 9.81;
ego_lim_slip_rate = 15 * pi / 180;
ego_controller_flag = 1;
ego_safety_factor = 0.5; % range: 0.1~1, 1 means try to have a maximum safety
controller_goal = EgoControllerGoal(ego_initial_lane_id, ego_direction_flag, ego_desired_speed, lanes, ego_lim_slip_angle, ego_lim_acc, ego_lim_slip_rate, ego_safety_factor, scenario);
param_opt = ParamOptEgo(dt);
controller0 = Controller(controller_goal, ego_controller_flag, param_opt, param_sys, lanes, other_vehicles);

%% define an ego-vehicle
ego_vehicle = Vehicle(1, param_sys, ego_veh_initial_state, ego_veh_initial_input, controller0, ego_initial_lane_id, dt, lanes, ego_direction_flag, ego_acc_flag, scenario);

%% define a simulator and start simulation
simulator0 = Simulator(lanes, ego_vehicle, other_vehicles, dt);
[state_log, input_log] = simulator0.startSimulation(simulation_time);

%% plot
[yaw_log, velocity_log, steering_log] = get_movement_log(state_log, input_log, param_sys);
plot_movement_log(yaw_log, velocity_log, steering_log, dt, simulation_time, scenario);
figure(3)
plot(state_log(1, :), state_log(2, :))
function [] = plot_movement_log(yaw_log, velocity_log, steering_log, dt, simulation_time, scenario)
figure(2)
subplot(3, 1, 1)
plot(0:dt:simulation_time, velocity_log)
title('velocity history')
ylabel('m/s')
xlabel('s')
if scenario == 1
    ylim([14, 36])
elseif scenario == 2
    ylim([7, 17])
end
subplot(3, 1, 2)
plot(0:dt:simulation_time, steering_log)
title('steering history')
ylabel('rad')
xlabel('s')
ylim([-0.05, 0.05])
subplot(3, 1, 3)
plot(0:dt:simulation_time, yaw_log)
title('yaw history')
ylabel('rad')
xlabel('s')
ylim([-0.3, 0.3])
end
function [yaw_log, velocity_log, steering_log] = get_movement_log(state_log, input_log, param_sys)
yaw_log = state_log(3, :);
velocity_log = state_log(4, :);
slip_angle_log = input_log(2, :);
steering_log = atan(tan(slip_angle_log).*(param_sys.l_f + param_sys.l_r)./param_sys.l_r);
end