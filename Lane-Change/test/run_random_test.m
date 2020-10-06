% define simulation scenario
dt = 0.01;
simulation_time = 60;
param_sys = ParamVeh();
scenario = 1; % 1 indicates highway, 2 indicates urban road
% driving lanes for different scenarios
if scenario == 1
    lane_width = 3.6;
elseif scenario == 2
    lane_width = 3;
end
lanes = StraightLane(3, lane_width, 1500);
% surrounding vehicle's initial states log
T = zeros(100, 1);
initial_state_log = zeros(100, 6, 3);
% generate 100 groups of random tests
for i = 1:100
    i
    % gernerate a controller for lane changing other car
    car6_controller_flag = 2;
    car6_direction_flag = -1; % change to the right
    car6_initial_lane = 3;
    if scenario == 1
        car6_target_speed = 29 + 3 * (2 * rand - 1);
    elseif scenario == 2
        car6_target_speed = 13 + 2 * (2 * rand - 1);
    end
    car6_lim_acc = 0.3 * 9.81;
    car6_lim_beta = 15 * pi / 180;
    car6_lim_slip_rate = 12 * pi / 180;
    controller_goal = LaneChangeSurroundingVehicleGoal((car6_initial_lane+car6_direction_flag-0.5)*lanes.lane_width, car6_target_speed, car6_lim_beta, car6_lim_acc, car6_lim_slip_rate);
    param_opt_normal = ParamOptSurroundingVeh(dt);
    controller6 = Controller(controller_goal, car6_controller_flag, param_opt_normal, param_sys, lanes, []);
    % generated cars in the environment randomly (1 car ahead ego vehicle, 4 cars on the perspective lane, 1 car is chaging the lane from lane 3 to lane 2)
    if scenario == 1
        car1 = Vehicle(0, param_sys, [50 + 15 * rand; 0.5 * lane_width; 0; 29 + 3 * (2 * rand - 1)], [3 * (2 * rand - 1); 0], [], 1, dt, lanes, 0, 0, scenario);
        car2 = Vehicle(0, param_sys, [-85 + 170 * rand; 1.5 * lane_width; 0; 29 + 3 * (2 * rand - 1)], [6 * rand - 3; 0], [], 2, dt, lanes, 0, 0, scenario);
        car3 = Vehicle(0, param_sys, [-85 + 170 * rand; 1.5 * lane_width; 0; 29 + 3 * (2 * rand - 1)], [6 * rand - 3; 0], [], 2, dt, lanes, 0, 0, scenario);
        car4 = Vehicle(0, param_sys, [-85 + 170 * rand; 1.5 * lane_width; 0; 29 + 3 * (2 * rand - 1)], [6 * rand - 3; 0], [], 2, dt, lanes, 0, 0, scenario);
        car5 = Vehicle(0, param_sys, [-85 + 170 * rand; 1.5 * lane_width; 0; 29 + 3 * (2 * rand - 1)], [6 * rand - 3; 0], [], 2, dt, lanes, 0, 0, scenario);
        car6 = Vehicle(2, param_sys, [-85 + 170 * rand; 2.5 * lane_width; 0; car6_target_speed], [0; 0], controller6, 3, dt, lanes, car6_direction_flag, 0, scenario);
    elseif scenario == 2
        car1 = Vehicle(0, param_sys, [25 + 15 * rand; 0.5 * lane_width; 0; 13 + 2 * (2 * rand - 1)], [4 * rand - 2; 0], [], 1, dt, lanes, 0, 0, scenario);
        car2 = Vehicle(0, param_sys, [-50 + 100 * rand; 1.5 * lane_width; 0; 13 + 2 * (2 * rand - 1)], [4 * rand - 2; 0], [], 2, dt, lanes, 0, 0, scenario);
        car3 = Vehicle(0, param_sys, [-50 + 100 * rand; 1.5 * lane_width; 0; 13 + 2 * (2 * rand - 1)], [4 * rand - 2; 0], [], 2, dt, lanes, 0, 0, scenario);
        car4 = Vehicle(0, param_sys, [-50 + 100 * rand; 1.5 * lane_width; 0; 13 + 2 * (2 * rand - 1)], [4 * rand - 2; 0], [], 2, dt, lanes, 0, 0, scenario);
        car5 = Vehicle(0, param_sys, [-50 + 100 * rand; 1.5 * lane_width; 0; 13 + 2 * (2 * rand - 1)], [4 * rand - 2; 0], [], 2, dt, lanes, 0, 0, scenario);
        car6 = Vehicle(2, param_sys, [-50 + 100 * rand; 2.5 * lane_width; 0; car6_target_speed], [0; 0], controller6, 3, dt, lanes, car6_direction_flag, 0, scenario);
    end
    other_vehicles = [car1; car2; car3; car4; car5; car6];
    for j = 1:6
        initial_state_log(i, j, 1) = other_vehicles(j).state(1);
        initial_state_log(i, j, 2) = other_vehicles(j).state(4);
        initial_state_log(i, j, 3) = other_vehicles(j).input(1);
    end
    % define an ego vehilce
    ego_acc_flag = 0;
    ego_initial_lane_id = 1; % the initial lane id of ego vehicle
    ego_direction_flag = 1; % the direction of the lane chane process of ego vehicle
    if scenario == 1
        ego_desired_speed = 29;
    elseif scenario == 2
        ego_desired_speed = 13;
    end
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
    ego_vehicle = Vehicle(1, param_sys, ego_veh_initial_state, ego_veh_initial_input, controller0, ego_initial_lane_id, dt, lanes, ego_direction_flag, ego_acc_flag, scenario);
    % define a simulator and get simulation result
    simulator0 = SimulatorRandomTest(lanes, ego_vehicle, other_vehicles, dt, controller0);
    [complete_time] = simulator0.startSimulation(simulation_time);
    T(i) = complete_time;
end