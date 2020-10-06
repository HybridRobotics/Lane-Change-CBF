classdef Vehicle < handle
    properties
        dynamics_flag; % 1 stands for ego-vehicle, 0 stands for normal surrounding vehicles (move at a constant speed or acceleration)
        % 2 stands for surrounding vehicles that will change the lane
        initial_lane_id; % initial lane id of the vehicle
        lane_id; % curren lane id of the vehicle
        param; % Parameters of the vehicle
        state; % current state of the vehicle
        state_log; % history of states
        input; % current input of the vehicle
        input_log; % history of inputs (speed and slip angle for the ego vehicle and lane change vehicle; speed and acceleartion for normal vehicles)
        controller; % controller used for lane changing
        dt; % time gap for simulation
        lanes;
        direction_flag; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing
        % to the right adjacent lane
        acc_flag;
        scenario;
        other_log;
    end
    methods
        function self = Vehicle(dynamics_flag, veh_param, state_initial, initial_input, controller, initial_lane_id, dt, lanes, direction_flag, acc_flag, scenario)
            self.dynamics_flag = dynamics_flag;
            self.param = veh_param;
            self.controller = controller;
            self.state = state_initial;
            % at first time step, the current state of the vehicle is same to initial state
            self.input = initial_input;
            % at first time step, the current input is same as initial input
            self.state_log = state_initial;
            % the first element of states' history
            self.input_log = initial_input;
            % the first element of inputs' history
            self.initial_lane_id = initial_lane_id;
            self.lane_id = initial_lane_id;
            self.dt = dt;
            self.lanes = lanes;
            self.direction_flag = direction_flag;
            self.acc_flag = acc_flag;
            self.scenario = scenario;
            self.other_log = [initial_lane_id; 1];
        end
        function update(self)
            % during the simulation, using this method at each time step to update the vehicle
            if self.dynamics_flag == 0 % when the vehile is a normal vehilce
                normal_car_update(self);
            elseif self.dynamics_flag == 1 % using the controller to calculate the input and update the state
                ego_vehicle_update(self);
            elseif self.dynamics_flag == 2
                lane_change_car_update(self);
            end
        end
        function plot_vehicle(self)
            if self.dynamics_flag == 0 | self.dynamics_flag == 2
                self.draw_vehicle(self.state, (self.param.l_fc + self.param.l_rc), self.param.width, self.dynamics_flag);
            else
                self.draw_vehicle(self.state, (self.param.l_fc + self.param.l_rc), self.param.width, self.dynamics_flag);
            end
        end
        function normal_car_update(self)
            self.get_lane_id(self.state);
            speed = self.state(4);
            acceleration = self.input(1);
            self.state(4) = self.state(4) + acceleration * self.dt; % new speed
            % speed limit according to different scenarios
            if self.scenario == 1
                ulim = 33.33;
                llim = 23;
            elseif self.scenario == 2
                ulim = 16.67;
                llim = 12;
            end
            if self.state(4) >= ulim
                self.state(4) = ulim;
            elseif self.state(4) <= llim
                self.state(4) = llim;
            end
            dx = speed * self.dt + 0.5 * acceleration * self.dt^2; %dx=v*dt+0.5*a*dt^2
            self.state = [self.state(1) + dx; self.state(2); self.state(3); self.state(4)]; % new state of normal cars
            self.state_log = [self.state_log, self.state]; % update the state hisotory
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), 0];
        end
        function ego_vehicle_update(self)
            self.get_lane_id(self.state);
            [self.acc_flag, u, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag, self.acc_flag);
            % calculate the optimal input of the vehicle
            dx = self.Bicycle(self.state(1:3), [self.state(4) + self.dt * u(1); u(2)]); % calculate dX through nonlinear model
            self.state = self.state + self.dt .* [dx; u(1)]; % update the state of the vehicle
            self.input = u; % update the input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
        end
        function lane_change_car_update(self)
            self.get_lane_id(self.state);
            [n, u, e] = self.controller.get_optimal_input(self.state, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag);
            % calculate the optimal input of the vehicle
            dx = self.Bicycle(self.state(1:3), [self.state(4) + self.dt * u(1); u(2)]); % calculate dX through nonlinear model
            self.state = self.state + self.dt .* [dx; u(1)]; % update the state of the vehicle
            self.input = u; % update the input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input history
            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e];
        end

        function [] = get_lane_id(self, state)
            if state(2) <= self.lanes.lane_width - 0.5 * self.param.width
                self.lane_id = 1;
            else if state(2) <= self.lanes.lane_width + 0.5 * self.param.width
                    self.lane_id = 1.5;
                else if state(2) <= 2 * self.lanes.lane_width - 0.5 * self.param.width
                        self.lane_id = 2;
                    else if state(2) <= 2 * self.lanes.lane_width + 0.5 * self.param.width
                            self.lane_id = 2.5;
                        else if state(2) <= 3 * self.lanes.lane_width - 0.5 * self.param.width
                                self.lane_id = 3;
                            else if state(2) <= 3 * self.lanes.lane_width + 0.5 * self.param.width
                                    self.lane_id = 3.5;
                                else if state(2) <= 4 * self.lanes.lane_width - 0.5 * self.param.width
                                        self.lane_id = 4;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        function dX = Bicycle(self, state, input)
            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            [x, y, phi] = self.unpack_state(state);
            [v, beta] = self.unpack_input(input);
            delta_f = atan(l*tan(beta)/l_r); % calculate front steering angle from slip angle
            xdot = v * cos(phi+beta); % velocity in x direction
            ydot = v * sin(phi+beta); % velocity in y direction
            phidot = v * sin(beta) / l_r; % yaw rate
            dX = [xdot; ydot; phidot];
        end
        function [speed, beta] = unpack_input(self, input)
            speed = input(1);
            beta = input(2);
        end
        function [x, y, phi] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
        end
        function[] = draw_vehicle(self, state, L, H, dynamics_flag)
            theta = state(3);
            center1 = state(1);
            center2 = state(2);
            R = ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
            X = ([-L / 2, L / 2, L / 2, -L / 2]);
            Y = ([-H / 2, -H / 2, H / 2, H / 2]);
            for i = 1:4
                T(:, i) = R * [X(i); Y(i)];
            end
            x_lower_left = center1 + T(1, 1);
            x_lower_right = center1 + T(1, 2);
            x_upper_right = center1 + T(1, 3);
            x_upper_left = center1 + T(1, 4);
            y_lower_left = center2 + T(2, 1);
            y_lower_right = center2 + T(2, 2);
            y_upper_right = center2 + T(2, 3);
            y_upper_left = center2 + T(2, 4);
            x_coor = [x_lower_left, x_lower_right, x_upper_right, x_upper_left];
            y_coor = [y_lower_left, y_lower_right, y_upper_right, y_upper_left];
            if dynamics_flag == 0 | dynamics_flag == 2
                color = [0, 0, 1]; % blue
            else
                color = [1, 0, 0]; % red
            end
            patch('Vertices', [x_coor; y_coor]', 'Faces', [1, 2, 3, 4], 'Edgecolor', color, 'Facecolor', color, 'Linewidth', 1.2);
            axis equal;
        end
    end
end