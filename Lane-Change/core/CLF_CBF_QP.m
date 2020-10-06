classdef CLF_CBF_QP
    properties
        param_opt; % qp relevant parameters
        param_sys; % parameters of vehicles
        goal; % control objective
        straightlane; % driving lanes
        other_vehicles; % surrounding vehicles
    end
    methods
        function self = CLF_CBF_QP(cbf_param, veh_param, controller_goal, straightlane, other_vehicles)
            self.param_opt = cbf_param;
            self.param_sys = veh_param;
            self.goal = controller_goal;
            self.straightlane = straightlane;
            self.other_vehicles = other_vehicles;
        end
        function [acc_flag, input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, current_lane_id, direction_flag, acc_flag)

            %% load parameter and control objective
            safety_factor = self.goal.safety_factor;
            lim_speed = self.goal.lim_speed;
            l_rc = self.param_sys.l_rc;
            l_fc = self.param_sys.l_fc;
            l_f = self.param_sys.l_f;
            l_r = self.param_sys.l_r;
            width = self.param_sys.width;
            dt = self.param_opt.dt;
            [x, y, psi, v] = self.unpack_state(state);
            [acc, beta] = self.unpack_input(last_input);
            alpha_y = self.param_opt.alpha_y;
            alpha_v = self.param_opt.alpha_v;
            alpha_yaw = self.param_opt.alpha_yaw;
            gamma_1 = self.param_opt.gamma_1;
            gamma_2 = self.param_opt.gamma_2;
            gamma_3 = self.param_opt.gamma_3;
            gamma_4 = self.param_opt.gamma_4;
            gamma_5 = self.param_opt.gamma_5;
            gamma_6 = self.param_opt.gamma_6;
            target_y = self.goal.target_y;

            %% load surrounding vehicles
            current_lane_vehicles = [];
            target_lane_vehicles = [];
            num_veh = length(self.other_vehicles);
            % sort surrounding vehilces according to the lane id information
            for i = 1:num_veh
                if (self.other_vehicles(i).lane_id == current_lane_id | self.other_vehicles(i).lane_id == current_lane_id - direction_flag * 0.5)
                    % the vehicle is in the current lane
                    if self.other_vehicles(i).state(1) >= x
                        % collect the vehicle in the current lane before the
                        % ego vehicle
                        current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
                    end
                elseif self.other_vehicles(i).lane_id == current_lane_id + direction_flag * 0.5
                    % the vehicle is accross the dividing line
                    if self.other_vehicles(i).state(1) >= x
                        current_lane_vehicles = [current_lane_vehicles, self.other_vehicles(i)];
                    end
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                elseif (self.other_vehicles(i).lane_id == current_lane_id + direction_flag)
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                elseif self.other_vehicles(i).lane_id == current_lane_id + 1.5 * direction_flag
                    target_lane_vehicles = [target_lane_vehicles, self.other_vehicles(i)];
                end
            end
            car_fc = []; % car_fc is the closest leading vehicle in the current lane (vehicle fc)
            carfc_range = x + 100; % this value can be tuned, it shows in which range the vehicle would be considered
            for j = 1:length(current_lane_vehicles)
                if current_lane_vehicles(j).state(1) <= carfc_range
                    car_fc = current_lane_vehicles(j);
                    carfc_range = current_lane_vehicles(j).state(1);
                end
            end
            car_bt = []; % car_bt is the closeset vehicle in the target lane that is behind ego vehicle (vehicle bt)
            car_bt_range = x - 100;
            car_ft = []; % car_ft is the closet leading vehicle in the target lane (vehicle ft)
            carft_range = x + 100;
            for i = 1:length(target_lane_vehicles)
                if target_lane_vehicles(i).state(1) <= x & target_lane_vehicles(i).state(1) >= car_bt_range
                    car_bt = target_lane_vehicles(i);
                    car_bt_range = target_lane_vehicles(i).state(1);
                end
                if target_lane_vehicles(i).state(1) >= x & target_lane_vehicles(i).state(1) <= carft_range
                    car_ft = target_lane_vehicles(i);
                    carft_range = target_lane_vehicles(i).state(1);
                end
            end
            if lane_id == current_lane_id + direction_flag
                acc_flag = 0; % indicates if ego vehicle is accelerating
            end
            if acc_flag == 0
                target_speed = self.goal.target_speed;
            else
                target_speed = lim_speed;
            end

            %% CLF-CBF-QP formulation

            %% lateral position CLF
            h_y = y - target_y;
            V_y = h_y^2;
            phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y;
            phi1_y = [0, 2 * h_y * v * cos(psi)];

            %% velocity CLF
            h_v = v - target_speed;
            V_v = h_v^2;
            phi0_v = alpha_v * V_v;
            phi1_v = [2 * h_v * 1, 0];

            %% yaw angle CLF
            h_yaw = psi;
            V_yaw = h_yaw^2;
            phi0_yaw = alpha_yaw * V_yaw;
            phi1_yaw = [0, 2 * h_yaw * v * l_r];
            Aclf = [phi1_y, -1, 0, 0; ...
                phi1_v, 0, -1, 0; ...
                phi1_yaw, 0, 0, -1;];
            bclf = [-phi0_y; -phi0_v; -phi0_yaw];

            %% Car_fc relevant CBFs
            if isempty(car_fc)
                Acbf1 = [0, 0, 0, 0, 0];
                bcbf1 = [0];
                Acbf2 = [0, 0, 0, 0, 0];
                bcbf2 = [0];
                h_CBF1 = [];
                h_CBF2 = [];
            else
                x_carfc_rear = car_fc.state(1) - l_rc;
                v_carfc = car_fc.state_log(4, end);
                a_carfc = car_fc.input(1);
                % distance based CBF
                h_CBF1 = x_carfc_rear - x - self.param_sys.l_fc;
                h1dot = v_carfc;
                Lfh1 = [-cos(psi) * v];
                Lgh1 = [0, v * sin(psi)];
                Acbf1 = [-Lgh1, 0, 0, 0];
                bcbf1 = [Lfh1 + gamma_1 * h_CBF1 + h1dot];
                % force based CBF
                if v > v_carfc
                    h_CBF2 = x_carfc_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v - 0.5 * (v_carfc - v) * (v_carfc - v) / self.goal.lim_acc;
                    h2dot = v_carfc - (0.5 / self.goal.lim_acc) * 2 * (v_carfc - v) * a_carfc;
                    Lfh2 = [-cos(psi) * v];
                    Lgh2 = [(-(1 + safety_factor) + (v_carfc - v) / self.goal.lim_acc), v * sin(psi)];
                    Acbf2 = [-Lgh2, 0, 0, 0];
                    bcbf2 = [Lfh2 + gamma_2 * h_CBF2 + h2dot];
                else
                    h_CBF2 = x_carfc_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v;
                    h2dot = v_carfc;
                    Lfh2 = [-cos(psi) * v];
                    Lgh2 = [-(1 + safety_factor), v * sin(psi)];
                    Acbf2 = [-Lgh2, 0, 0, 0];
                    bcbf2 = [Lfh2 + gamma_2 * h_CBF2 + h2dot];
                end
            end

            %% Car_bt relevant CBFs
            if isempty(car_bt)
                Acbf3 = [0, 0, 0, 0, 0];
                bcbf3 = [0];
                Acbf4 = [0, 0, 0, 0, 0];
                bcbf4 = [0];
                h_CBF3 = [];
                h_CBF4 = [];
            else
                x_carbt_front = car_bt.state(1) + l_fc;
                v_carbt = car_bt.state_log(4, end);
                a_carbt = car_bt.input(1);
                if car_bt.state(1) <= x - l_fc - l_rc
                    % distance based CBF
                    if v_carbt <= v
                        h_CBF3 = x - l_rc - x_carbt_front;
                        h3dot = -v_carbt;
                        Lfh3 = (cos(psi) * v);
                        Lgh3 = [0, -v * sin(psi)];
                        Acbf3 = [-Lgh3, 0, 0, 0];
                        bcbf3 = [Lfh3 + gamma_3 * h_CBF3 + h3dot];
                    else
                        h_CBF3 = x - l_rc - x_carbt_front - 0.5 * (v_carbt - v) * (v_carbt - v) / self.goal.lim_acc;
                        h3dot = -v_carbt - (1 / self.goal.lim_acc) * (v_carbt - v) * a_carbt;
                        Lfh3 = (cos(psi) * v);
                        Lgh3 = [(v - v_carbt) / self.goal.lim_acc, -v * sin(psi)];
                        Acbf3 = [-Lgh3, 0, 0, 0];
                        bcbf3 = [Lfh3 + gamma_3 * h_CBF3 + h3dot];
                    end
                else
                    h_CBF3 = direction_flag * (car_bt.state(2) - y - width - safety_factor);
                    Lfh3 = -direction_flag * v * sin(psi);
                    Lgh3 = [0, -direction_flag * v * cos(psi)];
                    Acbf3 = [-Lgh3, 0, 0, 0];
                    bcbf3 = [Lfh3 + gamma_3 * h_CBF3];
                end
                if v_carbt > v
                    h_CBF4 = -x_carbt_front + x - l_rc - (1 + safety_factor) * v_carbt - 0.5 * (v_carbt - v) * (v_carbt - v) / self.goal.lim_acc;
                    h4dot = -v_carbt - (1 + safety_factor) * a_carbt - (1 / self.goal.lim_acc) * (v_carbt - v) * a_carbt;
                    Lfh4 = (cos(psi) * v);
                    Lgh4 = [(v - v_carbt) / self.goal.lim_acc, -v * sin(psi)];
                    Acbf4 = [-Lgh4, 0, 0, 0];
                    bcbf4 = [Lfh4 + gamma_4 * h_CBF4 + h4dot];
                else
                    h_CBF4 = -x_carbt_front + x - l_rc - (1 + safety_factor) * v_carbt;
                    h4dot = -v_carbt - (1 + safety_factor) * car_bt.input(1);
                    Lfh4 = cos(psi) * v;
                    Lgh4 = [0, -v * sin(psi)];
                    Acbf4 = [-Lgh4, 0, 0, 0];
                    bcbf4 = [Lfh4 + gamma_4 * h_CBF4 + h4dot];
                end
            end

            %% Car_ft relevant CBFs
            if isempty(car_ft)
                Acbf5 = [0, 0, 0, 0, 0];
                bcbf5 = [0];
                Acbf6 = [0, 0, 0, 0, 0];
                bcbf6 = [0];
                h_CBF5 = [];
                h_CBF6 = [];
            else
                x_carft_rear = car_ft.state(1) - l_rc;
                v_carft = car_ft.state_log(4, end);
                a_carft = car_ft.input_log(1, end);
                if car_ft.state(1) - x > l_fc + l_rc
                    % distance based CBF
                    if v_carft >= v
                        h_CBF5 = x_carft_rear - x - self.param_sys.l_fc;
                        h5dot = v_carft;
                        Lfh5 = (-cos(psi) * v);
                        Lgh5 = [0, v * sin(psi)];
                        Acbf5 = [-Lgh5, 0, 0, 0];
                        bcbf5 = [Lfh5 + gamma_5 * h_CBF5 + h5dot];
                    else
                        h_CBF5 = x_carft_rear - x - self.param_sys.l_fc - 0.5 * (v_carft - v) * (v_carft - v) / self.goal.lim_acc;
                        h5dot = v_carft - (0.5 / self.goal.lim_acc) * 2 * (v_carft - v) * a_carft;
                        Lfh5 = (-cos(psi) * v);
                        Lgh5 = [((v_carft - v) / self.goal.lim_acc), v * sin(psi)];
                        Acbf5 = [-Lgh5, 0, 0, 0];
                        bcbf5 = [Lfh5 + gamma_5 * h_CBF5 + h5dot];
                    end
                else
                    h_CBF5 = direction_flag * (car_ft.state(2) - y - width - safety_factor);
                    Lfh5 = -direction_flag * v * sin(psi);
                    Lgh5 = [0, -direction_flag * v * cos(psi)];
                    Acbf5 = [-Lgh5, 0, 0, 0];
                    bcbf5 = [Lfh5 + gamma_5 * h_CBF5];
                end
                % force based CBF
                if v_carft < v
                    h_CBF6 = x_carft_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v - 0.5 * (v_carft - v) * (v_carft - v) / self.goal.lim_acc;
                    h6dot = v_carft - (0.5 / self.goal.lim_acc) * 2 * (v_carft - v) * a_carft;
                    Lfh6 = -cos(psi) * v;
                    Lgh6 = [(-(1 + safety_factor) + (v_carft - v) / self.goal.lim_acc), v * sin(psi)];
                    Acbf6 = [-Lgh6, 0, 0, 0];
                    bcbf6 = [Lfh6 + gamma_6 * h_CBF6 + h6dot];
                else
                    h_CBF6 = x_carft_rear - x - self.param_sys.l_fc - (1 + safety_factor) * v;
                    h6dot = v_carft;
                    Lfh6 = (-cos(psi) * v);
                    Lgh6 = [-(1 + safety_factor), v * sin(psi)];
                    Acbf6 = [-Lgh6, 0, 0, 0];
                    bcbf6 = [Lfh6 + gamma_6 * h_CBF6 + h6dot];
                end
            end
            % three states, 0: keep the current lane, 1: change to the left
            %  adjacent lane, -1: change to the right adjacent lane
            if direction_flag == 0 % ACC state
                % only car_fc relevant CBF is considered
                Acbf3 = [0, 0, 0, 0, 0];
                bcbf3 = [0];
                Acbf4 = [0, 0, 0, 0, 0];
                bcbf4 = [0];
                Acbf5 = [0, 0, 0, 0, 0];
                bcbf5 = [0];
                Acbf6 = [0, 0, 0, 0, 0];
                bcbf6 = [0];
            else % the ego vehicle is changing its lane, L or R state
                if lane_id == current_lane_id + direction_flag
                    % the ego vehicle is already in its target lane, we
                    % don't need car_fc and car_bt relevant CBFs
                    Acbf1 = [0, 0, 0, 0, 0];
                    bcbf1 = [0];
                    Acbf2 = [0, 0, 0, 0, 0];
                    bcbf2 = [0];
                    Acbf3 = [0, 0, 0, 0, 0];
                    bcbf3 = [0];
                    Acbf4 = [0, 0, 0, 0, 0];
                    bcbf4 = [0];
                end
            end
            % input constraint (accleration limit, slip anlge limit,
            % lateral acceleration limit, slip angle changing rate limit)
            A_u = [1, 0, 0, 0, 0; ...
                -1, 0, 0, 0, 0; ...
                0, 1, 0, 0, 0; ...
                0, -1, 0, 0, 0; ...
                cos(psi + beta), 0, 0, 0, 0; ...
                -cos(psi + beta), 0, 0, 0, 0];
            A_u0 = [0, 1, 0, 0, 0; ...
                0, -1, 0, 0, 0];
            b_u = [self.goal.lim_acc; self.goal.lim_acc; self.goal.lim_beta; self.goal.lim_beta; 0.5 * 0.9 * 9.81; 0.5 * 0.9 * 9.81];
            b_u0 = [beta + 1 * self.goal.lim_slip_rate * dt; -beta + 1 * self.goal.lim_slip_rate * dt];
            Constraint_A = [Aclf; Acbf2; Acbf4; Acbf6; A_u; A_u0];
            Constraint_b = [bclf; bcbf2; bcbf4; bcbf6; b_u; b_u0];
            H = self.param_opt.H;
            F = self.param_opt.F;
            options = optimoptions('quadprog', 'Display', 'off');
            u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
            if ~isempty(u) % get optimal input
                acc_n = u(1);
                beta_n = u(2);
                input = [acc_n; beta_n];
                e = 1;
                if v >= target_speed + 0.5
                    acc_flag = 1;
                end
            else % no optimal input, the ego vehicle enter BL or BR state, drives back to its current lane

                %% predictive calculation, to determine if the ego vehicle can change the lane through acceleration
                if ~isempty(car_ft)
                    x_carft_rear = car_ft.state(1) - l_rc;
                    v_carft = car_ft.state(4);
                    tmp3 = -x_carft_rear + x + (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)^2 / 2 / self.goal.lim_acc) - l_fc - v_carft * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v_carft;
                end
                if ~isempty(car_fc)
                    x_carfc_rear = car_fc.state(1) - l_rc;
                    v_carfc = car_fc.state(4);
                    tmp1 = x_carfc_rear - x - (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)^2 / 2 / self.goal.lim_acc) - l_fc + v_carfc * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v;
                end
                if ~isempty(car_bt)
                    x_carbt_front = car_bt.state(1) + l_fc;
                    v_carbt = car_bt.state(4);
                    tmp2 = x - x_carbt_front - l_rc + (v * (lim_speed - v) / self.goal.lim_acc + (lim_speed - v)^2 / 2 / self.goal.lim_acc) - v_carbt * (lim_speed - v) / self.goal.lim_acc - 0.1 * safety_factor * v_carbt;
                end
                if (~isempty(car_ft) & ~isempty(car_fc) & (tmp3 >= 0 & tmp1 >= 0)) | (~isempty(car_ft) & isempty(car_fc) & (tmp3 >= 0))
                    track_speed = lim_speed;
                    acc_flag = 1;
                elseif (~isempty(car_bt) & (~isempty(car_ft)) & (~isempty(car_fc)) & (tmp2 >= 0 & tmp1 >= 0 & tmp3 >= 0)) | (~isempty(car_bt) & (isempty(car_ft)) & (~isempty(car_fc)) & (tmp2 >= 0 & tmp1 >= 0)) | (~isempty(car_bt) & (~isempty(car_ft)) & (isempty(car_fc)) & (tmp2 >= 0 & tmp3 >= 0)) | (~isempty(car_bt) & (isempty(car_ft)) & (isempty(car_fc)) & (tmp2 >= 0))
                    track_speed = lim_speed;
                    acc_flag = 1;
                else
                    track_speed = target_speed;
                end

                %% lateral position CLF
                h_y = y - (current_lane_id - 0.5) * self.straightlane.lane_width;
                V_y = h_y^2;
                phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y;
                phi1_y = [0, 2 * h_y * v * cos(psi)];

                %% velocity CLF
                h_v = v - track_speed;
                V_v = h_v^2;
                phi0_v = alpha_v * V_v;
                phi1_v = [2 * h_v * 1, 0];

                %% yaw angle CLF
                h_yaw = psi;
                V_yaw = h_yaw^2;
                phi0_yaw = alpha_yaw * V_yaw;
                phi1_yaw = [0, 2 * h_yaw * v * l_r];
                Aclf = [phi1_y, -1, 0, 0; ...
                    phi1_v, 0, -1, 0; ...
                    phi1_yaw, 0, 0, -1;];
                bclf = [-phi0_y; -phi0_v; -phi0_yaw];
                if lane_id == current_lane_id % the ego vehicle is already in its current lane,
                    % FSM enters ACC state, no car_ft and car_bt relevant CBF
                    Acbf3 = [0, 0, 0, 0, 0];
                    bcbf3 = [0];
                    Acbf4 = [0, 0, 0, 0, 0];
                    bcbf4 = [0];
                    Acbf5 = [0, 0, 0, 0, 0];
                    bcbf5 = [0];
                    Acbf6 = [0, 0, 0, 0, 0];
                    bcbf6 = [0];
                end
                Constraint_A = [Aclf; Acbf2; Acbf3; Acbf5; A_u; A_u0];
                Constraint_b = [bclf; bcbf2; bcbf3; bcbf5; b_u; b_u0];
                H = self.param_opt.H;
                F = self.param_opt.F;
                options = optimoptions('quadprog', 'Display', 'off');
                u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
                if (~isempty(u))
                    acc_n = u(1);
                    beta_n = u(2);
                    input = [acc_n; beta_n];
                    e = 0;
                else % the ego vehicle can not meet safety-critical requirement
                    error('fail')
                end
            end
        end
        function [acc, beta] = unpack_input(self, input)
            acc = input(1);
            beta = input(2);
        end
        function [x, y, psi, v] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            psi = state(3);
            v = state(4);
        end
    end
end