classdef CLF_QP % an qp for surrounding vehicle that is changing the lane
    properties
        param_opt;
        param_sys;
        goal;
        straightlane;
    end
    methods
        function self = CLF_QP(cbf_param, veh_param, controller_goal, straightlane)
            self.param_opt = cbf_param;
            self.param_sys = veh_param;
            self.goal = controller_goal;
            self.straightlane = straightlane;
        end
        function [acc_flag, input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, initial_lane_id, direction_flag)
            alpha = self.param_opt.alpha;
            target_y = self.goal.target_y;
            target_speed = self.goal.target_speed;
            l_r = self.param_sys.l_r;
            l_f = self.param_sys.l_f;
            width = self.param_sys.width;
            dt = self.param_opt.dt;
            [x, y, phi, v] = self.unpack_state(state);
            [acc, beta] = self.unpack_input(last_input);
            A = [0, 0, -v * sin(phi + beta), cos(phi + beta); ...
                0, 0, v * cos(phi + beta), sin(phi + beta); ...
                0, 0, 0, sin(beta) / l_r; ...
                0, 0, 0, 0];
            B = [0, -v * sin(phi + beta); ...
                0, v * cos(phi + beta); ...
                0, v * cos(beta) / l_r; ...
                1, 0];
            if lane_id == initial_lane_id + direction_flag;
                Q = diag([10^-15, 10^-1, 10^(10), 0.5]);
            else
                Q = diag([10^-15, 0.4, 10^(3), 1]);
            end
            R = diag([1, 1]);
            [K, P] = lqr(A, B, Q, R);
            x_lqr = state - [0; target_y; 0; target_speed];
            V = x_lqr' * P * x_lqr;
            phi0 = x_lqr' * (A' * P + P * A) * x_lqr + alpha * V;
            phi1 = [2 * x_lqr' * P * B];
            Aclf = [phi1, -1];
            bclf = [-phi0];
            H = self.param_opt.H;
            F = self.param_opt.F;
            % input constraint
            A_u = [1, 0, 0; ...
                -1, 0, 0; ...
                0, 1, 0; ...
                0, -1, 0];
            A_u0 = [0, 1, 0; ...
                0, -1, 0];
            b_u = [self.goal.lim_acc; self.goal.lim_acc; self.goal.lim_beta; self.goal.lim_beta];
            b_u0 = [beta + self.goal.lim_slip_rate * dt; -beta + self.goal.lim_slip_rate * dt];
            Constraint_A = [Aclf; A_u; A_u0];
            Constraint_b = [bclf; b_u; b_u0];
            options = optimoptions('quadprog', 'Display', 'off');
            % in some Windows OS PC, adding options to the QP may get error
            u = quadprog(H, F, Constraint_A, Constraint_b, [], [], [], [], [], options);
            acc_n = u(1);
            beta_n = u(2);
            input = [acc_n; beta_n];
            acc_flag = 0;
            e = 0;
        end
        function [acc, beta] = unpack_input(self, input)
            acc = input(1);
            beta = input(2);
        end
        function [x, y, phi, v] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            v = state(4);
        end
    end
end