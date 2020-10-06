classdef Controller
    properties
        goal;
        flag; % type of the controller, value 1 stands for CBF_CLF_QP controller of the ego-vehicle;
        % value 2 stands for CLF_QP controller of surrounding vehicle
        param_opt; % parameters for optimization problem
        param_sys; % parameter for vehicle
        straightlane;
        other_vehicles; % surrounding vehicle
    end
    methods
        function self = Controller(controller_goal, controller_flag, method_param, veh_param, straightlane, other_vehicles)
            self.goal = controller_goal;
            self.flag = controller_flag;
            self.param_opt = method_param;
            self.param_sys = veh_param;
            self.straightlane = straightlane;
            self.other_vehicles = other_vehicles;
        end
        function [acc_flag, optimal_input, e] = get_optimal_input(self, state, last_input, lane_id, input_log, initial_lane_id, direction_flag, acc_flag)
            if self.flag == 1
                cbf = CLF_CBF_QP(self.param_opt, self.param_sys, self.goal, self.straightlane, self.other_vehicles);
                [acc_flag, optimal_input, e] = cbf.get_optimal_input(state, last_input, lane_id, input_log, initial_lane_id, direction_flag, acc_flag);
            elseif self.flag == 2
                clf = CLF_QP(self.param_opt, self.param_sys, self.goal, self.straightlane);
                [acc_flag, optimal_input, e] = clf.get_optimal_input(state, last_input, lane_id, input_log, initial_lane_id, direction_flag);
            end
        end
    end
end
