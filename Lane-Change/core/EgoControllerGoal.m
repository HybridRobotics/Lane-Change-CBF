classdef EgoControllerGoal
    properties
        target_y; % target lateral position in frame E
        target_speed; % the ego vehicle's desired speed
        lim_beta; % slip angle limit
        lim_acc; % acceleration limit
        lim_slip_rate; % slip angle changing rate limit
        safety_factor;
        lim_speed;
        lim_yaw;
        scenario;
    end
    methods
        function self = EgoControllerGoal(initial_lane_id, direction_flag, desired_speed, lanes, lim_slip_angle, lim_acc, lim_slip_rate, safety_factor, scenario)
            self.target_y = ((initial_lane_id + direction_flag) - 1) * lanes.lane_width + 0.5 * lanes.lane_width;
            self.target_speed = desired_speed;
            self.lim_beta = lim_slip_angle;
            self.lim_acc = lim_acc;
            self.lim_slip_rate = lim_slip_rate;
            self.safety_factor = safety_factor;
            self.scenario = scenario;
            if self.scenario == 1
                self.lim_speed = 33.33; % equal to 120 km/h
            elseif self.scenario == 2
                self.lim_speed = 16.67; % equal to 60 km/h
            end
        end
    end
end
