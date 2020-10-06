classdef LaneChangeSurroundingVehicleGoal
    properties
        target_y; % desired lateral position in frame E
        target_speed;
        lim_beta;
        lim_acc;
        lim_slip_rate;
    end
    methods
        function self = LaneChangeSurroundingVehicleGoal(target_y, target_speed, lim_beta, lim_acc, lim_slip_rate)
            self.target_speed = target_speed;
            self.target_y = target_y;
            self.lim_acc = lim_acc;
            self.lim_beta = lim_beta;
            self.lim_slip_rate = lim_slip_rate;
        end
    end
end
