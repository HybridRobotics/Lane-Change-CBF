classdef StraightLane
    % assuming that the vehicle is driving from left to right on the lane,
    % and the lowes lane is with the number id 1; the start position of the
    % lowest line is (0,0)
    properties
        num_lanes; % number of the lanes
        lane_width; % width of each single lane
        max_length; % maximum length of the lanes
    end
    methods
        function self = StraightLane(num_lanes, lane_width, max_length)
            self.num_lanes = num_lanes;
            self.lane_width = lane_width;
            self.max_length = max_length;
        end
        function [] = plot_lanes(self)
            for i = 1:self.num_lanes + 1
                plot([0, self.max_length], [0 + (i - 1) * self.lane_width, 0 + (i - 1) * self.lane_width], 'k--');
                hold on
                xlim([-10, self.max_length + 10]);
                ylim([-2.5, self.lane_width * self.num_lanes + 2.5]);
                % show the whole scenes in the visulization, -10, 10, -2.5 and 2.5
                % are self defined values to make the visulization more clear
            end
        end
    end
end