classdef ParamOptSurroundingVeh
    properties
        alpha = 2;
        dt;
        H = [0, 0, 0; ...
            0, 0.01, 0; ...
            0, 0, 10^(-6); ...
            ];
        F = [0; 0; 0];
    end
    methods
        function self = ParamOptSurroundingVeh(dt)
            self.dt = dt;
        end
    end
end
