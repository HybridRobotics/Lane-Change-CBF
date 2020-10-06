classdef ParamOptEgo
    properties
        alpha_y = 0.8;
        alpha_v = 1.7;
        alpha_yaw = 12;
        gamma_1 = 1;
        gamma_2 = 1;
        gamma_3 = 1;
        gamma_4 = 1;
        gamma_5 = 1;
        gamma_6 = 1;
        dt;
        H = [0.01, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0; ...
            0, 0, 15, 0, 0; ...
            0, 0, 0, 0.1, 0; ...
            0, 0, 0, 0, 400; ...
            ];
        F = [0; 0; 0; 0; 0];
    end
    methods
        function self = ParamOptEgo(dt)
            self.dt = dt;
        end
    end
end
