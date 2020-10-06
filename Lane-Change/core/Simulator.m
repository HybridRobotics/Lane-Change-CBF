classdef Simulator
    properties
        lanes;
        ego_vehicle;
        other_vehicles; % defined as a list of vehicles
        dt;
    end
    methods
        function self = Simulator(lanes, ego_vehicle, other_vehicles, dt)
            self.lanes = lanes;
            self.ego_vehicle = ego_vehicle;
            self.other_vehicles = other_vehicles;
            self.dt = dt;
        end
        function [state_log, input_log] = startSimulation(self, simulation_time)
            num = simulation_time / self.dt;
            for i = 1:num
                figure(1);
                set(gcf, 'Position', [0, 0, 2500, 1300]);
                clf;
                self.lanes.plot_lanes; % plot lanes
                hold on
                num_car = size(self.other_vehicles, 1);
                if num_car >= 1
                    for k = 1:num_car
                        self.other_vehicles(k).plot_vehicle; % plot vehicles
                        self.other_vehicles(k).update; % update vehicles' state
                    end
                end
                self.ego_vehicle.plot_vehicle; % plot ego-vehicle
                self.ego_vehicle.update; % update ego-vehicle's state
            end
            state_log = self.ego_vehicle.state_log; % update states history
            input_log = self.ego_vehicle.input_log; % update inputs history
        end
    end
end