classdef SimulatorRandomTest
    properties
        lanes;
        ego_vehicle;
        other_vehicles; % defined as a list of vehicles
        dt;
        controller;
    end
    methods
        function self = SimulatorRandomTest(lanes, ego_vehicle, other_vehicles, dt, controller)
            self.lanes = lanes;
            self.ego_vehicle = ego_vehicle;
            self.other_vehicles = other_vehicles;
            self.dt = dt;
            self.controller = controller;
        end
        function [complete_time] = startSimulation(self, simulation_time)
            try
                num = simulation_time / self.dt;
                lane_count = 0; % used to find, how long has the ego vehicle in the target lane continuously
                for i = 1:num
                    num_car = size(self.other_vehicles, 1);
                    if num_car >= 1
                        for k = 1:num_car
                            self.other_vehicles(k).update; % update vehicles' state
                        end
                    end
                    self.ego_vehicle.update; % update ego-vehicle's state
                    if self.ego_vehicle.lane_id == self.ego_vehicle.initial_lane_id + self.ego_vehicle.direction_flag
                        lane_count = lane_count + 1;
                    else
                        lane_count = 0;
                    end
                    if lane_count >= 150 % if the ego vehicle is in the target lane for 1.5s continuously
                        break
                    end
                end
                complete_time = i * self.dt;
            catch
                complete_time = 100; % if the ego vehicle can not meet the safety-critical constraint, the time will be set to 100s
            end
        end
    end
end