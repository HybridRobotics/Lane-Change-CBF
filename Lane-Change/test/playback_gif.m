close all
t = 0:0.01:10;
filename = ['play_back'];
filename = [filename, '.gif'];
fileplace = 'gif/';
filename = [fileplace, filename];
for i = 1:length(t)
    num = length(other_vehicles);
    set(gcf, 'Position', [100, 100, 2300, 1000]);
    clf;
    hold on
    axis off
    hold on
    lane_width = ego_vehicle.lanes.lane_width;
    plot([-150, 1500], [0, 0], 'k--', 'LineWidth', 1.5)
    plot([-150, 1500], [lane_width, lane_width], 'k--', 'LineWidth', 1.5)
    plot([-150, 1500], [2 * lane_width, 2 * lane_width], 'k--', 'LineWidth', 1.5)
    plot([-150, 1500], [3 * lane_width, 3 * lane_width], 'k--', 'LineWidth', 1.5)

    x = ego_vehicle.state_log(1, i);
    car_bt_range = x - 100;
    carft_range = x + 100;
    carfc_range = x + 100;
    target_lane_vehicles = [];
    current_lane_vehicles = [];
    carbt = 0;
    carft = 0;
    carfc = 0;
    for k = 1:num
        if other_vehicles(k).other_log(1, i) <= ego_vehicle.initial_lane_id + 0.5 * ego_vehicle.direction_flag & other_vehicles(k).other_log(1, i) >= ego_vehicle.initial_lane_id - 0.5 * ego_vehicle.initial_lane_id
            current_lane_vehicles = [current_lane_vehicles, other_vehicles(k)];
        end
        if (other_vehicles(k).other_log(1, i) >= ego_vehicle.initial_lane_id + ego_vehicle.direction_flag * 0.5) & (other_vehicles(k).other_log(1, i) <= ego_vehicle.initial_lane_id + 3 * ego_vehicle.direction_flag * 0.5)
            target_lane_vehicles = [target_lane_vehicles, other_vehicles(k)];
        end
    end
    for k = 1:length(target_lane_vehicles)
        if target_lane_vehicles(k).state_log(1, i) <= x & target_lane_vehicles(k).state_log(1, i) >= car_bt_range
            car_bt_range = target_lane_vehicles(k).state_log(1, i);
            carbt = 1;
        end
        if target_lane_vehicles(k).state_log(1, i) >= x & target_lane_vehicles(k).state_log(1, i) <= carft_range
            carft_range = target_lane_vehicles(k).state_log(1, i);
            carft = 1;
        end
    end
    for k = 1:length(current_lane_vehicles)
        if current_lane_vehicles(k).state_log(1, i) >= x & current_lane_vehicles(k).state_log(1, i) <= carfc_range
            carfc_range = current_lane_vehicles(k).state_log(1, i);
            carfc = 1;
        end
    end

    for j = 1:num
        if other_vehicles(j).other_log(1, i) <= ego_vehicle.initial_lane_id + 0.5 * ego_vehicle.direction_flag & other_vehicles(k).other_log(1, i) >= ego_vehicle.initial_lane_id - 0.5 * ego_vehicle.initial_lane_id
            if other_vehicles(j).state_log(1, i) <= carfc_range
                if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + ego_vehicle.direction_flag
                    plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.19, 0.50, 0.08]);
                else
                    plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0, 0, 1]);
                end
            else
                plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.19, 0.50, 0.08]);
            end
        elseif other_vehicles(j).other_log(1, i) >= ego_vehicle.initial_lane_id + 0.5 * ego_vehicle.direction_flag & other_vehicles(j).other_log(1, i) <= ego_vehicle.initial_lane_id + 1.5 * ego_vehicle.direction_flag
            if other_vehicles(j).state_log(1, i) <= carft_range & other_vehicles(j).state_log(1, i) >= x
                plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.9290, 0.6940, 0.1250]);
            elseif other_vehicles(j).state_log(1, i) <= x & other_vehicles(j).state_log(1, i) >= car_bt_range
                if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + ego_vehicle.direction_flag
                    plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.19, 0.50, 0.08]);
                else
                    plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.63, 0.13, 0.94]);
                end
            else
                plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.19, 0.50, 0.08]);
            end
        else
            plot_car(other_vehicles(j).state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [0.19, 0.50, 0.08]);
        end
    end
    plot_car(ego_vehicle.state_log(1:3, i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [1, 0, 0])
    xlim([-90, 300])
    tmp = ego_vehicle.state_log(4, i);
    tmp0 = num2str(tmp);
    tmp0 = ['=', tmp0, ' ', 'm/s'];
    str0 = ['$$v(t)$$', tmp0];
    text(105, 40, str0, 'Interpreter', 'latex', 'FontSize', 22);

    if ego_vehicle.direction_flag == 0
        str1 = ['Current State: Adaptive Cruise Control State $\textbf{(ACC)}$'];
    elseif ego_vehicle.direction_flag == 1
        if ego_vehicle.other_log(2, i) == 1
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + ego_vehicle.direction_flag
                str1 = ['Current State: Adaptive Cruise Control State $\textbf{(ACC)}$'];
            else
                str1 = ['Current State: Left Lane Change State $\textbf{(L)}$'];
            end
        else
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id
                str1 = ['Current State: Adaptive Cruise Control State $\textbf{(ACC)}$'];
            else
                str1 = ['Current State: Back to Current (Left) State $\textbf{(BL)}$'];
            end
        end
    else
        if ego_vehicle.other_log(2, i) == 1
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + ego_vehicle.direction_flag
                str1 = ['Current State: Adaptive Cruise Control State $\textbf{(ACC)}$'];
            else
                str1 = ['Current State: Left Lane Change State $\textbf{(R)}$'];
            end
        else
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id
                str1 = ['Current State: Adaptive Cruise Control State $\textbf{(ACC)}$'];
            else
                str1 = ['Current State: Back to Current (Right) State $\textbf{(BR)}$'];
            end
        end
    end
    text(105, 70, str1, 'Interpreter', 'latex', 'FontSize', 22);

    if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id
        tmpp = num2str(0);
    elseif ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + 0.5 * ego_vehicle.direction_flag
        tmpp = num2str(0.5);
    else
        tmpp = num2str(1);
    end
    if ego_vehicle.other_log(2, i) == 1
        tmpe = num2str(1);
    else
        tmpe = num2str(0);
    end
    if ego_vehicle.direction_flag == 0
        tmpc = num2str(0);
    elseif ego_vehicle.direction_flag == 1
        tmpc = num2str(1);
    else
        tmpc = num2str(-1);
    end
    str2 = ['Input Signals: $$c=$$', tmpc, ', $$p=$$', tmpp, ', $$e=$$', tmpe];
    text(105, 60, str2, 'Interpreter', 'latex', 'FontSize', 22);

    if ego_vehicle.direction_flag == 0
        str3 = ['Barrier Function used in CLF-CBF-QP Formulation: $$h_{fc}$$'];
    else
        if ego_vehicle.other_log(2, i) == 1
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id + ego_vehicle.direction_flag
                if carft == 1
                    str3 = ['Barrier Function used in CLF-CBF-QP Formulation: $$h_{ft}$$'];
                else
                    str3 = ['No Barrier Function used in CLF-CBF-QP Formulation'];
                end
            else
                if carft == 1
                    tmpft = [' $$h_{ft}$$,'];
                else
                    tmpft = [''];
                end
                if carfc == 1
                    tmpfc = [' $$h_{fc}$$,'];
                else
                    tmpfc = [''];
                end
                if carbt == 1
                    tmpbt = [' $$h_{bt}$$'];
                else
                    tmpbt = [''];
                end
                if carft == 0 & carfc == 0 & carbt == 0
                    str3 = ['No Barrier Function used in CLF-CBF-QP Formulation'];
                else
                    str3 = ['Barrier Function used in CLF-CBF-QP Formulation:', tmpfc, tmpft, tmpbt];
                end
            end
        else
            if ego_vehicle.other_log(1, i) == ego_vehicle.initial_lane_id
                if carfc == 1
                    str3 = ['Barrier Function used in CLF-CBF-QP Formulation: $$h_{fc}$$'];
                else
                    str3 = ['No Barrier Function used in CLF-CBF-QP Formulation'];
                end
            else
                if carft == 1
                    tmpft = [' $$h_{ft}$$,'];
                else
                    tmpft = [''];
                end
                if carfc == 1
                    tmpfc = [' $$h_{fc}$$,'];
                else
                    tmpfc = [''];
                end
                if carbt == 1
                    tmpbt = [' $$h_{bt}$$'];
                else
                    tmpbt = [''];
                end
                if carft == 0 & carfc == 0 & carbt == 0
                    str3 = ['No Barrier Function used in CLF-CBF-QP Formulation'];
                else
                    str3 = ['Barrier Function used in CLF-CBF-QP Formulation:', tmpfc, tmpft, tmpbt];
                end
            end
        end
    end
    text(105, 50, str3, 'Interpreter', 'latex', 'FontSize', 22);
    CurrFrame = getframe;
    im = frame2im(CurrFrame);
    [A, map] = rgb2ind(im, 256);
    if i == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.01);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.01);
    end
end
