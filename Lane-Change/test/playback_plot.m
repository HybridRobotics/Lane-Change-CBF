close all
color_band = [[0.8500, 0.3250, 0.0980]; ...
    [0.9290, 0.6940, 0.1250]; ...
    [0.4940, 0.1840, 0.5560]; ...
    [0.4660, 0.6740, 0.1880]; ...
    ];

%% trajectory
figure1 = figure('Renderer', 'painters', 'Position', [500, 500, 625, 80]);
set(gca, 'LooseInset', get(gca, 'TightInset'));
axis off
hold on
plot([-15, 350], [0, 0], 'k--', 'LineWidth', 1.5)
plot([-15, 350], [3.5, 3.5], 'k--', 'LineWidth', 1.5)
plot([-15, 350], [7, 7], 'k--', 'LineWidth', 1.5)
plot([-15, 350], [10.5, 10.5], 'k--', 'LineWidth', 1.5)
xlim([-15, 350])
count = 50;
for i = 1:count
    plot_car_alpha(ego_vehicle.state_log(:, (1000 / count) * i), ego_vehicle.param.l_fc+ego_vehicle.param.l_rc, ego_vehicle.param.width, [1, 0, 0], 0.3+0.5*i/count)
    plot_car_alpha(car1.state_log(:, (1000 / count) * i), car1.param.l_fc+car1.param.l_rc, car1.param.width, [0, 0, 1], 0.3+0.5*i/count)
end
axis normal
print(gcf,'figures/trajectory.png','-dpng','-r1000');

%% speed
figure2 = figure('Renderer', 'painters', 'Position', [500, 500, 625, 180]);
set(gca, 'LooseInset', get(gca, 'TightInset'));
xlabel('Time (s)', 'interpreter', 'latex', 'FontSize', 12);
ylabel('$v(t)$ (m/s)', 'Interpreter', 'latex', 'FontSize', 12);
set(gca, 'LineWidth', 0.2, 'FontSize', 12);
hold on
plot(0:0.01:10, ego_vehicle.state_log(4, :), 'Color', color_band(1, :), 'LineWidth', 2);
print(gcf,'figures/speed.png','-dpng','-r1000');
%% steering
figure3 = figure('Renderer', 'painters', 'Position', [500, 500, 625, 180]);
set(gca, 'LooseInset', get(gca, 'TightInset'));
xlabel('Time (s)', 'interpreter', 'latex', 'FontSize', 12);
ylabel('$\delta_f(t)$ (rad)', 'Interpreter', 'latex', 'FontSize', 12);
set(gca, 'LineWidth', 0.2, 'FontSize', 12);
hold on
steering_log = atan(tan(ego_vehicle.input_log(2, :)).*(ego_vehicle.param.l_f + ego_vehicle.param.l_r)./ego_vehicle.param.l_r);
plot(0:0.01:10, steering_log(1, :), 'Color', color_band(1, :), 'LineWidth', 2);
print(gcf,'figures/steering.png','-dpng','-r1000');