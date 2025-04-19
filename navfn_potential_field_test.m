% 1. 生成测试地图并预处理
costmap = zeros(30, 30);
costmap(10:20, 15) = 100; % 垂直障碍
costmap(15, 5:25) = 100;  % 水平障碍（十字形）
inflated_map = navfn_preprocessing.inflate_obstacles(costmap, 1);
normalized_map = navfn_preprocessing.normalize_costmap(inflated_map);
smoothed_map = navfn_preprocessing.smooth_edges(normalized_map, 1.5);   % 高斯平滑


% 2. 设置目标点（势场扩散起点）
goal = [5, 5]; % [row, col]

% 3. 计算势场
potential = navfn_potential_field.compute_potential(smoothed_map, goal);

% 4. 可视化势场和梯度
navfn_potential_field.plot_potential(potential, smoothed_map);