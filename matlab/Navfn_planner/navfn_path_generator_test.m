% 1. 生成测试环境
costmap = zeros(50, 50);
costmap(20:40, 10) = 100;  % 垂直障碍
costmap(20:40, 40) = 100;  % 垂直障碍
costmap(30, 10:40) = 100;   % 水平障碍
inflated_map = navfn_preprocessing.inflate_obstacles(costmap, 1);
normalized_map = navfn_preprocessing.normalize_costmap(inflated_map);
smoothed_map = navfn_preprocessing.smooth_edges(normalized_map, 1.5);   % 高斯平滑

% 2. 计算势场
goal = [5, 5];
start = [45, 45];
potential = navfn_potential_field.compute_potential(smoothed_map, goal);


% 3. 生成路径
path = navfn_path_generator.gradient_descent(potential, start, goal,500);

% 4. 可视化
navfn_path_generator.plot_path_comparison(path, smoothed_map);