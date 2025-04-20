% 创建生成器实例
generator = CylinderCostmapGenerator('mapSize', [100, 100], ...
                                   'numCylinders', 64, ...
                                   'minRadius', 3, ...
                                   'maxRadius', 6);
% 0. 生成costmap
while true
    [costmap, cylinderInfo] = generator.generate();
    
    % 可视化
    generator.visualize(costmap);
    
    % 提示用户输入
    userInput = input('是否采用这些数据？(y/n): ', 's');
    
    if lower(userInput) == 'y'
        disp('数据已被采用。');
        break;
    elseif lower(userInput) == 'n'
        disp('数据未被采用，生成新数据。');
    else
        disp('无效输入，退出。');
        return; % 退出
    end
end

% 1. costmap处理
% 障碍物层膨胀
inflated_map = navfn_preprocessing.inflate_obstacles(costmap, 1);
% 高斯平滑
smoothed_map = navfn_preprocessing.smooth_edges(inflated_map, 1.5);

    
% 输入起点
start = input('请输入起点（格式为 [x1 y1]）：');

% 输入终点
goal = input('请输入终点（格式为 [x2 y2]）：');

% 2. 计算势场
potential = navfn_potential_field.compute_potential(smoothed_map, goal);

% 3. 生成路径
path = navfn_path_generator.gradient_descent(potential, start, goal,500);

% 4. 可视化
navfn_path_generator.plot_path_comparison(path, smoothed_map);
