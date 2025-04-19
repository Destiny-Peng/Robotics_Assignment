function path = navfn_planner(costmap, start, goal)
    % 1. 预处理代价地图
    inflated_map = inflate_obstacles(costmap);
    
    % 2. 初始化势场
    potential = Inf(size(costmap));
    potential(goal(1), goal(2)) = 0;
    
    % 3. 扩散算法计算势场
    potential = compute_potential_field(inflated_map, goal, potential);
    
    % 4. 梯度下降生成路径
    path = gradient_descent(start, potential);
end