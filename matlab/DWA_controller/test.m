% 初始化导航器
initial_state = [0.0, 0.0, pi/2, 0.0, 0.0]; % [x,y,yaw,v,omega]
goal_pos = [10.0, 10.0];
dwa = DWANavigator(initial_state, goal_pos);

% 设置障碍物
dwa.obstacles = [...
    1.0, 2.0;
    3.0, 3.0;
    3.0, 4.0;
    5.0, 5.0;
    7.0, 7.0;
    8.0, 8.0;
    8.0, 9.0];

% 调整参数
dwa.max_speed = 1.5;
dwa.predict_time = 4.0;
dwa.obstacle_cost_gain = 2.0;

% 主循环
figure;
while ~dwa.is_goal_reached()
    % 规划路径
    [v, omega] = dwa.plan();
    
    % 更新状态
    dwa = dwa.update_state(v, omega);
    
    % 可视化
    dwa.plot_navigation();
    
    % 暂停以便观察
    pause(0.1);
end

disp('Goal reached!');