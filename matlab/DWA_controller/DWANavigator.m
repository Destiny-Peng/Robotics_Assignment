classdef DWANavigator
    % DWANavigator - Dynamic Window Approach路径规划算法的MATLAB实现
    
    properties
        % 机器人参数
        radius = 0.178;            % 机器人半径 [m]
        max_speed = 1.0;         % 最大速度 [m/s]
        min_speed = -0.5;         % 最小速度 [m/s]
        max_omega = 40.0*pi/180; % 最大角速度 [rad/s]
        max_accel = 0.2;          % 最大加速度 [m/ss]
        max_domega = 40.0*pi/180; % 最大角加速度 [rad/ss]
        v_resolution = 0.05;      % 速度分辨率 [m/s]
        omega_resolution = pi/180; % 角速度分辨率 [rad/s]
        dt = 0.25;                 % 时间步长 [s]
        predict_time = 3.0;       % 预测时间 [s]
        
        % 代价函数权重
        to_goal_cost_gain = 1.0;  % 目标代价增益
        speed_cost_gain = 1.0;     % 速度代价增益
        obstacle_cost_gain = 1.0;  % 障碍物代价增益
        
        % 状态变量 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        state = zeros(1,5);
        
        % 目标位置 [x(m), y(m)]
        goal = zeros(1,2);
        
        % 障碍物列表 [x(m) y(m)]
        obstacles = [];
        
        % 轨迹记录
        trajectory = [];
    end
    
    methods
        function obj = DWANavigator(initial_state, goal_pos)
            % 构造函数
            % 输入:
            %   initial_state - 初始状态 [x,y,yaw,v,omega]
            %   goal_pos - 目标位置 [x,y]
            
            obj.state = initial_state;
            obj.goal = goal_pos;
            obj.trajectory = initial_state;
        end
        
        function [best_v, best_omega] = plan(obj)
            % 执行DWA路径规划，返回最佳速度和角速度
            
            % 计算动态窗口
            [v_min, v_max, omega_min, omega_max] = obj.calc_dynamic_window();
            
            % 在动态窗口内采样速度
            [v_samples, omega_samples] = obj.sample_velocity_space(...
                v_min, v_max, omega_min, omega_max);
            
            % 评估每条轨迹并选择最佳速度
            [best_v, best_omega] = obj.evaluate_trajectories(...
                v_samples, omega_samples);
        end
        
        function obj = update_state(obj, v, omega)
            % 更新机器人状态
            % 输入:
            %   v - 速度 [m/s]
            %   omega - 角速度 [rad/s]
            
            obj.state = obj.motion(obj.state, [v, omega], obj.dt);
            obj.trajectory = [obj.trajectory; obj.state];
        end
        
        function reached = is_goal_reached(obj, threshold)
            % 检查是否到达目标
            % 输入:
            %   threshold - 到达阈值 [m]
            % 输出:
            %   reached - 布尔值，表示是否到达
            
            if nargin < 2
                threshold = obj.radius;
            end
            
            reached = norm(obj.state(1:2) - obj.goal) < threshold;
        end
        
        function plot_navigation(obj)
            % 可视化导航情况
            
            % 绘制当前位置
            plot(obj.state(1), obj.state(2), 'ro', 'DisplayName', 'Current State'); hold on;
            
            % 绘制目标位置
            plot(obj.goal(1), obj.goal(2), 'gx', 'DisplayName', 'Goal'); hold on;
            
            % 绘制障碍物
            if ~isempty(obj.obstacles)
                plot(obj.obstacles(:,1), obj.obstacles(:,2), 'ks', 'DisplayName', 'Obstacles'); hold on;
            end
            
            % 绘制轨迹
            if size(obj.trajectory,1) > 1
                plot(obj.trajectory(:,1), obj.trajectory(:,2), 'b-', 'DisplayName', 'Trajectory'); hold on;
            end
            
            % 绘制机器人
            obj.draw_robot();
            
            % 设置图例
            legend('show','Location', 'best');
            
            axis equal;
            grid on;
            xlabel('X [m]');
            ylabel('Y [m]');
            title('Dynamic Window Approach Navigation');
            drawnow;
            hold off;
        end
        
        function draw_robot(obj)
            % 绘制机器人
            
            angle = 0:0.1:2*pi;
            x = obj.state(1) + obj.radius * cos(angle);
            y = obj.state(2) + obj.radius * sin(angle);
            plot(x, y, 'r-','DisplayName','Robot'); hold on;
            
            % 绘制方向
            plot([obj.state(1), obj.state(1)+obj.radius*cos(obj.state(3))*10], ...
                 [obj.state(2), obj.state(2)+obj.radius*sin(obj.state(3))*10], 'r-','DisplayName','Direction');hold on; 
        end
    end
    
    methods (Access = private)
        function [v_min, v_max, omega_min, omega_max] = calc_dynamic_window(obj)
            % 基于机器人的运动限制计算动态窗口
            
            % 速度限制
            Vs = [obj.min_speed, obj.max_speed, ...
                  -obj.max_omega, obj.max_omega];
            
            % 基于加速度的动态窗口
            Vd = [obj.state(4) - obj.max_accel * obj.dt, ...
                  obj.state(4) + obj.max_accel * obj.dt, ...
                  obj.state(5) - obj.max_domega * obj.dt, ...
                  obj.state(5) + obj.max_domega * obj.dt];
            
            % 最终动态窗口
            v_min = max(Vs(1), Vd(1));
            v_max = min(Vs(2), Vd(2));
            omega_min = max(Vs(3), Vd(3));
            omega_max = min(Vs(4), Vd(4));
        end
        
        function [v_samples, omega_samples] = sample_velocity_space(obj, v_min, v_max, omega_min, omega_max)
            % 在动态窗口内生成速度样本
            
            % 生成速度样本
            v_samples = v_min:obj.v_resolution:v_max;
            if isempty(v_samples)
                v_samples = [v_min, v_max];
            end
            
            % 生成角速度样本
            omega_samples = omega_min:obj.omega_resolution:omega_max;
            if isempty(omega_samples)
                omega_samples = [omega_min, omega_max];
            end
            
            % 确保至少有一个样本
            if length(v_samples) < 2
                v_samples = [v_samples, v_samples];
            end
            if length(omega_samples) < 2
                omega_samples = [omega_samples, omega_samples];
            end
        end
        
        function [best_v, best_omega] = evaluate_trajectories(obj, v_samples, omega_samples)
            % 评估所有可能的轨迹并选择最佳速度
            
            max_score = -inf;
            best_v = 0.0;
            best_omega = 0.0;
            
            for v = v_samples
                for omega = omega_samples
                    % 预测轨迹
                    traj = obj.predict_trajectory(obj.state, v, omega);
                    plot(traj(:,1),traj(:,2),'y-','HandleVisibility', 'off');hold on;
                    
                    % 计算代价
                    to_goal_cost = obj.calc_to_goal_cost(traj);
                    speed_cost = obj.calc_speed_cost(traj);
                    obstacle_cost = obj.calc_obstacle_cost(traj);
                    
                    % 总代价
                    score = obj.to_goal_cost_gain * to_goal_cost + ...
                            obj.speed_cost_gain * speed_cost + ...
                            obj.obstacle_cost_gain * obstacle_cost;
                    
                    % 选择最佳轨迹
                    if score > max_score
                        max_score = score;
                        best_v = v;
                        best_omega = omega;
                    end
                end
            end
             plot(NaN, NaN, 'y-', 'DisplayName', 'Predicted Trajectories'); % 用 NaN 创建图例项
        end
        
        function traj = predict_trajectory(obj, state, v, omega)
            % 预测给定速度下的轨迹
            
            traj = state;
            time = 0;
            
            while time <= obj.predict_time
                state = obj.motion(state, [v, omega], obj.dt);
                traj = [traj; state];
                time = time + obj.dt;
            end
        end
        
        function state = motion(~, state, u, dt)
            % 机器人运动模型
            
            v = u(1);       % 速度 [m/s]
            omega = u(2); % 角速度 [rad/s]
            
            if omega ~= 0
                state(1) = state(1) + v/omega * (sin(state(3) + omega*dt) - sin(state(3)));
                state(2) = state(2) + v/omega * (cos(state(3)) - cos(state(3) + omega*dt));
            else
                state(1) = state(1) + v * dt * cos(state(3));
                state(2) = state(2) + v * dt * sin(state(3));
            end
            state(3) = state(3) + omega * dt;
            state(4) = v;
            state(5) = omega;
        end
        
        function cost = calc_to_goal_cost(obj, traj)
            % 计算到目标的代价
            
            dx = obj.goal(1) - traj(end,1);
            dy = obj.goal(2) - traj(end,2);
            cost = -sqrt(dx^2 + dy^2); % 负号表示距离越小，代价越小(得分越高)
        end
        
        function cost = calc_speed_cost(obj, traj)
            % 计算速度代价 (鼓励高速运动)
            cost = traj(end,4); % 速度越大，代价越小(得分越高)
        end
        
        function cost = calc_obstacle_cost(obj, traj)
            % 计算障碍物代价
            
            if isempty(obj.obstacles)
                cost = 0;
                return;
            end
            
            min_dist = inf;
            
            for i = 1:size(traj,1)
                for j = 1:size(obj.obstacles,1)
                    dist = sqrt((traj(i,1)-obj.obstacles(j,1))^2 + (traj(i,2)-obj.obstacles(j,2))^2);
                    
                    % 如果碰撞，返回极小值
                    if dist <= obj.radius
                        cost = -inf;
                        return;
                    end
                    
                    if dist < min_dist
                        min_dist = dist;
                    end
                end
            end
            
            cost = min_dist; % 距离障碍物越远，代价越小(得分越高)
        end
    end
end
