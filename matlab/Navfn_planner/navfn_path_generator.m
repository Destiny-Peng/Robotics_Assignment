classdef navfn_path_generator
    % 路径生成模块：梯度下降 + 局部极小值处理
    
    methods (Static)
        function path = gradient_descent(potential, start, goal, max_steps)
            [rows, cols] = size(potential);
            path = start;
            current = start;
            step = 0;
            stuck_counter = 0;
            max_stuck = 20;  % 增大允许的震荡次数
            
            % 预计算梯度（注意方向修正）
            [gx, gy] = gradient(potential);
            gx = -gx;  % 取负梯度
            gy = -gy;
            
            while step < max_steps
                % 检查是否到达目标（允许3x3邻域内终止）
                if norm(current - goal) <= 2
                    path(end+1,:) = goal;
                    break;
                end
                
                % 获取当前梯度方向
                dx = gx(current(1), current(2));
                dy = gy(current(1), current(2));
                
                % 归一化梯度方向
                norm_factor = hypot(dx, dy);
                if norm_factor > 1e-3  % 避免除以零
                    dx = dx / norm_factor;
                    dy = dy / norm_factor;
                else
                    % 零梯度时启用增强逃逸策略
                    [dx, dy] = navfn_path_generator.escape_local_min(current, path, rows, cols);
                    stuck_counter = stuck_counter + 1;
                end
                
                % 计算下一步位置（允许浮点坐标）
                next = current + [dy, dx];  % 注意：dy对应行，dx对应列
                next(1) = max(min(round(next(1)), rows), 1);
                next(2) = max(min(round(next(2)), cols), 1);
                
                % 检测震荡（重复访问相同区域）
                if size(path,1) > 10 && sum(ismember(path(max(1,end-10):end,:), next, 'rows')) > 2
                    stuck_counter = stuck_counter + 1;
                    [dx, dy] = navfn_path_generator.escape_local_min(current, path, rows, cols);
                    next = current + [dy, dx];
                    next(1) = max(min(round(next(1)), rows), 1);
                    next(2) = max(min(round(next(2)), cols), 1);
                end
                
                % 强制终止条件
                if stuck_counter > max_stuck
                    disp('Warning: Escaped after max stuck steps.');
                    break;
                end
                
                % 更新路径
                path(end+1,:) = next;
                current = next;
                step = step + 1;
            end
        end
        
        function [dx, dy] = escape_local_min(current, path, rows, cols)
            % 增强型局部极小值逃逸策略
            % 策略1：尝试沿历史路径的垂直方向移动
            if size(path,1) >= 2
                dir = path(end,:) - path(end-1,:);
                perp_dir = [-dir(2), dir(1)];  % 垂直方向
                if any(perp_dir ~= 0)
                    perp_dir = perp_dir / norm(perp_dir);
                    dx = perp_dir(2);
                    dy = perp_dir(1);
                    return;
                end
            end
            
            % 策略2：随机方向+偏向目标点
            target_dir = [randn(), randn()];
            target_dir = target_dir / norm(target_dir);
            dx = target_dir(2);
            dy = target_dir(1);
        end
        
        function plot_path_comparison(path, costmap)
            % 可视化原始路径与简化路径
            figure;
            imagesc(costmap);
            colormap(flipud(gray));
            hold on;
            
            % 绘制路径（红色）
            plot(path(:,2), path(:,1), 'r-', 'LineWidth', 1.5);
            
            % 标记起点和终点
            plot(path(1,2), path(1,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            plot(path(end,2), path(end,1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            
            legend('Path', 'Start', 'Goal');
            title('Path Comparison');
            axis equal;
        end
    end
end