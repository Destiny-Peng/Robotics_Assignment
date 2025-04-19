classdef navfn_potential_field
    % 势场计算模块：基于优先级队列的Dijkstra扩散算法
    
    methods (Static)
        function potential = compute_potential(costmap, goal, max_iter)
            % 计算势场
            % 输入：
            %   costmap: 预处理后的地图（值越大代价越高）
            %   goal: 目标点坐标 [row, col]
            %   max_iter: 最大迭代次数（可选）
            % 输出：
            %   potential: 势场矩阵（与costmap同尺寸）
            
            [rows, cols] = size(costmap);
            potential = inf(rows, cols);
            potential(goal(1), goal(2)) = 0; % 目标点势能为0
            
            % 优先级队列：元胞数组手动维护优先级
            priority_queue = {};
            priority_queue{1} = {0, goal}; % {势能, 坐标}
            
            % 定义8邻域偏移量
            neighbors = [-1,0; 1,0; 0,-1; 0,1; -1,-1; -1,1; 1,-1; 1,1];
            
            iter = 0;
            while ~isempty(priority_queue) && (nargin < 3 || iter < max_iter)
                % 找到当前队列中势能最小的元素
                [~, idx] = min(cellfun(@(x) x{1}, priority_queue));
                current_item = priority_queue{idx};
                current_potential = current_item{1};
                current = current_item{2};
                
                % 从队列中移除该元素
                priority_queue(idx) = [];
                
                % 检查8邻域
                for k = 1:size(neighbors, 1)
                    nx = current(1) + neighbors(k, 1);
                    ny = current(2) + neighbors(k, 2);
                    
                    % 边界检查
                    if nx < 1 || nx > rows || ny < 1 || ny > cols
                        continue;
                    end
                    
                    % 跳过障碍物
                    if costmap(nx, ny) >= 100
                        continue;
                    end
                    
                    % 计算新势能（对角线代价乘sqrt(2)）
                    if abs(neighbors(k,1)) + abs(neighbors(k,2)) == 2
                        step_cost = costmap(nx, ny) * 1.4142;
                    else
                        step_cost = costmap(nx, ny);
                    end
                    new_potential = current_potential + step_cost+1;
                    
                    % 如果新势能更低，则更新
                    if new_potential < potential(nx, ny)
                        potential(nx, ny) = new_potential;
                        
                        % 添加到队列
                        priority_queue{end+1} = {new_potential, [nx, ny]};
                    end
                end
                iter = iter + 1;
            end
        end
        
        function plot_potential(potential, costmap)
            % 可视化势场
            figure;
            subplot(1,2,1);
            imagesc(potential);
            colorbar;
            title('Potential Field');
            axis equal;
            
            subplot(1,2,2);
            imagesc(costmap);
            hold on;
            [gx, gy] = gradient(potential);
            quiver(gy, gx, 'r'); % 绘制势场梯度
            title('Costmap with Gradient Vectors');
            axis equal;
            colormap(jet);
        end
    end
end