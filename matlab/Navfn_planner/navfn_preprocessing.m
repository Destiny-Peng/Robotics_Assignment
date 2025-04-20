classdef navfn_preprocessing
    % 代价地图预处理模块：膨胀障碍物 + 代价值归一化
    
    methods (Static)
        function inflated_map = inflate_obstacles(costmap, inflation_radius)
            % 膨胀障碍物
            % 输入：
            %   costmap: 原始地图（0=自由, 100=障碍）
            %   inflation_radius: 膨胀半径（单位：栅格数）
            % 输出：
            %   inflated_map: 膨胀后的地图（0=自由, 100=障碍）
            
            % 二值化地图：障碍物为1，其他为0
            binary_obstacles = costmap >= 100;
            
            % 创建圆形膨胀结构元素
            [x, y] = meshgrid(-inflation_radius:inflation_radius);
            se = (x.^2 + y.^2) <= inflation_radius^2;
            
            % 膨胀操作
            inflated_obstacles = imdilate(binary_obstacles, se);
            
            % 恢复为NavFn格式：自由=0，障碍=100
            inflated_map = costmap;
            inflated_map(inflated_obstacles) = 100;
        end
        
        function normalized_map = normalize_costmap(costmap, max_cost)
            % 代价值归一化
            % 输入：
            %   costmap: 原始地图（0=自由, 100=障碍）
            %   max_cost: 归一化后的最大值（默认255）
            % 输出：
            %   normalized_map: 归一化后的地图（0~max_cost）
            
            if nargin < 2
                max_cost = 255;
            end
            
            % 将原始代价值线性映射到[0, max_cost]
            normalized_map = double(costmap);
            normalized_map = normalized_map / 100 * max_cost;
            normalized_map = min(max(normalized_map, 0), max_cost);
        end
        
        function smoothed_map = smooth_edges(costmap, sigma)
            % 边缘平滑（高斯模糊）
            % 输入：
            %   costmap: 膨胀后的地图
            %   sigma: 高斯核标准差（默认1.0）
            % 输出：
            %   smoothed_map: 平滑后的地图
            
            if nargin < 2
                sigma = 1.0;
            end
            
            % 对非障碍物区域进行高斯模糊
            obstacle_mask = costmap >= 100;
            smoothed_map = imgaussfilt(double(costmap), sigma);
            smoothed_map(obstacle_mask) = costmap(obstacle_mask); % 保持障碍物不变
        end
    end
end