classdef CylinderCostmapGenerator
    % CYLINDERCOSTMAPGENERATOR 随机生成包含圆柱形障碍物的costmap
    
    properties
        mapSize = [100, 100];      % 地图尺寸 [行, 列]
        numCylinders = 5;          % 圆柱障碍物数量
        minRadius = 5;             % 圆柱最小半径
        maxRadius = 15;            % 圆柱最大半径
        maxCost = 100;             % 障碍物代价值
        safetyMargin = 2;          % 安全边距
    end
    
    methods
        function obj = CylinderCostmapGenerator(varargin)
            % 构造函数，可设置参数
            for i = 1:2:nargin
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function [costmap, cylinderInfo] = generate(obj)
            % 生成costmap
            costmap = zeros(obj.mapSize);
            cylinderInfo = struct('center', {}, 'radius', {});
            
            for i = 1:obj.numCylinders
                % 随机生成圆柱参数
                radius = randi([obj.minRadius, obj.maxRadius]);
                centerX = randi([radius+1, obj.mapSize(2)-radius-1]);
                centerY = randi([radius+1, obj.mapSize(1)-radius-1]);
                
                % 检查是否与其他圆柱重叠
                validPosition = true;
                for j = 1:length(cylinderInfo)
                    dist = sqrt((centerX - cylinderInfo(j).center(1))^2 + ...
                                (centerY - cylinderInfo(j).center(2))^2);
                    if dist < (radius + cylinderInfo(j).radius + obj.safetyMargin)
                        validPosition = false;
                        break;
                    end
                end
                
                % 如果位置有效，添加圆柱
                if validPosition
                    costmap = obj.addCylinder(costmap, [centerY, centerX], radius);
                    cylinderInfo(end+1) = struct('center', [centerY, centerX], 'radius', radius);
                else
                    % 如果位置无效，减少计数重试
                    i = i - 1;
                end
            end
        end
        
        function costmap = addCylinder(obj, costmap, center, radius)
            % 在costmap中添加一个圆柱形障碍物
            [rows, cols] = size(costmap);
            [X, Y] = meshgrid(1:cols, 1:rows);
            
            % 计算每个点到圆心的距离
            distFromCenter = sqrt((X - center(2)).^2 + (Y - center(1)).^2);
            
            % 设置障碍物区域
            obstacleRegion = distFromCenter <= radius;
            costmap(obstacleRegion) = obj.maxCost;
        end
        
        function visualize(obj, costmap)
            % 可视化costmap
            figure;
            imagesc(costmap);
            axis equal tight;
            colorbar;
            title('随机圆柱障碍物Costmap');
            xlabel('X坐标');
            ylabel('Y坐标');
            colormap(flipud(gray)); % 障碍物显示为深色
        end
    end
end