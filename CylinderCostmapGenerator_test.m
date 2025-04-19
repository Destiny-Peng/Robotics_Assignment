% 创建生成器实例
generator = CylinderCostmapGenerator('mapSize', [100, 100], ...
                                   'numCylinders', 64, ...
                                   'minRadius', 3, ...
                                   'maxRadius', 6);

% 生成costmap
[costmap, cylinderInfo] = generator.generate();

% 可视化
generator.visualize(costmap);

% 打印圆柱信息
disp('生成的圆柱信息:');
disp(cylinderInfo);