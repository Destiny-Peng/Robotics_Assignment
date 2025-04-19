% 1. 创建测试地图（0=自由, 100=障碍）
% costmap = zeros(20, 20);
% costmap(5:15, 8) = 100;  % 垂直障碍
% costmap(10, 3:17) = 100; % 水平障碍（十字形）

% 2. 预处理步骤
inflated_map = navfn_preprocessing.inflate_obstacles(costmap, 1);       % 膨胀半径=2
normalized_map = navfn_preprocessing.normalize_costmap(inflated_map);   % 归一化到0-255
smoothed_map = navfn_preprocessing.smooth_edges(inflated_map, 1.5);   % 高斯平滑

% 3. 可视化对比
figure;
subplot(2,2,1); imagesc(costmap); title('原始地图'); axis equal;
subplot(2,2,2); imagesc(inflated_map); title('膨胀后地图'); axis equal;
subplot(2,2,3); imagesc(normalized_map); title('归一化地图'); axis equal;
subplot(2,2,4); imagesc(smoothed_map); title('平滑后地图'); axis equal;
% colormap(gray);