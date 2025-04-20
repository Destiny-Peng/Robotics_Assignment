# Navfn路径规划器

## 概述

这个目录包含了基于导航函数(Navigation Function)的路径规划算法的MATLAB实现。Navfn规划器能够在给定的代价地图上生成从起点到目标点的最优路径，通常用于机器人导航系统中。

## 文件结构

本目录包含以下文件：

- `CylinderCostmapGenerator.m` - 用于创建包含圆柱形障碍物的代价地图的生成器
- `CylinderCostmapGenerator_test.m` - 圆柱形代价地图生成器的测试脚本
- `main.m` - 主程序脚本，用于展示完整的Navfn规划流程
- `navfn_path_generator.m` - 基于导航函数生成路径的核心算法
- `navfn_path_generator_test.m` - 路径生成器的测试脚本
- `navfn_potential_field.m` - 构建导航函数势场的算法实现
- `navfn_potential_field_test.m` - 势场构建算法的测试脚本
- `navfn_preprocessing.m` - 用于预处理代价地图的函数
- `navfn_preprocessing_tesst.m` - 预处理函数的测试脚本

## 使用方法

### 快速开始

   运行`main.m`脚本查看完整的Navfn规划器演示

### 自定义使用

1. 使用`CylinderCostmapGenerator.m`创建自定义代价地图
2. 通过`navfn_preprocessing.m`预处理代价地图
3. 使用`navfn_potential_field.m`构建导航函数势场
4. 通过`navfn_path_generator.m`基于势场生成最优路径
