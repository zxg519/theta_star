# theta_star
## introduction
Optimal path finding algorithm using theta*, which can vaoid the searching direction limitation in A* in regular grids.
| description| values|
|-------|-------------------|
|Programming Language | C++ |
|-------|-------------------|
|GUI Lib used| EasyX        |
|-------|-------------------|
|algorithms compared| A* vs Theta* |
|-------|------------------|

xgzhang@seu.edu.cn <br>
2026/03/10 <br>

/*******************************************************
 *  最终版：A* / Theta* + EasyX
 *  修复：
 *    - A*：禁止对角线穿过两个障碍顶点
 *    - Theta*：禁止线段与任何障碍格子相交（像素级采样）
 *  功能：手动设置障碍、起点、终点；实时切换算法
 ******************************************************/

## 测试结果：
### case 1
| A* Result | Theta* Result |
|-----------|---------------|
| ![image](https://github.com/zxg519/theta_star/blob/main/A_star_result-2.png) | ![image](https://github.com/zxg519/theta_star/blob/main/theta_star_result-2.png) |

### case 2
| A* Result | Theta* Result |
|-----------|---------------|
| ![image](https://github.com/zxg519/theta_star/blob/main/A_star_result-1.png) | ![image](https://github.com/zxg519/theta_star/blob/main/theta_star_result-1.png) |

