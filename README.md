# theta_star
## introduction
 This program compares the performance of A* and Theta*, in regular grid scenario. <br>
 —— A*，disallow overpass the corner of Non‑passable grid;  
 —— Theta*, disallow overpass the corner of Non‑passable grid;  
 You can manully set non-paasable grids, or directly load maps stored in ../mapfiles, and you can save the map you set to file.  You can manully set the start grid and the end grid.   
 When you press [A*], it will execute A* algorithm, and draw the result on the map. When you press [Theta*], it will execute Theta* algorithm, and draw the result on the map.  
 
## language and GUI lib used
Optimal path finding algorithm using theta*, which can vaoid the searching direction limitation in A* in regular grids.
| description| values       |
|------------|--------------|
|Programming Language | C++ |
|GUI Lib used| EasyX        |
|algorithms compared| A* vs Theta* |

## Author contact
xgzhang@seu.edu.cn <br>
2026/03/10 <br>

## 测试结果：
### case 1
| A* Result | Theta* Result |
|-----------|---------------|
| ![image](https://github.com/zxg519/theta_star/blob/main/A_star_result-2.png) | ![image](https://github.com/zxg519/theta_star/blob/main/theta_star_result-2.png) |

### case 2
| A* Result | Theta* Result |
|-----------|---------------|
| ![image](https://github.com/zxg519/theta_star/blob/main/A_star_result-1.png) | ![image](https://github.com/zxg519/theta_star/blob/main/theta_star_result-1.png) |

