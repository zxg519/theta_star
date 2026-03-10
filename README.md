# theta_star

optimal path finding algorithm using theta*, which can vaoid the searching limitation in A* in regular grid.
I used easyX to draw picture in this program.

xgzhang@seu.edu.cn
2026/03/10

/*******************************************************
 *  最终版：A* / Theta* + EasyX
 *  修复：
 *    - A*：禁止对角线穿过两个障碍顶点
 *    - Theta*：禁止线段与任何障碍格子相交（像素级采样）
 *  功能：手动设置障碍、起点、终点；实时切换算法
 ******************************************************/
