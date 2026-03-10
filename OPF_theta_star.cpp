/*******************************************************
 *  最终版：A* / Theta* + EasyX
 *  修复：
 *    - A*：禁止对角线穿过两个障碍顶点
 *    - Theta*：禁止线段与任何障碍格子相交（像素级采样）
 *  功能：手动设置障碍、起点、终点；实时切换算法
 ******************************************************/
#define  _CRT_SECURE_NO_WARNINGS
#include <graphics.h>
#include <conio.h>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <string>
#include <sstream>
#include <locale>
#include <codecvt>

using namespace std;

const int CELL = 40;          // 网格正方形的边长
const int COLS = 20;
const int ROWS = 15;
const int WIDTH = COLS * CELL+160;
const int HEIGHT = ROWS * CELL+50;

struct Node {
    int x;                        // 网格的列号，行号
    int y;                        // 网格的列号，行号
    double g = 1e9, f = 1e9;      // 当前代价， f 表示目前代价，g表示预期后续最小代价  总代价 = f + g
    Node* parent = nullptr;               
    bool pass = true;                     // 如果可通行，为true, 不可通行为false
};
Node grid[ROWS][COLS];             // 整个地图！

int dx8[8] = { -1,1,0,0,-1,-1,1,1 };    // 水平方向，8领域的网格变化（实际是列数的变化）
int dy8[8] = { 0,0,-1,1,-1,1,-1,1 };    // 垂直方向，8领域的网格变化（实际是行数的变化）

int px(int x) { return x * CELL + CELL / 2; }
int py(int y) { return y * CELL + CELL / 2; }

double dist(int x1, int y1, int x2, int y2) 
{
    return hypot(x1 - x2, y1 - y2);
}

/* ================= 按钮 ================= */
struct Button 
{
    int left, top, right, bottom;
    const wchar_t* text;
    COLORREF color;

    // 判断是否被点击了，其实这儿可以直接加回调函数！！！
    bool click(int mx, int my) const 
    {
        return mx >= left && mx <= right && my >= top && my <= bottom;
    }
    //绘制函数，我们没有正儿八经设置window机制，否则可以供主窗口直接调用就可以了
    void draw() const 
    {
        setfillcolor(color);
        solidrectangle(left, top, right, bottom);
        setlinecolor(BLACK); rectangle(left, top, right, bottom);
        setbkmode(TRANSPARENT);
        settextstyle(20, 0, _T("Consolas"));
        int tw = textwidth(text), th = textheight(text);
        outtextxy((left + right - tw) / 2, (top + bottom - th) / 2, text);
    }
};

Button btnAstar{ 830,10,930,50, _T("A*") , DARKGRAY };
Button btnTheta{ 830,80,930,120, _T("θ*") , DARKGRAY };
Button btnBarrier{ 830, 150, 930, 190, _T("设置障碍点"), DARKGRAY };
Button btnStart{ 830, 220, 930, 260, _T("设置起点"), DARKGRAY };
Button btnGoal{ 830, 290, 930, 330, _T("设置终点"), DARKGRAY };
Button btnSave{ 830, 370, 930, 410, _T("存储地图"), BROWN };
Button btnLoad{ 830, 440, 930, 480, _T("调入地图"), BROWN };
Button btnClear{ 830, 510, 930, 550, _T("清除地图"), BROWN };
Button btnAbout{ 830, 580, 930, 620, _T("关于..."),  BROWN };

enum Mode { NONE, ASTAR, THETA, BARRIER, SET_START, SET_GOAL };
Mode mode = NONE;

/* ================= 地图初始化（全空） ================= */
void initMap() 
{
    for (int y = 0; y < ROWS; ++y)
    {
        for (int x = 0; x < COLS; ++x)
        {
            grid[y][x] = Node{ x,y,1e9,1e9,nullptr,true };
        }
    }
}

void drawMap() 
{
    cleardevice();
    setbkcolor(RGB(255,255, 255));
    setlinecolor(DARKGRAY);
    setfillcolor(DARKGRAY);
    for (int y = 0; y < ROWS; ++y)
    {
        for (int x = 0; x < COLS; ++x)
        {
            rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL);
            if (!grid[y][x].pass)
            {
                solidrectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL);
                // fillrectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL);
            }
        }
    }

    btnAstar.draw();
    btnTheta.draw(); 
    btnBarrier.draw();
    btnStart.draw(); 
    btnGoal.draw(); 
    btnSave.draw();
    btnLoad.draw();
    btnClear.draw();
    btnAbout.draw();
}

void drawPath(const std::vector<Node*>& path, COLORREF color) 
{
    if (path.size() < 2) 
        return;

    setlinecolor(color);
    setlinestyle(PS_SOLID, 3);
    cout << "画路径(（列，行）格式):" << endl;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        cout << "(" << path[i]->x << "," << path[i]->y << "),";
        line(px(path[i]->x), py(path[i]->y),
            px(path[i + 1]->x), py(path[i + 1]->y));
    }
    cout << "(" << path[path.size() - 1]->x << "," << path[path.size() - 1]->y << "),";
    cout << endl;
    Node* start =path[0];
    Node* goal = path[path.size()-1];
    setfillcolor(GREEN);  
    solidcircle(px(start->x), py(start->y), CELL / 3);
    setfillcolor(MAGENTA); 
    solidcircle(px(goal->x), py(goal->y), CELL / 3);
    setlinestyle(PS_SOLID, 1);
}


/* ========== 像素级线段与障碍相交检测（Theta* 用），基于 Amanatides-Woo 算法  ========== */
/*
1. 关于 “t 值” 的含义
    这里的 “t” 是一个标准化的参数，用于描述直线上的点与起点、终点的相对位置，取值范围为 [0, 1]：
    当 t=0 时，对应直线的起点(x0,y0)；
    当 t=1 时，对应直线的终点(x1,y1)；
    当 0 < t < 1 时，对应起点和终点之间的某个点。

2. 算法中计算 “穿过网格边界的 t 值”，本质是在计算：
   直线从起点出发后，第一次到达某个网格边界（x 方向或 y 方向的网格线）时，所经过的 “距离比例”。

3.为何 dx=0 时，txDelta 要设置为无穷大？
   dx 是直线在 x 方向的总增量（dx=x1−x0）：

   当 dx=0 时，直线是垂直的（x 坐标始终不变），不会在 x 方向上移动，因此永远不会穿过 x 方向的网格边界。
      txDelta 表示 “x 方向上每穿过一个网格边界，t 值的增量”。对于垂直直线：
      由于不会穿过 x 方向的网格边界，“穿过一个 x 边界的 t 值增量” 是没有意义的，因此设置为无穷大（INFINITY）。

*/
#if 0
/*bool lineIntersectsObstacle(int x0,
                            int y0,  // 开始点网格对应的编号（x0，y0)表示（列号，行号）都是从0开始的 
                            int x1, 
                            int y1)  // 结束点网格对应的编号（x1，y1)表示（列号，行号）都是从0开始的
{
    std::vector<Node*> path;

    // 符号函数，>0:1;     <0:-1;    0:0
    auto sign = [](int delta)->int  {
            return delta > 0 ? 1 : (delta < 0 ? -1 : 0);
     };
    auto get_center_coor = [](int grid_xy)->double { // 用网格的水平/垂直编号，得到其坐标，此处假设每个网格都是长宽为1
        return grid_xy + 0.5;
    };

    // 起点和终点的网格坐标(实际就是列好（x)和行号(y)
    int currentX = x0;
    int currentY = y0;         // 起点
    int targetX = x1; 
    int targetY = y1;          // 终点

    // 计算起点和终点的实际坐标，由于我们始终设定起点和终点为网格中心
    double xx0 = get_center_coor(x0);
    double yy0 = get_center_coor(y0);
    double xx1 = get_center_coor(x1);
    double yy1 = get_center_coor(y1);

    // 方向向量(需要移动的总量）
    double dx = xx1 - xx0;
    double dy = yy1 - yy0;

    // 步进方向（1或-1）
    int stepX = sign(dx);
    int stepY = sign(dy);

    // 若起点和终点在同一网格，直接返回
    if (currentX == targetX && currentY == targetY) {
        // 实际应该不会执行到这，此似乎肯定不会碰到障碍物！
        return false;
    }

    // 计算穿过x/y方向网格边界的步长增量,总共dx,dy的移动量，一步就是1/dx, 1/dy, 注意：这个是绝对量
    double txDelta = (dx != 0) ? std::abs(1.0f / dx) : INFINITY;  // txDelta（x 方向穿过一个网格的 t 值增量）由 dx 决定（txDelta = 1/|dx|）；
    double tyDelta = (dy != 0) ? std::abs(1.0f / dy) : INFINITY;  // tyDelta（y 方向穿过一个网格的 t 值增量）由 dy 决定（tyDelta = 1/|dy|）。


    // 计算初始穿过第一个网格边界的t值
    double tx=0, ty=0;
    // 计算x方向
    if (dx > 0) {
        // 向右移动：下一个x边界是当前网格右边界 (currentX + 1.0),实际上就是起始点所在网格的边线（如往右走，就是右侧；如往左，就是左侧侧线）
        tx = (currentX + 1.0f - xx0) * txDelta;
    }
    else if (dx < 0) {
        // 向左移动：下一个x边界是当前网格左边界 (currentX)
        tx = (xx0 - currentX) * txDelta;
    }
    else {
        tx = INFINITY;
    }

    // 计算y方向
    if (dy > 0) {
        // 向上移动：下一个y边界是当前网格上边界 (currentY + 1.0)
        ty = (currentY + 1.0f - yy0) * tyDelta;
    }
    else if (dy < 0) {
        // 向下移动：下一个y边界是当前网格下边界 (currentY)
        ty = (yy0 - currentY) * tyDelta;
    }
    else {
        ty = INFINITY;
    }
    //=============================================================
     // 遍历所有穿过的网格,****************************  这儿很复杂，我们起点和终点都在网格中心，要计算网格如何跨越的！！！
    
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   | * |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   | @ |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+
            |   |   |   |   |   |   |   |   |   |   |
            +---+---+---+---+---+---+---+---+---+---+

    
    
    // 遍历所有穿过的网格
    while (true) 
    {
        path.emplace_back(currentX, currentY);

        // 到达终点网格时退出
        if (currentX == targetX && currentY == targetY) {
            break;
        }

        // 移动到下一个网格
        if (tx < ty) {
            currentX += stepX;
            tx += txDelta;
        }
        else {
            currentY += stepY;
            ty += tyDelta;
        }
    }
    //============== check every grid =============================================
    int iall = path.size();
    Node* pre = nullptr;
    for (int i = 0; i < iall; ++i)
    {
        Node* p = path[i];
        if (p->pass == false)
            return true;
        
        if (pre)
        {
            // 如果pre 和 p是上下左右关系， 没有问题


            // 如果是斜着的关系，通过判断斜率，

        }


        pre = p;
    }

    return false;

}
*/
#endif

//
// 似乎否水平方向存在阻碍，注意调用了全局网格！！！
//
//  x,y are coordinate for the path intersection with grid line
//  the unit length of one grid == 1
//
bool is_H_blocked(const double x,  //  这儿实际是两个路径点和垂直grid线的交点！
                const double y) 
{

    int col_id = (int)(x+0.1);  //避免 计算误差，x+0.1的裕量，理论上x应该是整数值的，为避免误差，所以，加0.1的松弛量
    int row_id = (int)y;
    struct cell
    {
        int row;
        int col;
    };
    cell cells[4];
    int size = 0;

    cells[size]=cell{row_id ,col_id};  // 垂直线右侧网格
    size++;

    if (col_id - 1 >= 0)
    {
        cells[size] = cell{ row_id ,col_id - 1 };  // 垂直线左侧网格
        size++;
    }
  
    // 特殊情况处理，如果穿越网格交点，要求周围4个邻域都必须可以通行
    if ( y-(int)y  < 1e-9)
    {
        // y 略微比网格点大一点
        cells[size] = cell{ row_id - 1 ,col_id };
        size++;
  
        if (col_id - 1 >= 0)
        {
            cells[size] = cell{ row_id - 1 ,col_id - 1 };
            size++;
        }
    }
    else if (abs((int)(y + 1e-9) - y) <= 1e-9)
    {
        // y 略微比网格点小一点
                // y 略微比网格点大一点
        cells[size] = cell{ row_id + 1 ,col_id };
        size++;

        if (col_id - 1 >= 0)
        {
            cells[size] = cell{ row_id + 1 ,col_id - 1 };
            size++;
        }
    }

    for (int i = 0; i < size; ++i)
    {
        if (!grid[cells[i].row][cells[i].col].pass)
            return true;
    }

    return false;
}

//
// 似乎否垂直方向存在阻碍，注意调用了全局网格！！！
// 
//  x,y are coordinate for the path intersection with grid line
//  the unit length of one grid == 1

bool is_V_blocked(const double x,    //  这儿实际是两个路径点和水平grid线的交点！
                  const double y)
{
    int row_id = (int)(y + 0.1);  //避免 计算误差，x+0.1的裕量
    int col_id = (int)x;
    struct cell
    {
        int row;
        int col;
    };
    cell cells[4];
    int size = 0;

    cells[size] = cell{ row_id ,col_id };
    size++;

    if (row_id - 1 >= 0)
    {
        cells[size] = cell{ row_id -1,col_id  };
        size++;
    }

    // 特殊情况处理，如果穿越网格交点，要求周围4个邻域都必须可以通行
    if (abs((int)x - x) < 1e-9)
    {
        // y 略微比网格点大一点
        cells[size] = cell{ row_id ,col_id-1 };
        size++;

        if (row_id - 1 >= 0)
        {
            cells[size] = cell{ row_id - 1 ,col_id - 1 };
            size++;
        }
    }
    else if (abs((int)(x + 1e-9) - x) < 1e-7)
    {
        // y 略微比网格点小一点
                // y 略微比网格点大一点
        cells[size] = cell{ row_id + 1 ,col_id };
        size++;

        if (col_id - 1 >= 0)
        {
            cells[size] = cell{ row_id + 1 ,col_id - 1 };
            size++;
        }
    }

    for (int i = 0; i < size; ++i)
    {
        if (!grid[cells[i].row][cells[i].col].pass)
            return true;
    }

    return false;
}

/*
  愚蠢版本，但是逻辑简单，
  （1）利用参数方程，直接和水平线/垂直求交，得到穿越的网格

*/
bool lineIntersectsObstacle(int x0,
                            int y0,  // 开始点网格对应的编号（x0，y0)表示（列号，行号）都是从0开始的 
                            int x1,
                            int y1)  // 结束点网格对应的编号（x1，y1)表示（列号，行号）都是从0开始的
{
    
    // 符号函数，>0:1;     <0:-1;    0:0
    auto sign = [](int delta)->int {
        return delta > 0 ? 1 : (delta < 0 ? -1 : 0);
        };
    auto get_center_coor = [](int grid_xy)->double { // 用网格的水平/垂直编号，得到其坐标，此处假设每个网格都是长宽为1
        return grid_xy + 0.5;
        };

    // 起点和终点的网格坐标(实际就是列号（x)和行号(y)
    int currentX = x0;
    int currentY = y0;         // 起点
    int targetX = x1;
    int targetY = y1;          // 终点

    // 计算起点和终点的实际坐标，由于我们始终设定起点和终点为网格中心
    double xx0 = get_center_coor(x0);
    double yy0 = get_center_coor(y0);
    double xx1 = get_center_coor(x1);
    double yy1 = get_center_coor(y1);

    // 方向向量
    double dx = xx1 - xx0;        
    double dy = yy1 - yy0;

    // 步进方向
    int stepX = sign(dx);
    int stepY = sign(dy);

    // 1. 得到参数方程 x = xx0 + (xx1-xx0)*t
    //                 y = yy0 + (yy1-yy0)*t
    
    // 从xx0 -> xx1, 得到从 start到目标点穿越的第一条 横向网格线(y = starty， 最后一条横向网格线(y=endy), 求这些网格线和（起点）-（终点）之间的交点
    // 交点上下的网格，都是要判断是不是障碍点的（一旦发现障碍点，直接return true退出）
    double startX = 0;
    double endX = 0;
    if (stepX != 0) //stepX==0方向没有移动，垂直移动，只需要考虑y方向
    {
        if (stepX > 0)
        {
            startX = x0+1;
            endX = x1;
        }
        else
        {
            startX = x0;
            endX = x1 + 1;
        }

        //能到这儿来，(x0,y0)肯定和(x1,y1)不在一个网格
        double x = startX;
        double k = dy / dx; //斜率拉！
        double y = yy0 + k * (x - xx0); //点斜式，我懒得直接敌退了，直接算吧
        if (is_H_blocked(x,y))
            return true;

        while (x != endX)
        {           
            // 在(x,y)两侧的网格都要检测
            x = x + stepX; // 移动一格, stepX可能式+1，也可能-1
            y = y + k* stepX;

            if (is_H_blocked(x,y))
                return true;

        }
    }



    // 3. 从xx0 -> xx1, 得到从 start到目标点穿越的第一条 横向网格线(x = startx， 最后一条横向网格线(x=endx), 求这些网格线和（起点）-（终点）之间的交点
    // 交点左右的网格，都是要判断是不是障碍点的（一旦发现障碍点，直接return true退出）
    double startY = 0;
    double endY = 0;
    if (stepY != 0) //stepY==0方向没有移动，水平移动，只需要考虑x方向
    {
        if (stepY > 0)
        {
            startY = y0 + 1;
            endY = y1;
        }
        else
        {
            startY = y0;
            endY = y1 + 1;
        }

        //能到这儿来，(x0,y0)肯定和(x1,y1)不在一个网格
        double y = startY;
        double k = dx / dy; //斜率拉！
        double x = xx0 + k * (y - yy0); //点斜式，我懒得直接敌退了，直接算吧
        if (is_V_blocked(x,y))
            return true;

        while (y != endY)
        {
            // 在(x,y)两侧的网格都要检测
            y = y + stepY; // 移动一格, stepX可能式+1，也可能-1
            x = x + k* stepY;

            if (is_V_blocked(x,y))
                return true;

        }
    }

    return false;
}

/*
  采用4领域搜索的 Bresenham检测算法
*/

// 4-邻域 Bresenham 线段扫描（仅上下左右）
bool lineOfSight4(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int cx = x0, cy = y0;
    int err = dx - dy;

    while (true) {
        if (!grid[cy][cx].pass) 
            return false; // 当前格是障碍

        if (cx == x1 && cy == y1) 
            break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; cx += sx; } // 横向移动
        if (e2 < dx) { err += dx; cy += sy; }  // 纵向移动
        // 注意：不允许对角线，因此不合并移动
    }
    return true;
}
/* ================= A* ================= */
std::vector<Node*> findPath_Astar(Node* start, Node* goal) 
{
    auto cmp = [](Node* a, Node* b) {
            return a->f > b->f; 
    };

    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

    for (int y = 0; y < ROWS; ++y)
    {
        for (int x = 0; x < COLS; ++x)
        {
            grid[y][x].g = 1e9;
            grid[y][x].f = 1e9;
            grid[y][x].parent = nullptr;
        }
    }
    start->g = 0;
    start->f = dist(start->x, start->y, goal->x, goal->y);
    open.push(start);

    while (!open.empty()) 
    {
        Node* curr = open.top(); open.pop();
        if (curr == goal) {
            std::vector<Node*> path;
            for (Node* p = goal; p; p = p->parent) path.push_back(p);
            return path;
        }
        for (int i = 0; i < 8; ++i) 
        {
            int nx = curr->x + dx8[i], ny = curr->y + dy8[i];

            // 2.1. 越界
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) 
                continue;

            // 2.2 本身不通行
            Node* neib = &grid[ny][nx];
            if (!neib->pass) continue;

            // ✅ 禁止对角线穿过两个障碍顶点
            //if (i >= 4) {
            //    if (isDiagonalBlocked(curr->x, curr->y, dx8[i], dy8[i]))
            //        continue;
            // }

            // 2.3 不许擦着角通行
            auto is_gothrough_conner = [](Node* curr, int search_direction)->bool
            {
                    // 正向搜索，不需要考虑
                    if (search_direction <= 3)
                        return false;

                    // 斜向搜索，才考虑
                    struct search_res
                    {
                        int res_id[2];
                    };
                    static search_res res[8]{
                        // 实际上，前四个元素我们是不管的，也不用，方前四个只是为了容易处理

                        {-1,-1},  //正方向的移动，只需要考虑移动到的网格是否可以通行即可！
                        {-1,-1},
                        {-1,-1},
                        {-1,-1},
                        {0,2},  // 斜角方向的移动
                        {0,3},
                        {1,2},
                        {1,3}
                    };

                    bool if_blocked = false;
                    int* drs = res[search_direction].res_id;
                    for (int i = 0; i < 2; ++i)
                    {
                        int pos = drs[i];
                        int nx = curr->x + dx8[pos];
                        int ny = curr->y + dy8[pos];

                        if (grid[ny][nx].pass == false)
                        {
                            if_blocked = true;
                            break;
                        }
                    }

                    // 斜向搜索，要考虑拉！
                    return if_blocked;
            };

            if (is_gothrough_conner(curr,i))
                  continue;

            // 2.4 其他正常情况
            double cost = (i < 4 ? 1.0 : 1.41421356);
            double tg = curr->g + cost;
            if (tg < neib->g) {
                neib->parent = curr;
                neib->g = tg;
                neib->f = tg + dist(neib->x, neib->y, goal->x, goal->y);
                open.push(neib);
            }
        }
    }
    return {};
}

/* ================= Theta* ================= */
std::vector<Node*> findPath_Theta(Node* start, Node* goal) 
{
    auto cmp = [](Node* a, Node* b) {
            return a->f > b->f; 
    };

    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    for (int y = 0; y < ROWS; ++y)
    {
        for (int x = 0; x < COLS; ++x)
        {
            grid[y][x].g = 1e9;
            grid[y][x].f = 1e9;
            grid[y][x].parent = nullptr;
        }
    }
    start->g = 0;
    start->f = dist(start->x, start->y, goal->x, goal->y);
    open.push(start);

    while (!open.empty()) 
    {
        Node* curr = open.top(); 
        open.pop();
        if (curr == goal) 
        {
            std::vector<Node*> path;
            for (Node* p = goal; p; p = p->parent) 
                path.push_back(p);
            return path;
        }
        for (int i = 0; i < 8; ++i) 
        {
            int nx = curr->x + dx8[i], ny = curr->y + dy8[i];
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) 
                continue;

            Node* neib = &grid[ny][nx];
            if (!neib->pass) continue;

            // 2.3 不许擦着角通行
            auto is_gothrough_conner = [](Node* curr, int search_direction)->bool
                {
                    // 正向搜索，不需要考虑
                    if (search_direction <= 3)
                        return false;

                    // 斜向搜索，才考虑
                    struct search_res
                    {
                        int res_id[2];
                    };
                    static search_res res[8]{
                        // 实际上，前四个元素我们是不管的，也不用，方前四个只是为了容易处理

                        {-1,-1},  //正方向的移动，只需要考虑移动到的网格是否可以通行即可！
                        {-1,-1},
                        {-1,-1},
                        {-1,-1},
                        {0,2},  // 斜角方向的移动
                        {0,3},
                        {1,2},
                        {1,3}
                    };

                    bool if_blocked = false;
                    int* drs = res[search_direction].res_id;
                    for (int i = 0; i < 2; ++i)
                    {
                        int pos = drs[i];
                        int nx = curr->x + dx8[pos];
                        int ny = curr->y + dy8[pos];

                        if (grid[ny][nx].pass == false)
                        {
                            if_blocked = true;
                            break;
                        }
                    }

                    // 斜向搜索，要考虑拉！
                    return if_blocked;
                };

            if (is_gothrough_conner(curr, i))
                continue;

            //===================================================================
            Node* par = curr->parent ? curr->parent : curr;
#if 1            
            bool los = (lineIntersectsObstacle(par->x, par->y, neib->x, neib->y)==false);  //检测是否可以精简
#else
            bool los = (lineOfSight4(par->x, par->y, neib->x, neib->y) == false);  //这个函数有问题哦！
#endif
            double tg = los ? par->g + dist(par->x, par->y, neib->x, neib->y)
                : curr->g + dist(curr->x, curr->y, neib->x, neib->y);
            if (tg < neib->g) 
            {
                neib->parent = los ? par : curr;
                neib->g = tg;
                neib->f = tg + dist(neib->x, neib->y, goal->x, goal->y);
                open.push(neib);
            }
        }
    }
    return {};
}
BOOL CenterWindow(HWND hwnd) {
    // 验证窗口句柄有效性
    if (hwnd == NULL || !IsWindow(hwnd)) {
        return FALSE;
    }

    // 获取窗口自身尺寸（包含边框和标题栏）
    RECT windowRect;
    if (!GetWindowRect(hwnd, &windowRect)) {
        return FALSE;
    }
    int windowWidth = windowRect.right - windowRect.left;
    int windowHeight = windowRect.bottom - windowRect.top;

    // 获取屏幕工作区尺寸（排除任务栏等系统区域）
    HWND hDesktop = GetDesktopWindow();
    RECT screenRect;
    if (!GetClientRect(hDesktop, &screenRect)) {
        // 备选方案：获取整个屏幕尺寸（包含任务栏）
        screenRect.right = GetSystemMetrics(SM_CXSCREEN);
        screenRect.bottom = GetSystemMetrics(SM_CYSCREEN);
    }
    int screenWidth = screenRect.right - screenRect.left;
    int screenHeight = screenRect.bottom - screenRect.top;

    // 计算居中位置
    int x = (screenWidth - windowWidth) / 2;
    int y = (screenHeight - windowHeight) / 2;

    // 确保窗口不会超出屏幕边界（特别是在多显示器环境下）
    x = (x < 0) ? 0 : x;
    y = (y < 0) ? 0 : y;

    // 移动窗口到居中位置（保持原尺寸）
    return SetWindowPos(
        hwnd,
        NULL,       // 不改变Z序
        x, y,       // 新位置
        0, 0,       // 保持原尺寸
        SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE  // 不改变尺寸、Z序和激活状态
    );
}

/* ================= 主函数 ================= */
int main() 
{
    initgraph(WIDTH, HEIGHT);
    ShowWindow(GetConsoleWindow(), SW_HIDE);
    CenterWindow(GetHWnd());
    BringWindowToTop(GetHWnd());
    setbkcolor(RGB(255, 255, 255));

    initMap();
    SetWindowText(GetHWnd(), L"最优路径演示工具V1.0, 张小国老师开发, 东南大学仪科学院. xgzhang@seu.edu.cn ");
    Node* start = &grid[0][0];
    Node* goal = &grid[ROWS - 1][COLS - 1];
    bool needRedraw = true;
    //setbkcolor(RGB(255, 255, 255));

    while (true) 
    {
        //setbkcolor(RGB(255, 255, 255));
        //SetWindowText(GetForegroundWindow(), L"最优路径演示工具V1.0, 张小国老师开发, 东南大学仪科学院. xgzhang@seu.edu.cn ");
        if (needRedraw) 
        {
            drawMap();
            if (mode == ASTAR) 
            {
                // 这是很愚蠢的做法，每次都重新计算的，但是我懒得改了。。。。
                auto path = findPath_Astar(start, goal);
                drawPath(path, RED);
            }
            else if (mode == THETA) 
            {
                auto path = findPath_Theta(start, goal);
                drawPath(path, BLUE);
            }
            needRedraw = false;
        }
        //==============================================================================
        if (MouseHit()) {
            MOUSEMSG m = GetMouseMsg();
            int mx = m.x, my = m.y;              // 鼠标所在位置
            int gx = mx / CELL, gy = my / CELL;  // 鼠标所在网格
            COLORREF buttonNotOnState = BLUE;

            if (m.uMsg == WM_LBUTTONDOWN) 
            {
                /* 按钮点击检测 */
                if (btnBarrier.click(mx, my)) 
                {
                    if (mode != BARRIER) 
                    { 
                        mode = BARRIER; 
                        btnBarrier.color = buttonNotOnState;
                    }
                    else 
                    { 
                        mode = NONE; 
                        btnBarrier.color = LIGHTGRAY;
                    }
                    needRedraw = true; continue;
                }
                else if (btnAbout.click(mx, my))
                {
                    MessageBox(GetForegroundWindow(),
                        L"最优路径演示程序V1.0\r\n"
                        L" By Xiaoguo Zhang\r\n"
                        L" Southeast Univ.\r\n"
                        L" xgzhang@seu.edu.cn",
                        L"最优路径演示程序",
                        MB_OK
                    );
                    needRedraw = true; 
                    continue;
                }
                else if (btnStart.click(mx, my)) 
                {
                    if (mode != SET_START) 
                    { 
                        mode = SET_START; 
                        btnStart.color = buttonNotOnState;
                    }
                    else 
                    { 
                        mode = NONE; 
                        btnStart.color = BROWN;
                    }
                    needRedraw = true; 
                    continue;
                }
                else if (btnGoal.click(mx, my)) 
                {
                    if (mode != SET_GOAL) 
                    {
                        mode = SET_GOAL; 
                        btnGoal.color = buttonNotOnState;
                    }
                    else 
                    {
                        mode = NONE; 
                        btnGoal.color = LIGHTGRAY;
                    }
                    needRedraw = true;
                    continue;
                }
                else if (btnAstar.click(mx, my)) 
                {
                    mode = ASTAR; 
                    btnAstar.color = buttonNotOnState;
                    btnTheta.color = DARKGRAY;
                    needRedraw = true; 
                    continue;
                }
                else if (btnTheta.click(mx, my)) 
                {
                    mode = THETA; 
                    btnTheta.color = buttonNotOnState;
                    btnAstar.color = DARKGRAY;
                    needRedraw = true; 
                    continue;
                }                
                 else if (btnClear.click(mx, my))
                 {
                     int result = MessageBox(GetForegroundWindow(),
                         L"清除地图？请选择确认或取消",
                         L"提示",
                         MB_OKCANCEL | MB_ICONINFORMATION
                     );

                     // 判断用户选择
                     if (result == IDOK) {
                         // 用户点击了确认按钮
                         for (int i = 0; i < ::ROWS; ++i)
                         {
                             for (int j = 0; j < COLS; ++j)
                             {
                                 grid[i][j].pass=true;
                             }
                         }
                     }
                     drawMap();
                     continue;
                 }
                else if (btnLoad.click(mx, my))
                {
                    auto openFileDialog = []()->wstring {
                        OPENFILENAMEW ofn;
                        WCHAR szFile[MAX_PATH] = { 0 };

                        // 初始化OPENFILENAME结构
                        ZeroMemory(&ofn, sizeof(ofn));
                        ofn.lStructSize = sizeof(ofn);
                        ofn.hwndOwner = GetHWnd();  // 获取EasyX窗口句柄
                        ofn.lpstrFile = szFile;
                        ofn.nMaxFile = sizeof(szFile) / sizeof(WCHAR);

                        // 设置文件过滤，例如只显示图片文件
                        ofn.lpstrFilter = L"地图文件 (*.dat)\0*.dat\0所有文件 (*.*)\0*.*\0\0";
                        ofn.nFilterIndex = 1;
                        ofn.lpstrFileTitle = NULL;
                        ofn.nMaxFileTitle = 0;
                        ofn.lpstrInitialDir = NULL;
                        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

                        // 显示打开文件对话框
                        if (GetOpenFileNameW(&ofn) == TRUE) {
                            return std::wstring(ofn.lpstrFile);
                        }
                        return L"";  // 用户取消选择
                        };

                    wstring s = openFileDialog();
                    ifstream is(s, ios::binary);
                    if (is.is_open() == false)
                        continue;
                    for (int i = 0; i < ::ROWS; ++i)
                    {
                        for (int j = 0; j < COLS; ++j)
                        {
                            is.read((char*)&grid[i][j].pass, sizeof(grid[i][j].pass));
                        }
                    }
                    wstring s1 = _T("successfully load:");
                    s1 += s;
                    drawMap();
                    MessageBox(GetForegroundWindow(),s1.c_str(), NULL, MB_OK);
                    continue;
                }
                else if (btnSave.click(mx, my))
                {
                    auto generate_filename = []()
                        {
                            // 获取当前时间
                            std::time_t now = std::time(nullptr);
                            std::tm localTime = *std::localtime(&now);

                            // 格式化时间为字符串
                            std::stringstream ss;
                            ss << std::setfill('0')
                                << std::setw(4) << (localTime.tm_year + 1900) << "_"  // 年
                                << std::setw(2) << (localTime.tm_mon + 1) << "_"      // 月
                                << std::setw(2) << localTime.tm_mday << "_"           // 日
                                << std::setw(2) << localTime.tm_hour << "_"           // 时
                                << std::setw(2) << localTime.tm_min << "_"            // 分
                                << std::setw(2) << localTime.tm_sec << "_"            // 秒
                                << "map.dat";                                         // 后缀

                            return ss.str();
                        };
                    string s = generate_filename();
                    ofstream os(s, ios::binary);
                    for (int i = 0; i < ::ROWS; ++i)
                    {
                        for (int j=0;j<COLS;++j)
                        {
                            os.write((const char*) & grid[i][j].pass, sizeof(grid[i][j].pass));
                        }
                    }
                    os.close();
                    string s2 = "Genreated file:" + s;

                    // 将string转换为wstring
                    auto stringToWstring = [](const std::string& str)->wstring 
                        {
                            int len = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, nullptr, 0);
                            std::wstring wstr(len, L'\0');
                            MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, &wstr[0], len);
                            return wstr;
                        };

                    wstring s1 = stringToWstring(s2);
                    MessageBox(GetForegroundWindow(), s1.c_str(), NULL, MB_OK);
                    continue;
                }

                /* 网格点击处理 */
                if (gx >= 0 && gx < COLS && gy >= 0 && gy < ROWS) 
                {
                    if (mode == BARRIER) {
                        grid[gy][gx].pass = !grid[gy][gx].pass;
                        needRedraw = true;
                    }
                    else if (mode == SET_START && grid[gy][gx].pass) {
                        start = &grid[gy][gx];
                        cout << "起点：行:" << gy << ", 列：" << gx <<endl;
                        needRedraw = true;
                    }
                    else if (mode == SET_GOAL && grid[gy][gx].pass) {
                        goal = &grid[gy][gx];
                        cout << "终点：行:" << gy << ", 列：" << gx << endl;
                        needRedraw = true;
                    }
                }
            }
        }

        if (_kbhit()) 
        {
            char c = _getch();
            if (c == ' ') {   // 清空障碍
                for (int y = 0; y < ROWS; ++y)
                    for (int x = 0; x < COLS; ++x)
                        grid[y][x].pass = true;
                needRedraw = true;
            }
            if (c == 27) 
                break; // ESC 退出
        }
        Sleep(2);
    }


    closegraph();
    return 0;
}