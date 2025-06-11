#include <stdio.h>
#include <iostream>
#include <chrono>
#include <queue>
#include <cmath>

using namespace std;

constexpr int MAX = 64;
constexpr double INF = 1e9 + 7;

// 상하좌우
const int dx1[4] = { 0, 0, 1, -1 };
const int dy1[4] = { -1, 1, 0, 0 };

struct Cell {
    int parent_x, parent_y;
    double f, g, h;
};

struct Pair {
    int first;
    int second;
};

struct pPair {
    double first;
    Pair second;
    
    // 비교 연산자 추가
    bool operator>(const pPair& other) const {
        return first > other.first;
    }
    
    bool operator<(const pPair& other) const {
        return first < other.first;
    }
};

char zmap[MAX][MAX];
int ROW = 0, COL = 0;

Pair path[MAX];
int pIdx = 0;

bool isInRange(int row, int col) {
    return (row >= 0 && row < ROW && col >= 0 && col < COL);
}

bool isUnBlocked(int map[8][8], int row, int col) {
    return (map[row][col] == 0);
}

double GethValue(int row, int col, Pair dst) {
    return (double)(abs(row - dst.first) + abs(col - dst.second));
}

void tracePath(Cell cellDetails[MAX][MAX], Pair dst, bool isForward) {
    Pair s[MAX];
    int sIdx = 0;
    int y = dst.first;
    int x = dst.second;

    s[sIdx] = { y, x };
    sIdx++;

    while (!(cellDetails[y][x].parent_x == x && cellDetails[y][x].parent_y == y)) {
        int tempy = cellDetails[y][x].parent_y;
        int tempx = cellDetails[y][x].parent_x;
        y = tempy;
        x = tempx;
        s[sIdx] = { y, x };
        sIdx++;
    }

    if (isForward) {
        while (sIdx != 0) {
            zmap[s[sIdx-1].first][s[sIdx-1].second] = '*';
            path[pIdx] = s[sIdx-1];
            pIdx++; sIdx--;
        }
    } else {
        for (int i = 0; i < sIdx; i++) {
            zmap[s[i].first][s[i].second] = '*';
            path[pIdx] = s[i];
            pIdx++;
        }
    }
}

bool bidirectionalAStarSearch(int map[8][8], Pair src, Pair dst) {
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
    if (src.first == dst.first && src.second == dst.second) return false;

    // Forward and backward search data structures
    bool closedListForward[MAX][MAX] = {false};
    bool closedListBackward[MAX][MAX] = {false};
    Cell cellDetailsForward[MAX][MAX];
    Cell cellDetailsBackward[MAX][MAX];

    // Initialize both forward and backward search
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            cellDetailsForward[i][j].f = cellDetailsForward[i][j].g = cellDetailsForward[i][j].h = INF;
            cellDetailsForward[i][j].parent_x = cellDetailsForward[i][j].parent_y = -1;
            
            cellDetailsBackward[i][j].f = cellDetailsBackward[i][j].g = cellDetailsBackward[i][j].h = INF;
            cellDetailsBackward[i][j].parent_x = cellDetailsBackward[i][j].parent_y = -1;
        }
    }

    // Initialize forward search
    int sy = src.first;
    int sx = src.second;
    cellDetailsForward[sy][sx].f = cellDetailsForward[sy][sx].g = cellDetailsForward[sy][sx].h = 0.0;
    cellDetailsForward[sy][sx].parent_x = sx;
    cellDetailsForward[sy][sx].parent_y = sy;

    // Initialize backward search
    int dy = dst.first;
    int dx = dst.second;
    cellDetailsBackward[dy][dx].f = cellDetailsBackward[dy][dx].g = cellDetailsBackward[dy][dx].h = 0.0;
    cellDetailsBackward[dy][dx].parent_x = dx;
    cellDetailsBackward[dy][dx].parent_y = dy;

    // Priority queues for both directions
    priority_queue<pPair, vector<pPair>, greater<pPair>> openListForward;
    priority_queue<pPair, vector<pPair>, greater<pPair>> openListBackward;

    openListForward.push({0.0, {sy, sx}});
    openListBackward.push({0.0, {dy, dx}});

    while (!openListForward.empty() && !openListBackward.empty()) {
        // Forward search
        pPair pForward = openListForward.top();
        openListForward.pop();
        int yForward = pForward.second.first;
        int xForward = pForward.second.second;
        closedListForward[yForward][xForward] = true;

        // Check if current node is in backward closed list
        if (closedListBackward[yForward][xForward]) {
            // Path found
            tracePath(cellDetailsForward, {yForward, xForward}, true);
            tracePath(cellDetailsBackward, {yForward, xForward}, false);
            return true;
        }

        // Expand forward search
        for (int i = 0; i < 4; ++i) {
            int ny = yForward + dy1[i];
            int nx = xForward + dx1[i];

            if (isInRange(ny, nx) && !closedListForward[ny][nx] && isUnBlocked(map, ny, nx)) {
                double ng = cellDetailsForward[yForward][xForward].g + 1.0;
                double nh = GethValue(ny, nx, dst);
                double nf = ng + nh;

                if (cellDetailsForward[ny][nx].f == INF || cellDetailsForward[ny][nx].f > nf) {
                    cellDetailsForward[ny][nx].f = nf;
                    cellDetailsForward[ny][nx].g = ng;
                    cellDetailsForward[ny][nx].h = nh;
                    cellDetailsForward[ny][nx].parent_x = xForward;
                    cellDetailsForward[ny][nx].parent_y = yForward;
                    openListForward.push({nf, {ny, nx}});
                }
            }
        }

        // Backward search
        pPair pBackward = openListBackward.top();
        openListBackward.pop();
        int yBackward = pBackward.second.first;
        int xBackward = pBackward.second.second;
        closedListBackward[yBackward][xBackward] = true;

        // Check if current node is in forward closed list
        if (closedListForward[yBackward][xBackward]) {
            // Path found
            tracePath(cellDetailsForward, {yBackward, xBackward}, true);
            tracePath(cellDetailsBackward, {yBackward, xBackward}, false);
            return true;
        }

        // Expand backward search
        for (int i = 0; i < 4; ++i) {
            int ny = yBackward + dy1[i];
            int nx = xBackward + dx1[i];

            if (isInRange(ny, nx) && !closedListBackward[ny][nx] && isUnBlocked(map, ny, nx)) {
                double ng = cellDetailsBackward[yBackward][xBackward].g + 1.0;
                double nh = GethValue(ny, nx, src);
                double nf = ng + nh;

                if (cellDetailsBackward[ny][nx].f == INF || cellDetailsBackward[ny][nx].f > nf) {
                    cellDetailsBackward[ny][nx].f = nf;
                    cellDetailsBackward[ny][nx].g = ng;
                    cellDetailsBackward[ny][nx].h = nh;
                    cellDetailsBackward[ny][nx].parent_x = xBackward;
                    cellDetailsBackward[ny][nx].parent_y = yBackward;
                    openListBackward.push({nf, {ny, nx}});
                }
            }
        }
    }

    return false;
}

void PrintMap() {
    printf("\nPath Map:\n");
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            printf("%c", zmap[i][j]);
        }
        printf("\n");
    }
}

void PrintPath() {
    printf("\nPath Coordinates:\n");
    for (int i = 0; i < pIdx; i++) {
        printf("(%d, %d)\n", path[i].first, path[i].second);
    }
}

int main() {
    Pair src = {1, 0}, dst = {6, 7};
    int row = 8, col = 8;
    
    ROW = row;
    COL = col;
    
    int grid[8][8];
    
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            grid[i][j] = 0;
        }
    }
    
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            zmap[i][j] = grid[i][j] + '0';
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    bool isProcessed = bidirectionalAStarSearch(grid, src, dst);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;

    if (isProcessed) {
        PrintMap();
        PrintPath();
    } else {
        printf("Failed to find path.\n");
    }

    printf("\nBidirectional A* Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
    
    return 0;
} 