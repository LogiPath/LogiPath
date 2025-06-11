#include <stdio.h>
#include <chrono>

constexpr int MAX = 64;
constexpr int INF = 32767;

struct Pair {
    int first;
    int second;
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

void tracePath(Pair next[MAX][MAX], Pair src, Pair dst) {
    if (next[src.first * COL + src.second][dst.first * COL + dst.second].first == -1) return;
    
    Pair current = src;
    while (current.first != dst.first || current.second != dst.second) {
        path[pIdx] = current;
        pIdx++;
        zmap[current.first][current.second] = '*';
        
        int idx = current.first * COL + current.second;
        current = next[idx][dst.first * COL + dst.second];
    }
    
    path[pIdx] = dst;
    pIdx++;
    zmap[dst.first][dst.second] = '*';
}

bool floydWarshallSearch(int map[8][8], Pair src, Pair dst) {
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
    
    int n = ROW * COL;
    int dist[MAX][MAX];
    Pair next[MAX][MAX];
    
    const int dx[4] = {0, 0, 1, -1};
    const int dy[4] = {-1, 1, 0, 0};
    
    // Initialize distances and next pointers
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            dist[i][j] = INF;
            next[i][j] = {-1, -1};
        }
    }
    
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (!isUnBlocked(map, i, j)) continue;
            
            int idx = i * COL + j;
            dist[idx][idx] = 0;
            
            for (int k = 0; k < 4; k++) {
                int ny = i + dy[k];
                int nx = j + dx[k];
                
                if (isInRange(ny, nx) && isUnBlocked(map, ny, nx)) {
                    int nidx = ny * COL + nx;
                    dist[idx][nidx] = 1;
                    next[idx][nidx] = {ny, nx};
                }
            }
        }
    }
    
    // Floyd-Warshall algorithm
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (dist[i][k] != INF && dist[k][j] != INF && 
                    dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }
    
    int srcIdx = src.first * COL + src.second;
    int dstIdx = dst.first * COL + dst.second;
    
    if (dist[srcIdx][dstIdx] == INF) return false;
    
    tracePath(next, src, dst);
    return true;
}

void PrintMap() {
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            printf("%c", zmap[i][j]);
        }
        printf("\n");
    }
}

int main() {
    Pair src = {1, 1}, dst = {5, 5};
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
    
    if (floydWarshallSearch(grid, src, dst)) PrintMap();
    else printf("Failed to find path.");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;
    
    printf("\nFloyd-Warshall Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
    
    int i = 0;
    while (i < pIdx) {
        Pair p = path[i];
        printf("( %d, %d )\n", p.first, p.second);
        i++;
    }
    
    return 0;
} 