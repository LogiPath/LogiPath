#include <stdio.h>
#include <chrono>

constexpr int MAX = 64;
constexpr int INF = 32767;

struct Pair {
    int first;
    int second;
};

struct Edge {
    int from_y, from_x;
    int to_y, to_x;
    int weight;
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

void tracePath(Pair parent[MAX][MAX], Pair dst) {
    int y = dst.first;
    int x = dst.second;
    
    while (parent[y][x].first != -1 && parent[y][x].second != -1) {
        path[pIdx] = {y, x};
        pIdx++;
        zmap[y][x] = '*';
        
        int tempy = parent[y][x].first;
        int tempx = parent[y][x].second;
        y = tempy;
        x = tempx;
    }
    
    path[pIdx] = {y, x};
    pIdx++;
    zmap[y][x] = '*';
}

bool bellmanFordSearch(int map[8][8], Pair src, Pair dst) {
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
    
    int dist[MAX][MAX];
    Pair parent[MAX][MAX];
    Edge edges[MAX * MAX * 4];
    int edgeCount = 0;
    
    const int dx[4] = {0, 0, 1, -1};
    const int dy[4] = {-1, 1, 0, 0};
    
    // Initialize distances and parent
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            dist[i][j] = INF;
            parent[i][j] = {-1, -1};
        }
    }
    
    // Create edges
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (!isUnBlocked(map, i, j)) continue;
            
            for (int k = 0; k < 4; k++) {
                int ny = i + dy[k];
                int nx = j + dx[k];
                
                if (isInRange(ny, nx) && isUnBlocked(map, ny, nx)) {
                    edges[edgeCount++] = {i, j, ny, nx, 1};
                }
            }
        }
    }
    
    dist[src.first][src.second] = 0;
    
    // Bellman-Ford algorithm
    for (int i = 0; i < ROW * COL - 1; i++) {
        for (int j = 0; j < edgeCount; j++) {
            Edge edge = edges[j];
            if (dist[edge.from_y][edge.from_x] != INF && 
                dist[edge.from_y][edge.from_x] + edge.weight < dist[edge.to_y][edge.to_x]) {
                dist[edge.to_y][edge.to_x] = dist[edge.from_y][edge.from_x] + edge.weight;
                parent[edge.to_y][edge.to_x] = {edge.from_y, edge.from_x};
            }
        }
    }
    
    if (dist[dst.first][dst.second] == INF) return false;
    
    tracePath(parent, dst);
    return true;
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

    bool isProcessed = bellmanFordSearch(grid, src, dst);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;

    if (isProcessed) {
        PrintMap();
        PrintPath();
    } else {
        printf("Failed to find path.\n");
    }

    printf("\nBellman-Ford Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
    
    return 0;
} 