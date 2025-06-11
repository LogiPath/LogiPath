#include <stdio.h>
#include <chrono>

constexpr int MAX = 64;
constexpr int INF = 32767;

struct Pair {
    int first;
    int second;
};

struct Node {
    int dist;
    Pair pos;
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

void swap(Node* a, Node* b) {
    Node temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(Node arr[], int n, int i) {
    int smallest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (left < n && arr[left].dist < arr[smallest].dist)
        smallest = left;

    if (right < n && arr[right].dist < arr[smallest].dist)
        smallest = right;

    if (smallest != i) {
        swap(&arr[i], &arr[smallest]);
        heapify(arr, n, smallest);
    }
}

Node extractMin(Node arr[], int* n) {
    Node min = arr[0];
    arr[0] = arr[--(*n)];
    heapify(arr, *n, 0);
    return min;
}

void insertNode(Node arr[], int* n, Node key) {
    int i = (*n)++;
    arr[i] = key;
    
    while (i > 0 && arr[(i-1)/2].dist > arr[i].dist) {
        swap(&arr[i], &arr[(i-1)/2]);
        i = (i-1)/2;
    }
}

bool dijkstraSearch(int map[8][8], Pair src, Pair dst) {
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
    
    int dist[MAX][MAX];
    Pair parent[MAX][MAX];
    bool visited[MAX][MAX] = {false};
    
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            dist[i][j] = INF;
            parent[i][j] = {-1, -1};
        }
    }
    
    dist[src.first][src.second] = 0;
    
    Node pq[MAX * MAX];
    int pqSize = 0;
    insertNode(pq, &pqSize, {0, src});
    
    const int dx[4] = {0, 0, 1, -1};
    const int dy[4] = {-1, 1, 0, 0};
    
    while (pqSize > 0) {
        Node current = extractMin(pq, &pqSize);
        int y = current.pos.first;
        int x = current.pos.second;
        
        if (y == dst.first && x == dst.second) {
            tracePath(parent, dst);
            return true;
        }
        
        if (visited[y][x]) continue;
        visited[y][x] = true;
        
        for (int i = 0; i < 4; i++) {
            int ny = y + dy[i];
            int nx = x + dx[i];
            
            if (isInRange(ny, nx) && isUnBlocked(map, ny, nx) && !visited[ny][nx]) {
                int newDist = dist[y][x] + 1;
                
                if (newDist < dist[ny][nx]) {
                    dist[ny][nx] = newDist;
                    parent[ny][nx] = {y, x};
                    insertNode(pq, &pqSize, {newDist, {ny, nx}});
                }
            }
        }
    }
    
    return false;
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
    
    if (dijkstraSearch(grid, src, dst)) PrintMap();
    else printf("Failed to find path.");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;
    
    printf("\nDijkstra Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
    
    int i = 0;
    while (i < pIdx) {
        Pair p = path[i];
        printf("( %d, %d )\n", p.first, p.second);
        i++;
    }
    
    return 0;
} 