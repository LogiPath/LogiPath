#include <stdio.h>
#include <chrono>
#include <queue>
#include <vector>

using namespace std;

constexpr int MAX = 64;
constexpr int INF = 32767;

struct Pair {
    int first;
    int second;
};

struct Node {
    int dist;
    Pair pos;

    // 비교 연산자 추가
    bool operator>(const Node& other) const {
        return dist > other.dist;
    }

    bool operator<(const Node& other) const {
        return dist < other.dist;
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

void tracePath(Pair parent[MAX][MAX], Pair start, Pair end, bool reverse) {
    // 경로를 저장할 임시 벡터
    std::vector<Pair> temp;
    Pair current = end;
    int maxSteps = ROW * COL;  // 최대 가능한 스텝 수
    int steps = 0;
    
    // 시작점에 도달할 때까지 부모 노드를 따라가며 경로 구성
    while ((current.first != start.first || current.second != start.second) && steps < maxSteps) {
        // 유효하지 않은 부모 체인 체크
        if (!isInRange(current.first, current.second) || 
            parent[current.first][current.second].first == -1) {
            printf("Invalid parent chain detected at (%d, %d)\n", 
                   current.first, current.second);
            return;
        }
        
        temp.push_back(current);
        current = parent[current.first][current.second];
        steps++;
    }
    
    // 무한 루프 체크
    if (steps >= maxSteps) {
        printf("Error: Maximum path length exceeded. Possible cycle detected.\n");
        return;
    }
    
    // 시작점 추가
    temp.push_back(start);
    
    // 경로를 최종 경로 배열에 저장
    if (reverse) {
        // 역방향 경로 저장 (meeting point -> destination)
        // 역방향일 때는 시작점을 제외하고 저장 (중복 방지)
        for (int i = 0; i < temp.size(); i++) {
            path[pIdx++] = temp[i];
            zmap[temp[i].first][temp[i].second] = '*';
        }
    } else {
        // 정방향 경로 저장 (source -> meeting point)
        for (auto it = temp.rbegin(); it != temp.rend(); ++it) {
            path[pIdx++] = *it;
            zmap[it->first][it->second] = '*';
        }
    }
}

bool bidirectionalDijkstraSearch(int map[8][8], Pair src, Pair dst) {
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) {
        printf("Source or destination out of range\n");
        return false;
    }
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) {
        printf("Source or destination is blocked\n");
        return false;
    }
    if (src.first == dst.first && src.second == dst.second) {
        printf("Source and destination are the same\n");
        return false;
    }

    // Forward and backward search data structures
    int distForward[MAX][MAX];
    int distBackward[MAX][MAX];
    Pair parentForward[MAX][MAX];
    Pair parentBackward[MAX][MAX];
    bool visitedForward[MAX][MAX] = { false };
    bool visitedBackward[MAX][MAX] = { false };

    // Initialize both forward and backward search
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            distForward[i][j] = INF;
            distBackward[i][j] = INF;
            parentForward[i][j] = { -1, -1 };
            parentBackward[i][j] = { -1, -1 };
        }
    }

    // Initialize forward search
    distForward[src.first][src.second] = 0;
    parentForward[src.first][src.second] = { src.first, src.second };

    // Initialize backward search
    distBackward[dst.first][dst.second] = 0;
    parentBackward[dst.first][dst.second] = { dst.first, dst.second };

    // Priority queues for both directions
    priority_queue<Node, vector<Node>, greater<Node>> pqForward;
    priority_queue<Node, vector<Node>, greater<Node>> pqBackward;

    pqForward.push({ 0, src });
    pqBackward.push({ 0, dst });

    const int dx[4] = { 0, 0, 1, -1 };
    const int dy[4] = { -1, 1, 0, 0 };

    int bestDist = INF;
    Pair meetingPoint = { -1, -1 };

    while (!pqForward.empty() && !pqBackward.empty()) {
        // Forward search
        Node currentForward = pqForward.top();
        pqForward.pop();
        int yForward = currentForward.pos.first;
        int xForward = currentForward.pos.second;

        if (visitedBackward[yForward][xForward]) {
            int totalDist = distForward[yForward][xForward] + distBackward[yForward][xForward];
            if (totalDist < bestDist) {
                bestDist = totalDist;
                meetingPoint = { yForward, xForward };
            }
        }

        if (visitedForward[yForward][xForward]) continue;
        visitedForward[yForward][xForward] = true;

        for (int i = 0; i < 4; i++) {
            int ny = yForward + dy[i];
            int nx = xForward + dx[i];

            if (isInRange(ny, nx) && isUnBlocked(map, ny, nx) && !visitedForward[ny][nx]) {
                int newDist = distForward[yForward][xForward] + 1;

                if (newDist < distForward[ny][nx]) {
                    distForward[ny][nx] = newDist;
                    parentForward[ny][nx] = { yForward, xForward };
                    pqForward.push({ newDist, {ny, nx} });
                }
            }
        }

        // Backward search
        Node currentBackward = pqBackward.top();
        pqBackward.pop();
        int yBackward = currentBackward.pos.first;
        int xBackward = currentBackward.pos.second;

        if (visitedForward[yBackward][xBackward]) {
            int totalDist = distForward[yBackward][xBackward] + distBackward[yBackward][xBackward];
            if (totalDist < bestDist) {
                bestDist = totalDist;
                meetingPoint = { yBackward, xBackward };
            }
        }

        if (visitedBackward[yBackward][xBackward]) continue;
        visitedBackward[yBackward][xBackward] = true;

        for (int i = 0; i < 4; i++) {
            int ny = yBackward + dy[i];
            int nx = xBackward + dx[i];

            if (isInRange(ny, nx) && isUnBlocked(map, ny, nx) && !visitedBackward[ny][nx]) {
                int newDist = distBackward[yBackward][xBackward] + 1;

                if (newDist < distBackward[ny][nx]) {
                    distBackward[ny][nx] = newDist;
                    parentBackward[ny][nx] = { yBackward, xBackward };
                    pqBackward.push({ newDist, {ny, nx} });
                }
            }
        }
    }

    if (meetingPoint.first != -1 && meetingPoint.second != -1) {
        pIdx = 0;
        // forward: src -> meetingPoint
        tracePath(parentForward, src, meetingPoint, false);
        // backward: dst -> meetingPoint (역방향이므로 dst를 시작점으로)
        tracePath(parentBackward, dst, meetingPoint, true);
        return true;
    }

    printf("No path found\n");
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
    Pair src = { 1, 0 }, dst = { 6, 7 };
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

    bool isProcessed = bidirectionalDijkstraSearch(grid, src, dst);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;

    if (isProcessed) {
        PrintMap();
        PrintPath();
    }
    else {
        printf("Failed to find path.\n");
    }

    printf("\nBidirectional Dijkstra Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());

    return 0;
}