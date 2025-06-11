#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>

constexpr int MAX = 64;
constexpr int INF = std::numeric_limits<int>::max();

// 기본 구조체 정의
struct Pair {
    int first, second;
    bool operator==(const Pair& other) const {
        return first == other.first && second == other.second;
    }
};

// 상하좌우 이동 방향
const int dx[4] = { 0, 0, 1, -1 };
const int dy[4] = { -1, 1, 0, 0 };

// 전역 변수
char zmap[MAX][MAX];
int ROW = 0, COL = 0;
Pair path[MAX];
int pIdx = 0;

// 유틸리티 함수들
bool isInRange(int r, int c) {
    return (r >= 0 && r < ROW && c >= 0 && c < COL);
}

bool isUnBlocked(int map[][8], int r, int c) {
    return map[r][c] == 0;
}

// 랜드마크 관리 클래스
class LandmarkManager {
private:
    std::vector<Pair> landmarks;
    int ldist[4][MAX][MAX];  // 각 랜드마크에서의 거리

public:
    LandmarkManager() {
        // 네 구석을 랜드마크로 설정
        landmarks = {
            {0, 0}, {0, COL - 1}, {ROW - 1, 0}, {ROW - 1, COL - 1}
        };
    }

    // BFS로 각 랜드마크에서의 거리 계산
    void computeDistances(int map[][8]) {
        for (int i = 0; i < landmarks.size(); ++i) {
            // 거리 배열 초기화
            for (int r = 0; r < ROW; ++r)
                for (int c = 0; c < COL; ++c)
                    ldist[i][r][c] = INF;

            // BFS로 거리 계산
            std::queue<Pair> q;
            int lx = landmarks[i].first;
            int ly = landmarks[i].second;
            ldist[i][lx][ly] = 0;
            q.push({lx, ly});

            while (!q.empty()) {
                Pair current = q.front(); q.pop();
                int x = current.first;
                int y = current.second;
                for (int dir = 0; dir < 4; ++dir) {
                    int nx = x + dx[dir], ny = y + dy[dir];
                    if (isInRange(nx, ny) && isUnBlocked(map, nx, ny)
                        && ldist[i][nx][ny] == INF) {
                        ldist[i][nx][ny] = ldist[i][x][y] + 1;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }

    // 휴리스틱 함수: max(맨해튼 거리, 랜드마크 기반 거리)
    int getHeuristic(int x, int y, const Pair& dst) {
        int h = 0;
        // 랜드마크 기반 거리
        for (int i = 0; i < landmarks.size(); ++i) {
            int diff = std::abs(ldist[i][dst.first][dst.second] - ldist[i][x][y]);
            h = std::max(h, diff);
        }
        // 맨해튼 거리
        int manhattan = std::abs(x - dst.first) + std::abs(y - dst.second);
        return std::max(h, manhattan);
    }
};

// A* 노드 구조체
struct Node {
    int x, y, g, f;
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// 경로 추적 함수
void tracePath(Pair cameFrom[MAX][MAX], const Pair& dst) {
    std::vector<Pair> nodeStack;
    Pair cur = dst;
    int maxSteps = ROW * COL;
    int steps = 0;

    while (!(cameFrom[cur.first][cur.second] == cur) && steps < maxSteps) {
        nodeStack.push_back(cur);
        cur = cameFrom[cur.first][cur.second];
        steps++;
    }

    if (steps >= maxSteps) {
        std::cout << "Error: Maximum path length exceeded. Possible cycle detected.\n";
        return;
    }

    nodeStack.push_back(cur);
    for (auto it = nodeStack.rbegin(); it != nodeStack.rend(); ++it) {
        zmap[it->first][it->second] = '*';
        path[pIdx++] = *it;
    }
}

// ALT 알고리즘 메인 함수
bool ALT(int map[][8], const Pair& src, const Pair& dst) {
    // 기본 검사
    if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
    if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
    if (src == dst) return false;

    // 랜드마크 관리자 초기화 및 거리 계산
    LandmarkManager lm;
    lm.computeDistances(map);

    // A* 검색 준비
    int gScore[MAX][MAX];
    bool closed[MAX][MAX];
    Pair cameFrom[MAX][MAX];

    for (int r = 0; r < ROW; ++r) {
        for (int c = 0; c < COL; ++c) {
            gScore[r][c] = INF;
            closed[r][c] = false;
            cameFrom[r][c] = {-1, -1};
        }
    }

    // 우선순위 큐 초기화
    std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
    gScore[src.first][src.second] = 0;
    cameFrom[src.first][src.second] = src;
    open.push({src.first, src.second, 0, lm.getHeuristic(src.first, src.second, dst)});

    // A* 메인 루프
    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        if (closed[cur.x][cur.y]) continue;
        closed[cur.x][cur.y] = true;

        // 도착 확인
        if (cur.x == dst.first && cur.y == dst.second) {
            tracePath(cameFrom, dst);
            return true;
        }

        // 이웃 탐색
        for (int dir = 0; dir < 4; ++dir) {
            int nx = cur.x + dx[dir], ny = cur.y + dy[dir];
            if (!isInRange(nx, ny) || !isUnBlocked(map, nx, ny) || closed[nx][ny])
                continue;

            int tentative_g = cur.g + 1;
            if (tentative_g < gScore[nx][ny]) {
                gScore[nx][ny] = tentative_g;
                cameFrom[nx][ny] = {cur.x, cur.y};
                int f = tentative_g + lm.getHeuristic(nx, ny, dst);
                open.push({nx, ny, tentative_g, f});
            }
        }
    }

    return false;
}

// 출력 함수들
void PrintMap() {
    std::cout << "\nPath Map:\n";
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            std::cout << zmap[i][j];
        }
        std::cout << "\n";
    }
}

void PrintPath() {
    std::cout << "\nPath Coordinates:\n";
    for (int i = 0; i < pIdx; ++i) {
        std::cout << "(" << path[i].first << ", " << path[i].second << ")\n";
    }
}

int main() {
    Pair src = {1, 0}, dst = {6, 7};
    ROW = COL = 8;
    int grid[8][8] = {0};

    // zmap 초기화
    for (int i = 0; i < ROW; ++i)
        for (int j = 0; j < COL; ++j)
            zmap[i][j] = '0';

    auto start = std::chrono::high_resolution_clock::now();
    bool found = ALT(grid, src, dst);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (found) {
        PrintMap();
        PrintPath();
    }
    else {
        std::cout << "Failed to find path.\n";
    }
    std::printf("\nALT Algorithm Execution Time: %.3f milliseconds\n", ms);
    return 0;
} 