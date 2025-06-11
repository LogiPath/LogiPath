#include <stdio.h>
#include <chrono>

constexpr int MAX = 64;

struct Pair {
	int first;
	int second;
};

// 상하좌우
const int dx[4] = {0, 0, 1, -1};
const int dy[4] = {-1, 1, 0, 0};

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

bool DFS(int map[8][8], Pair src, Pair dst) {
	static bool visited[8][8] = {false};
	
	if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
	if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
	
	visited[src.first][src.second] = true;
	zmap[src.first][src.second] = '*';
	path[pIdx++] = src;
	
	if (src.first == dst.first && src.second == dst.second) {
		return true;
	}
	
	for (int i = 0; i < 4; i++) {
		int nx = src.first + dx[i];
		int ny = src.second + dy[i];
		
		if (isInRange(nx, ny) && !visited[nx][ny] && isUnBlocked(map, nx, ny)) {
			if (DFS(map, {nx, ny}, dst)) {
				return true;
			}
		}
	}
	
	visited[src.first][src.second] = false;
	zmap[src.first][src.second] = '0';
	pIdx--;
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

	bool isProcessed = DFS(grid, src, dst);

	auto end = std::chrono::high_resolution_clock::now();

	if (isProcessed) {
		PrintMap();
		PrintPath();
	} else {
		printf("Failed to find path.\n");
	}

	std::chrono::duration<double, std::milli> execution_time = end - start;
	
	printf("\nDFS Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
	
	return 0;
}