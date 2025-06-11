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

void tracePath(Pair cellData[8][8], Pair dst) {
	Pair s[MAX];
	int sIdx = 0;
	int x = dst.first;
	int y = dst.second;

	s[sIdx] = {x, y};
	sIdx++;
	while (!(cellData[x][y].first == x && cellData[x][y].second == y)) {
		int tempx = cellData[x][y].first;
		int tempy = cellData[x][y].second;
		x = tempx;
		y = tempy;
		s[sIdx] = {x, y};
		sIdx++;
	}

	while (sIdx != 0) {
		zmap[s[sIdx-1].first][s[sIdx-1].second] = '*';
		path[pIdx] = s[sIdx-1];
		pIdx++;
		sIdx--;
	}
}

bool BFS(int map[8][8], Pair src, Pair dst) {
	if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
	if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;

	bool visited[8][8] = {false};
	Pair queue[MAX];
	int front = 0, rear = 0;
	Pair cellData[8][8];

	visited[src.first][src.second] = true;
	queue[rear++] = src;
	cellData[src.first][src.second] = src;

	while (front != rear) {
		Pair current = queue[front++];
		
		if (current.first == dst.first && current.second == dst.second) {
			tracePath(cellData, dst);
			return true;
		}

		for (int i = 0; i < 4; i++) {
			int nx = current.first + dx[i];
			int ny = current.second + dy[i];

			if (isInRange(nx, ny) && !visited[nx][ny] && isUnBlocked(map, nx, ny)) {
				visited[nx][ny] = true;
				queue[rear++] = {nx, ny};
				cellData[nx][ny] = current;
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
	auto start = std::chrono::high_resolution_clock::now();
	
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
	
	if (BFS(grid, src, dst)) PrintMap();
	else printf("Failed to find path.");

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> execution_time = end - start;
	
	printf("\nBFS Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
	
	int i = 0;
	while (i < pIdx) {
		Pair p = path[i];
		printf("( %d, %d )\n", p.first, p.second);
		i++;
	}
	
	return 0;
}