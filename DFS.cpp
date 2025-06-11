#include <stdio.h>
#include <chrono>

struct pair {
	int first;
	int second;
};

pair dir[4] = {// Ž ⿡   ̰ ޶ , ׸ ϱ ǥ 񱳷     
	{0, 1},
	{1, 0},
	{-1, 0},
	{0, -1},
};

int graph[8][8] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
};
bool visted[8][8];
pair path[64];
int pIdx = 0;

bool flag = false;

void DFS(pair src, pair dst) {
	if (src.first < 0 || src.first > 7 || src.second < 0 || src.second > 7||flag) {
		return;
	}

	visted[src.first][src.second] = true;
	//printf("%d, %d\n", src.first, src.second);

	path[pIdx++] = src;

	if (src.first == dst.first && src.second == dst.second) {
		flag = true;
		return;
	}

	for (int i = 0; i < 4; i++) {
		int x = src.first + dir[i].first, y = src.second + dir[i].second;
		if (graph[x][y] == 0 && !visted[x][y]) {
			DFS({ x, y }, dst);
		}
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
	
	if (DFS(grid, src, dst)) PrintMap();
	else printf("실패.");

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> execution_time = end - start;
	
	printf("\nDFS 알고리즘 실행 시간: %.6f 초\n", execution_time.count());
	
	int i = 0;
	while (i < pIdx) {
		Pair p = path[i];
		printf("( %d, %d )\n", p.first, p.second);
		i++;
	}
	
	return 0;
}