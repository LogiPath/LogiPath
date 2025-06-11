#include <stdio.h>
#include <chrono>

struct pair {
	int first;
	int second;
};

pair dir[4] = {// 상하좌우
	{1, 0},
	{0, -1},
	{0, 1},
	{-1, 0},
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
pair queue[64];
int front = 0, rear = 0;

pair path[64];
int pIdx = 0;
int pLen = 0;

pair cellData[8][8];

bool flag = false;


void tracePath(pair cellData[8][8], pair dst) {
	pair s[64];
	int sIdx = 0;
	int x = dst.first;
	int y = dst.second;

	s[sIdx] = { x, y };
	sIdx++;
	while (!(cellData[x][y].first == x && cellData[x][y].second == y)) {
		int tempx = cellData[x][y].first;
		int tempy = cellData[x][y].second;
		x = tempx;
		y = tempy;
		s[sIdx] = { x, y };
		sIdx++;
	}


	while (sIdx != 0) {
		path[pIdx++] = s[sIdx-- - 1];
	}
}


void BFS(pair src, pair dst) {

	int x = src.first, y = src.second;
	visted[x][y] = true;
	queue[rear++] = { x,y };
	cellData[x][y] = { x, y };
	

	while (!flag) {
		x = queue[front % 64].first;
		y = queue[front % 64].second;
		front++;
		for (int i = 0; i < 4; i++) {
			int dx = x + dir[i].first;
			int dy = y + dir[i].second;

			if ((dx >= 0 && dx < 8) && (dy >= 0 && dy < 8)) {
				if (dx == dst.first && dy == dst.second) {
					cellData[dx][dy] = { x, y };
					tracePath(cellData, dst);
					flag = true;
				}
				else if (!visted[dx][dy] && graph[x][y] == 0) {
					visted[dx][dy] = true;
					queue[rear++ % 64] = { dx, dy };
					cellData[dx][dy] = { x, y };
				}
			}

			
		}
		

		
	}

}




int main() {
	auto start = std::chrono::high_resolution_clock::now();
	
	pair src = {6, 0}, dst = {0, 4};
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
	else printf("실패.");

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> execution_time = end - start;
	
	printf("\nBFS 알고리즘 실행 시간: %.6f 초\n", execution_time.count());
	
	int i = 0;
	while (i < pIdx) {
	LARGE_INTEGER frequency;
	LARGE_INTEGER start;
	LARGE_INTEGER end;
	double execution_time;

	// 성능 카운터 주파수 획득
	QueryPerformanceFrequency(&frequency);

	// 실행 시간 측정 시작
	QueryPerformanceCounter(&start);

	BFS({ 6, 0 }, { 0, 4 });

	// 실행 시간 측정 종료
	QueryPerformanceCounter(&end);
	execution_time = (double)(end.QuadPart - start.QuadPart) / frequency.QuadPart;

	printf("\nBFS 알고리즘 실행 시간: %.6f 초\n", execution_time);

	printf("경로\n");
	for (int i = 0; i < pIdx; i++) {
		printf("%d, %d||\t%d\n", path[i].first, path[i].second, graph[path[i].first][path[i].second]);
	}

	return 0;
}