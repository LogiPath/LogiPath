#include <stdio.h>
#include <iostream>
#include <chrono>

using namespace std;




// 참고 : https://velog.io/@dpmawile/astar

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
};


char zmap[MAX][MAX];
int ROW = 0, COL = 0;

Pair path[MAX];
int pIdx = 0;

bool isDestination(int row, int col, Pair dst) {
	if (row == dst.first && col == dst.second) return true;
	return false;
}

bool isInRange(int row, int col) {
	return (row >= 0 && row < ROW && col >= 0 && col < COL);
}

bool isUnBlocked(int map[8][8], int row, int col) {
	return (map[row][col] == 0);
}

double GethValue(int row, int col, Pair dst) {
	return (double) sqrt(pow(row - dst.first, 2) + pow(col - dst.second, 2));
}

void tracePath(Cell cellDetails[MAX][MAX], Pair dst) {
	Pair s[MAX];
	int sIdx = 0;
	int y = dst.first;
	int x = dst.second;

	s[sIdx] = { y, x };
	sIdx++;
	// cellDetails x, y 좌표를 시작점으로 설정
	while (!(cellDetails[y][x].parent_x == x && cellDetails[y][x].parent_y == y)) {
		int tempy = cellDetails[y][x].parent_y;
		int tempx = cellDetails[y][x].parent_x;
		y = tempy;
		x = tempx;
		s[sIdx] = { y, x };
		sIdx++;
	}


	while (sIdx != 0 ) {
		zmap[s[sIdx-1].first][s[sIdx-1].second] = '*';
		path[pIdx] = s[sIdx-1];
		pIdx++; sIdx--;
	}
}

bool aStarSearch(int map[8][8], Pair src, Pair dst) {
	if (!isInRange(src.first, src.second) || !isInRange(dst.first, dst.second)) return false;
	if (!isUnBlocked(map, src.first, src.second) || !isUnBlocked(map, dst.first, dst.second)) return false;
	if (isDestination(src.first, src.second, dst)) return false;

	bool closedList[MAX][MAX];
	std::memset(closedList, false, sizeof(closedList));

	Cell cellDetails[MAX][MAX];

	// 초기화
	// 모든 셀의 f, g, h 값을 INF로 초기화
	// parent_x와 parent_y를 -1로 초기화

	for (int i = 0; i < ROW; ++i) {
		for (int j = 0; j < COL; ++j) {
			cellDetails[i][j].f = cellDetails[i][j].g = cellDetails[i][j].h = INF;
			cellDetails[i][j].parent_x = cellDetails[i][j].parent_y = -1;
		}
	}

	// src 좌표를 시작점으로 설정
	int sy = src.first;
	int sx = src.second;
	cellDetails[sy][sx].f = cellDetails[sy][sx].g = cellDetails[sy][sx].h = 0.0;
	cellDetails[sy][sx].parent_x = sx;
	cellDetails[sy][sx].parent_y = sy;

	pPair openList[MAX];
	int oSIdx = 0, oEIdx = 0;
	openList[oEIdx] = { 0.0, { sy, sx } };
	oEIdx++;


	while (oEIdx != oSIdx ) {
		pPair p = openList[oSIdx];
		oSIdx++;

		int y = p.second.first;
		int x = p.second.second;
		closedList[y][x] = true;

		double ng, nf, nh;

		// ����
		for (int i = 0; i < 4; ++i) {
			int ny = y + dy1[i];
			int nx = x + dx1[i];

			if (isInRange(ny, nx)) {
				if (isDestination(ny, nx, dst)) {
					cellDetails[ny][nx].parent_y = y;
					cellDetails[ny][nx].parent_x = x;
					tracePath(cellDetails, dst);
					return true;
				}

				// bfs�� ���� �����ڸ�, closedList�� �湮���ζ�� �����Ͻø� �˴ϴ�.
				else if (!closedList[ny][nx] && isUnBlocked(map, ny, nx)) {
					// �̺κ� y x, ny nx �򰥸��°� ����
					ng = cellDetails[y][x].g + 1.0;
					nh = GethValue(ny, nx, dst);
					nf = ng + nh;

					// ���� �ѹ��� ������ �ȵ�f�ų�, ���ΰ��ŵ� f�� ����f���� ������ ��
					if (cellDetails[ny][nx].f == INF || cellDetails[ny][nx].f > nf) {
						cellDetails[ny][nx].f = nf;
						cellDetails[ny][nx].g = ng;
						cellDetails[ny][nx].h = nh;
						cellDetails[ny][nx].parent_x = x;
						cellDetails[ny][nx].parent_y = y;
						openList[oEIdx] = { nf, { ny, nx } };
						oEIdx++;
					}
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

	bool isProcessed = aStarSearch(grid, src, dst);

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> execution_time = end - start;

	if (isProcessed) {
		PrintMap();
		PrintPath();
	} else {
		printf("Failed to find path.\n");
	}

	printf("\nA* Algorithm Execution Time: %.3f milliseconds\n", execution_time.count());
	
	return 0;
}