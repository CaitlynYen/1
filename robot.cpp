#include <cmath>
#include <cstdio>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <set>
#include <queue>
#include <stack>
#include <vector>
#include <string>

using namespace std;

enum Flag {
	EMPTY,
	OBSTACLE,
	PATH,
};

struct Position {
	int x, y;
	Position() : x(0), y(0) {}
	Position(int _x, int _y) : x(_x), y(_y) {}
	bool operator< (const Position &rhs) const { return x < rhs.x || (x == rhs.x && y < rhs.y); }
};

struct Node {
	int x, y;
	int g;
	int h;
	int f;
	struct Node *father;
};

class ComF {
public:
	bool operator()(Node* x, Node* y) const {
		return x->f > y->f;
	}
};

class PathFinder {
private:
	int **m_maze;
	int m_rows, m_cols;
	int m_startX, m_startY;
	int m_endX, m_endY;
	int dx[4], dy[4];

	int **m_path;
	Node *startNode, *endNode;
	vector<Node*> OPENTable;
	vector<Node*> CLOSEDTable;

public:
	PathFinder(int **maze, int rows, int cols);
	~PathFinder();
	void setStartEnd(int sx, int sy, int ex, int ey);
	bool searchPath();
	vector<Position> generatePath();
	int judge(int x, int y);
	bool isIllegle(int x, int y);
	void printPath();
};


class Robot {
private:
	string dir;
	int **o_maze;
	int rows, cols;
	int battery;
	int initX, initY;

	int **v_maze;
	set<Position> uv_positions;

public:
	Robot(string d);
	~Robot();
	void run();
};

PathFinder::PathFinder(int **maze, int rows, int cols)
{
	m_maze = maze;
	m_rows = rows;
	m_cols = cols;

	dx[0] = -1;
	dx[1] = 0;
	dx[2] = 1;
	dx[3] = 0;

	dy[0] = 0;
	dy[1] = 1;
	dy[2] = 0;
	dy[3] = -1;

	m_path = new int *[m_rows];
	for (int i = 0; i < m_rows; ++i)
	{
		m_path[i] = new int[m_cols];
	}
}

PathFinder::~PathFinder()
{
	delete endNode;

	for (int i = 0; i < m_rows; ++i)
	{
		delete[] m_path[i];
	}
	delete[] m_path;

	vector<Node*>::iterator iter;
	for (iter = OPENTable.begin(); iter != OPENTable.end(); ++iter)
	{
		delete (*iter);
	}
	OPENTable.clear();

	vector<Node*>::iterator iter2;
	for (iter2 = CLOSEDTable.begin(); iter2 != CLOSEDTable.end(); ++iter2)
	{
		delete (*iter2);
	}
	CLOSEDTable.clear();
}

void PathFinder::setStartEnd(int sx, int sy, int ex, int ey)
{
	m_startX = sx;
	m_startY = sy;
	m_endX = ex;
	m_endY = ey;

	startNode = new Node;
	startNode->x = m_startX;
	startNode->y = m_startY;
	startNode->g = 0;
	startNode->h = judge(startNode->x, startNode->y);
	startNode->f = startNode->g + startNode->h;
	startNode->father = NULL;

	endNode = new Node;
	endNode->x = m_endX;
	endNode->y = m_endY;
	endNode->father = NULL;
}

bool PathFinder::searchPath()
{
	if (m_startX == m_endX && m_startY == m_endY)
	{
		return false;
	}

	OPENTable.push_back(startNode);
	push_heap(OPENTable.begin(), OPENTable.end(), ComF());
	Node *tempNode = new Node;

	for (;;)
	{
		if (OPENTable.empty())
		{
			return false;
		}

		tempNode = OPENTable.front();
		pop_heap(OPENTable.begin(), OPENTable.end(), ComF());
		OPENTable.pop_back();

		if (tempNode->x == m_endX && tempNode->y == m_endY)
		{
			endNode->g = tempNode->g;
			endNode->h = tempNode->h;
			endNode->f = tempNode->f;
			endNode->father = tempNode->father;

			return true;
		}

		for (int i = 0; i < 4; ++i)
		{
			int nextX = tempNode->x + dx[i];
			int nextY = tempNode->y + dy[i];
			if (isIllegle(nextX, nextY))
			{
				if (EMPTY != m_maze[tempNode->x][nextY] ||
					EMPTY != m_maze[nextX][tempNode->y])
				{
					continue;
				}

				int newGVal;
				if (!dx[i] && !dy[i])
				{
					newGVal = tempNode->g + 14;
				}
				else
				{
					newGVal = tempNode->g + 10;
				}

				vector<Node*>::iterator OPENTableResult;
				for (OPENTableResult = OPENTable.begin();
					OPENTableResult != OPENTable.end();
					++OPENTableResult)
				{
					if ((*OPENTableResult)->x == nextX &&
						(*OPENTableResult)->y == nextY)
					{
						break;
					}
				}

				if (OPENTableResult != OPENTable.end())
				{
					if ((*OPENTableResult)->g <= newGVal)
					{
						continue;
					}
				}

				vector<Node*>::iterator CLOSEDTableResult;
				for (CLOSEDTableResult = CLOSEDTable.begin();
					CLOSEDTableResult != CLOSEDTable.end();
					++CLOSEDTableResult)
				{
					if ((*CLOSEDTableResult)->x == nextX &&
						(*CLOSEDTableResult)->y == nextY)
					{
						break;
					}
				}

				if (CLOSEDTableResult != CLOSEDTable.end())
				{
					if ((*CLOSEDTableResult)->g <= newGVal)
					{
						continue;
					}
				}

				Node *bestNode = new Node;
				bestNode->x = nextX;
				bestNode->y = nextY;
				bestNode->father = tempNode;
				bestNode->g = newGVal;
				bestNode->h = judge(nextX, nextY);
				bestNode->f = bestNode->g + bestNode->h;

				if (CLOSEDTableResult != CLOSEDTable.end())
				{
					delete (*CLOSEDTableResult);
					CLOSEDTable.erase(CLOSEDTableResult);
				}

				if (OPENTableResult != OPENTable.end())
				{
					delete (*OPENTableResult);
					OPENTable.erase(OPENTableResult);

					make_heap(OPENTable.begin(), OPENTable.end(), ComF());
				}

				OPENTable.push_back(bestNode);

				push_heap(OPENTable.begin(), OPENTable.end(), ComF());
			}
		}

		CLOSEDTable.push_back(tempNode);
	}

	return false;
}



int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "ERROR" << endl;
		return -1;
	}


}
