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

vector<Position> PathFinder::generatePath()
{
	vector<Position> result;

	Node *nodeChild = endNode;
	Node *nodeParent = endNode->father;
	do
	{
		m_path[nodeChild->x][nodeChild->y] = PATH;
		result.push_back(Position(nodeChild->x, nodeChild->y));

		nodeChild = nodeParent;
		nodeParent = nodeParent->father;
	} while (nodeChild != startNode);

	m_path[startNode->x][startNode->y] = PATH;
	result.push_back(Position(nodeChild->x, nodeChild->y));
	reverse(result.begin(), result.end());

	return result;
}

void PathFinder::printPath() {
	for (int i = 0; i < m_rows; ++i)
	{
		for (int j = 0; j < m_cols; ++j)
		{
			if (PATH == m_path[i][j])
			{
				cout << "# ";
			}
			else
			{
				cout << m_maze[i][j] << " ";
			}
		}
		cout << endl;
	}
}

int PathFinder::judge(int x, int y)
{
	return 10 * (abs(m_endX - x) + abs(m_endY - y));
}

bool PathFinder::isIllegle(int x, int y)
{
	if (x >= 0 && x < m_rows &&
		y >= 0 && y < m_cols &&
		m_maze[x][y] == EMPTY)
		return true;
	else
		return false;
}


Robot::Robot(string d) : dir(d)
{
	ifstream mapFileStream("./" + dir + "/floor.data");
	string line;

	getline(mapFileStream, line);
	istringstream iss(line);
	iss >> rows >> cols >> battery;
	o_maze = new int *[rows];
	v_maze = new int *[rows];
	for (int i = 0; i < rows; ++i)
	{
		o_maze[i] = new int[cols];
		v_maze[i] = new int[cols];
	}

	int flag;
	for (int i = 0; i < rows; ++i)
	{
		getline(mapFileStream, line);
		istringstream iss(line);
		for (int j = 0; j < cols; ++j)
		{
			if (iss >> flag)
			{
				o_maze[i][j] = flag;
				v_maze[i][j] = flag;

				if (flag == EMPTY)
				{
					uv_positions.insert(Position(i, j));
				}
			}
			else
			{
				o_maze[i][j] = EMPTY;
				v_maze[i][j] = EMPTY;
				initX = i;
				initY = j;
				iss.clear();
				iss.ignore();
			}
		}
	}
}

vector<Position> tryOnce(int **maze, int rows, int cols, int sx, int sy, int ex, int ey)
{
	PathFinder pf(maze, rows, cols);
	pf.setStartEnd(sx, sy, ex, ey);
	if (pf.searchPath())
	{
		return pf.generatePath();
	}
	return vector<Position>();
}

void Robot::run()
{
	vector<Position> all_path;
	Position start(initX, initY);
	set<Position>::iterator end;
	while (!uv_positions.empty())
	{
		int r = rand() % uv_positions.size();
		end = uv_positions.begin();
		while (r--)
		{
			end++;
		}

		vector<Position> vec1, vec2;
		vec1 = tryOnce(v_maze, rows, cols, start.x, start.y, end->x, end->y);
		if (vec1.size() == 0)
			vec1 = tryOnce(o_maze, rows, cols, start.x, start.y, end->x, end->y);
		vec2 = tryOnce(v_maze, rows, cols, end->x, end->y, initX, initY);
		if (vec2.size() == 0)
			vec2 = tryOnce(o_maze, rows, cols, end->x, end->y, initX, initY);

		if (vec1.size() == 0 || vec2.size() == 0 || vec1.size() + vec2.size() - 2 > battery)
		{
			continue;
		}

		for (vector<Position>::iterator iter = vec1.begin(); iter != vec1.end(); ++iter)
		{
			v_maze[iter->x][iter->y] = PATH;
		}
		for (vector<Position>::iterator iter = vec2.begin(); iter != vec2.end(); ++iter)
		{
			v_maze[iter->x][iter->y] = PATH;
		}

		for (set<Position>::iterator iter = uv_positions.begin(); iter != uv_positions.end();)
		{
			if (PATH == v_maze[iter->x][iter->y])
				iter = uv_positions.erase(iter);
			else
				iter++;
		}

		all_path.insert(all_path.end(), vec1.begin(), vec1.end() - 1);
		all_path.insert(all_path.end(), vec2.begin(), vec2.end());
		start = all_path[all_path.size() - 2];
	}
	all_path.erase(all_path.begin());

	ofstream outfile("./" + dir + "/final.path");
	outfile << all_path.size() << endl;
	for (vector<Position>::iterator iter = all_path.begin(); iter != all_path.end(); ++iter)
	{
		outfile << iter->x << " " << iter->y << endl;
	}
}

Robot::~Robot()
{
	for (int i = 0; i < rows; ++i)
	{
		delete[] o_maze[i];
		delete[] v_maze[i];
	}
	delete[] o_maze;
	delete[] v_maze;
}


int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "ERROR" << endl;
		return -1;
	}

	Robot robot(argv[1]);
	robot.run();
	return 0;
}
