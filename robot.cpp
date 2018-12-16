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



int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "ERROR" << endl;
		return -1;
	}

	
}
