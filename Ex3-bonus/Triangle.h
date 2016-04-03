#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <vector>
using namespace std;

struct Vertex {
	int x, y;
	bool operator==(const Vertex &v) const {
		return this->x == v.x && this->y == v.y;
	}
	bool operator<(const Vertex &v) const {
		if (x > v.x) {
			return true;
		} else if (x == v.x) {
			return y > v.y;
		}
		return false;
	}
	Vertex() {}
	Vertex(int x, int y) {
		this->x = x;
		this->y = y;
	}
	Vertex(const Vertex& v) {
		this->x = v.x;
		this->y = v.y;
	}
	void operator=(const Vertex &v) {
		this->x = v.x;
		this->y = v.y;
	}
};

struct Edge {
	Vertex ori;
	//Edge* pred;
	Edge* succ;
	Edge() {}
	Edge(Vertex &o) {
		this->ori = o;
	}
	// Edge(const Edge& e) {
	// 	this->ori = e.ori;
	// 	this->succ = new Edge();
	// 	this->succ->ori = e.succ->ori;
	// 	this->succ->succ = NULL;
	// }
};

class Triangle {
public:
	Edge* edge;
	vector<Vertex> vertexs;
	vector<Vertex> is_in_triangle(Vertex &p);
	Triangle(Vertex &p, Vertex &q, Vertex &s);
	//Triangle(const Triangle& t);
	virtual ~Triangle();
private:
	double area2(Vertex &p, Vertex &q, Vertex &s);
	bool is_on_edge(Vertex &start, Vertex &end, Vertex &p);
};

#endif /* TRIANGLE_H_ */
