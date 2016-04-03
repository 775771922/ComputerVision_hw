#ifndef TRIANGULATION_H_
#define TRIANGULATION_H_

#include "Triangle.h"
#include <list>
#include <vector>
#include <set>
using namespace std;

class Triangulation {
public:
	list<Triangle> triangles;
	Triangulation(int w, int h, const set<Vertex> &vertexs);
	virtual ~Triangulation();
	void insert(Vertex p);
	void setOffset(int minWidth, int maxWidth, int minHeight, int maxHeight);
	void begin_tri();
private:
	int width, height;
	int minWidth, maxWidth, minHeight, maxHeight;
	set<Vertex> vertexs;
	Vertex lt, lb, rt, rb;
	// 初始化最开始四个顶点
	void init_vertex();
	// 插入第一个点更新三角剖分
	void update(Vertex &p);
	// 插入第二个及之后的点，更新三角剖分
	void update(vector<Triangle> &t, vector<Vertex> &v, Vertex &p);
};

#endif /* TRIANGULATION_H_ */
