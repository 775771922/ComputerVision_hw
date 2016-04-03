#include "Triangulation.h"
#include <cassert>
#include <iostream>
using namespace std;

void print(list<Triangle> &t) {
	list<Triangle>::iterator it;
	for (it = t.begin(); it != t.end(); ++it) {
		cout << "(" << it->vertexs[0].x << ", " << it->vertexs[0].y << ") "
		     << "(" << it->vertexs[1].x << ", " << it->vertexs[1].y << ") "
		     << "(" << it->vertexs[2].x << ", " << it->vertexs[2].y << ")\n";
	}
}

Triangulation::Triangulation(int w, int h, const set<Vertex> &s) {
	this->width = w;
	this->height = h;
	this->minWidth = 0, this->maxWidth = w-1;
	this->minHeight = 0, this->maxHeight = h-1;
	this->vertexs = s;
	init_vertex();
}

void Triangulation::setOffset(int minWidth, int maxWidth, int minHeight, int maxHeight) {
	this->minWidth = minWidth;
	this->maxWidth = maxWidth;
	this->minHeight = minHeight;
	this->maxHeight = maxHeight;
	init_vertex();
}

void Triangulation::begin_tri() {
	set<Vertex>::iterator it;
	for (it = vertexs.begin(); it != vertexs.end(); ++it) {
		insert(*it);
	}
}

void Triangulation::insert(Vertex p) {
	cout << "insert: " << p.x << " " << p.y << endl;

	if (triangles.size() == 0) {
		cout << "v.size()===>" << 0 << endl;
		update(p);
	} else {
		list<Triangle>::iterator it;
		vector<Vertex> v;
		vector<Triangle> t;
		bool first_find_on_side = true;
		for (it = triangles.begin(); it != triangles.end();) {
			v = it->is_in_triangle(p);
			if (v.size() == 3) {
				cout << "v.size()===>" << v.size() << endl;
				t.push_back(*it);
				triangles.erase(it++);
				update(t, v, p);
				return;
			} else if (v.size() == 2) {    // 点在边上，可能存在两个三角形
				t.push_back(*it);
				triangles.erase(it++);
				cout << "v.size()===>" << v.size() << endl;
				//remove(*it);
			} else {
				it++;
			}
		}
		// 在边上的情况
		if (t.size() > 0) {
			update(t, v, p);
		}
	}
}

void Triangulation::init_vertex() {
	this->lt.x = minWidth, this->lt.y = minHeight;
	this->lb.x = minWidth, this->lb.y = maxHeight;
	this->rt.x = maxWidth, this->rt.y = minHeight;
	this->rb.x = maxWidth, this->rb.y = maxHeight;
}

// 插入第一个点更新三角剖分
void Triangulation::update(Vertex &p) {
	assert(triangles.size() == 0);
	triangles.push_back(Triangle(lt, lb, p));
	triangles.push_back(Triangle(lt, rt, p));
	triangles.push_back(Triangle(rt, rb, p));
	triangles.push_back(Triangle(lb, rb, p));
	// cout << "update=====>" << triangles.size() << endl;
	// print(triangles);
}

// 插入第二个及之后的点，更新三角剖分
void Triangulation::update(vector<Triangle> &t, vector<Vertex> &v, Vertex &p) {
	assert(t.size() == 1 || t.size() == 2);
	if (t.size() == 1) {
		vector<Vertex> vertexs = t[0].vertexs;
		Triangle t1(vertexs[0], vertexs[1], p);
		Triangle t2(vertexs[1], vertexs[2], p);
		Triangle t3(vertexs[2], vertexs[0], p);
		triangles.push_back(t1);
		triangles.push_back(t2);
		triangles.push_back(t3);
	} else if (t.size() == 2) {
		Edge* e1 = t[0].edge;
		while (e1->ori == v[0] || e1->ori == v[1]) {
			e1 = e1->succ;
		}
		Edge* e2 = t[1].edge;
		while (e2->ori == v[0] || e2->ori == v[1]) {
			e2 = e2->succ;
		}
		Triangle t1(e1->ori, v[0], p);
		Triangle t2(e1->ori, v[1], p);
		Triangle t3(e2->ori, v[0], p);
		Triangle t4(e2->ori, v[1], p);
		triangles.push_back(t1);
		triangles.push_back(t2);
		triangles.push_back(t3);
		triangles.push_back(t4);
	}
	// cout << "update=====>" << triangles.size() << endl;
	// print(triangles);
}

Triangulation::~Triangulation() {
	this->triangles.clear();
	this->vertexs.clear();
}
