#include "Triangle.h"
#include <cmath>
#include <iostream>

using namespace std;

Triangle::Triangle(Vertex &p, Vertex &q, Vertex &s) {
	this->vertexs.push_back(p);
	this->vertexs.push_back(q);
	this->vertexs.push_back(s);
	Edge* e1 = new Edge(p);
	Edge* e2 = new Edge(q);
	Edge* e3 = new Edge(s);
	e1->succ = e2;
	e2->succ = e3;
	e3->succ = e1;
	this->edge = e1;
}

// Triangle::Triangle(const Triangle& t) {
	
// }

Triangle::~Triangle() {
	Edge* temp = edge, *next = temp->succ;
	int i = 0;
	while (temp != edge) {
		cout << ++i << " ";
		delete temp;
		temp = next;
		next = temp->succ;
	}
	cout << endl;
	vertexs.clear();
}

vector<Vertex> Triangle::is_in_triangle(Vertex &p) {
	double a1 = area2(vertexs[0], vertexs[1], p);
	double a2 = area2(vertexs[1], vertexs[2], p);
	double a3 = area2(vertexs[2], vertexs[0], p);
	if ((a1 > 0 && a2 > 0 && a3 > 0) || (a1 < 0 && a2 < 0 && a3 < 0)) {
		return this->vertexs;
	} else if (a1 == 0 && is_on_edge(vertexs[0], vertexs[1], p)) {
		vector<Vertex> v;
		v.push_back(vertexs[0]);
		v.push_back(vertexs[1]);
		return v;
	} else if (a2 == 0 && is_on_edge(vertexs[1], vertexs[2], p)) {
		vector<Vertex> v;
		v.push_back(vertexs[1]);
		v.push_back(vertexs[2]);
		return v;
	} else if (a3 == 0 && is_on_edge(vertexs[2], vertexs[0], p)) {
		vector<Vertex> v;
		v.push_back(vertexs[2]);
		v.push_back(vertexs[0]);
		return v;
	} else {
		return vector<Vertex>();
	}
}

bool Triangle::is_on_edge(Vertex &start, Vertex &end, Vertex &p) {
	double minX = min(start.x, end.x);
	double maxX = max(start.x, end.x);
	return p.x > minX && p.x < maxX;
}

double Triangle::area2(Vertex &p, Vertex &q, Vertex &s) {
	return p.x*q.y - p.y*q.x + q.x*s.y - q.y*s.x + s.x*p.y - s.y*p.x;
}

