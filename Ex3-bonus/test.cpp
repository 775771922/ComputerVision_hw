#include "CImg.h"
#include "Triangulation.h"
#include <iostream>
#include <vector>
#include <set>
#include <list>
using namespace std;
using namespace cimg_library;

int main() {
	CImg<unsigned char> srcImg(100, 100, 1, 3, 0);
	set<Vertex> s;
	s.insert(Vertex(10, 10));
	s.insert(Vertex(20, 10));
	s.insert(Vertex(20, 20));
	Triangulation t(100, 100, s);

	set<Vertex>::iterator its;
	for (its = s.begin(); its != s.end(); ++its) {
		cout << "(" << its->x << ", " << its->y << ")\n";
	}

	t.begin_tri();
	list<Triangle> ts = t.triangles;
	list<Triangle>::iterator it;
	for (it = ts.begin(); it != ts.end(); ++it) {
		cout << "(" << it->vertexs[0].x << ", " << it->vertexs[0].y << ") "
		     << "(" << it->vertexs[1].x << ", " << it->vertexs[1].y << ") "
		     << "(" << it->vertexs[2].x << ", " << it->vertexs[2].y << ") " << endl;
	}
}