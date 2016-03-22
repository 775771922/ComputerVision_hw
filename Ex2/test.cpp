#include "CImg.h"
#include "gaussian.h"
#include "canny.h"
#include "hough.h"
#include <cmath>
#include <fstream>
#include <cstring>
#include <string>
#include <cassert>
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>
#include <set>
#include <fstream>
using namespace cimg_library;
using namespace std;

#define TEST_BEBUG

struct Position {
    int x, y, sum;
    Position(int x, int y, int sum) {
        this->x = x;
        this->y = y;
        this->sum = sum;
    }
};

bool operator<(const Position &l, const Position &r) {
	return (l.x*l.x+l.y*l.y) < (r.x*r.x+r.y*r.y);
}

bool cmp(const Position& a, const Position& b) {
    return a.sum > b.sum;
}

void draw_line(CImg<float> &img, int theta, int p);
Position clusterPos(vector<Position> &v);


void print_map(multimap<int, Position> cluster) {
    multimap<int, Position>::iterator it;
    for (it = cluster.begin(); it != cluster.end(); ++it) {
        Position p = it->second;
        cout << it->first << "   (" << p.x << ", " << p.y << ")  " << p.sum << endl;
    }
    cout << endl;
}

void draw_lines(CImg<float> &o, vector<Position> &v) {
    for (int i = 0; i < v.size(); i++) {
        Position p = v[i];
        draw_line(o, p.x, p.y);
    }
    o.display();
}


int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}
	CImg<float> srcImg(argv[1]);

	Canny canny(1.5, 3, 3);
	CImg<float> cannyDetectImg = canny.detect_edge(srcImg);
	
	int width = cannyDetectImg.width();
	int height = cannyDetectImg.height();
	int diagonal = sqrt(width*width + height*height);

	HoughTransform hough(180, diagonal);
	hough.draw_hough_space(cannyDetectImg);

	CImg<float> houghSpace = hough.get_hough_space();
    houghSpace.save("houghSpace.bmp");

    vector<Position> v;
    for (int i = 0; i < houghSpace.width(); i++) {
        for (int j = 0; j < houghSpace.height(); j++) {
            if (houghSpace(i, j, 0, 0) <= 0) {
                continue;
            }
            v.push_back(Position(i, j, houghSpace(i, j, 0, 0)));
        }
    }
    cout << "v.size() =====> " << v.size() << endl;
    sort(v.begin(), v.end(), cmp);

    cout << "finish sort" << endl;

    int rate = 3;

    int threshold = v[0].sum / rate;
    set<Position> s;
    for (int i = 0; i < v.size(); i++) {
        if (v[i].sum < threshold) {
            break;
        }
        s.insert(v[i]);

        //cout << "i====>" << "(" << v[i].x << ", " << v[i].y << ") " << v[i].sum << endl;
    }

    // cout << "draw o.bmp" << endl;
    
        CImg<float> o(width, height, 1, 1, 0);
        ofstream fout("set.txt");
    for (set<Position>::iterator it = s.begin(); it != s.end(); ++it) {
        Position p = *it;
        fout << "(" << p.x << ", " << p.y << ")\n";
    }
    // o.save("o.bmp");


    cout << "s.size()===> " <<  s.size() << endl;

    int errorTheta, errorP;// = 110;
    cin >> errorTheta >> errorP;
    v.clear();
    //vector<Position> cluster;
    multimap<int, Position> cluster;
    set<Position>::iterator it;
    
    for (it = s.begin(); it != s.end(); ++it) {
    	if (v.size() == 0) {
    		v.push_back(*it);
            continue;
    	}
    	Position p = *it;
    	Position last = v[0];
    	if (sqrt((p.x-last.x)*(p.x-last.x) + (p.y-last.y)*(p.y-last.y)) <= errorDis) {
            #ifdef TEST_BEBUG
            cout << "cluster\n";
            fout << "cluster   " << "last: (" << last.x << ", " << last.y << ")" << endl;
            fout << "          " << "cur: (" << p.x << ", " << p.y << ")" << endl << endl;
            #endif
    		v.push_back(p);
            Position center = clusterPos(v);
            v.clear();
            v.push_back(center);
    	} else {
            cluster.insert(make_pair(v[0].sum, v[0]));

            #ifdef TEST_BEBUG
            cout << "uncluster\n";
            //print_map(cluster);
            //draw_lines(o, v);
            #endif
    		//cluster.push_back(clusterPos(v));
    		v.clear();
    		v.push_back(p);
    	}
    }

    // cluster.push_back(clusterPos(v));
    cluster.insert(make_pair(v[0].sum, v[0]));

    #ifdef TEST_BEBUG
    //print_map(cluster);
    //draw_lines(o, v);
    #endif
print_map(cluster);


//o.save("o.bmp");

    CImg<float> result(width, height, 1, 1, 0);
    multimap<int, Position>::reverse_iterator cluster_it = cluster.rbegin();
    for (int count = 1; cluster_it != cluster.rend() && count <= 4; cluster_it++, count++) {
    	Position p = cluster_it->second;
    	draw_line(result, p.x, p.y);
        //#ifdef TEST_BEBUG
        cout << "(" << p.x << ", " << p.y << ")" << endl;
        result.display();
        //#endif
    }
    result.save("result.bmp");

}


void draw_line(CImg<float> &img, int theta, int p) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double temp_p = (double)i * cos(PI*(theta)/180) + (double)j * sin(PI*(theta)/180);
            if (p >= temp_p-0.5 && p <= temp_p+0.5) {
                //cout << "i====>" << i << " j=====>" << j << endl;
                for (int channel = 0; channel < 1; channel++) {
                    img(i, j, 0, channel) = 0xff;
                }
            }
        }
    }
}

Position clusterPos(vector<Position> &v) {
	int x = 0, y = 0;// sum = v[0].sum;
    int sum = 0;
	for (int i = 0; i < v.size(); i++) {
		x += v[i].x;
		y += v[i].y;
		// if (v[i].sum > sum) {
		// 	sum = v[i].sum;
		// }
        sum += v[i].sum;
	}
	return Position((double)x/v.size(), (double)y/v.size(), sum/v.size());
}


