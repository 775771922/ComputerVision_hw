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
using namespace cimg_library;
using namespace std;


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

	HoughTransform hough(360, diagonal);
	hough.draw_hough_space(cannyDetectImg);

	CImg<float> houghSpace = hough.get_hough_space();

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

    int errorCount = 12;
    set<Position> s;
    for (int i = 0; i < errorCount; i++) {
    	s.insert(v[i]);
     
        Position p = v[i];
        // draw_line(result, p.x, p.y);
        cout << "x===>" << p.x << "   y====>" << p.y << "   sum====>" << p.sum << endl;
    }

    int errorDis = 5;
    v.clear();
    vector<Position> cluster;
    set<Position>::iterator it;
    for (it = s.begin(); it != s.end(); ++it) {
    	if (v.size() == 0) {
    		v.push_back(*it);
    	}
    	Position p = *it;
    	Position last = v[v.size()-1];
    	if (fabs(sqrt(p.x*p.x+p.y*p.y) - sqrt(last.x*last.x+last.y*last.y)) <= errorDis) {
    		v.push_back(p);
    	} else {
    		cluster.push_back(clusterPos(v));
    		v.clear();
    		v.push_back(p);
    	}
    }

    cluster.push_back(clusterPos(v));

    CImg<float> result(width, height, 1, 1, 0);
    for (int i = 0; i < cluster.size(); i++) {
    	Position p =cluster[i];
    	draw_line(result, p.x, p.y);
    }
    result.save("result.bmp");

    int threshold = 5;
    CImg<float> final(width, height, 1, 1, 0);
    for (int i = 0; i < width; i++) {
    	for (int j = 0; j < height; j++) {
    		final(i, j, 0, 0) = (int)result(i, j, 0, 0) & (int)cannyDetectImg(i, j, 0, 0);
    	}
    }
    final.save("final.bmp");

    for (int i = 0; i < width; i++) {
    	for (int j = 0; j < height; j++) {
    		// if (final(i, j, 0, 0) <= 0) {
    		// 	continue;
    		// }
    		if (result(i, j, 0, 0) <= 0) {
    			continue;
    		}
    		for (int offsetX = -threshold; offsetX <= threshold; offsetX++) {
    			for (int offsetY = -threshold; offsetY <= threshold; offsetY++) {
    				if (i+offsetX >= width || i+offsetX < 0 || j+offsetY >= height || j+offsetY < 0) {
    				   continue;
    				}
    				if (cannyDetectImg(i+offsetX, j+offsetY, 0, 0) > 0) {
    					final(i, j, 0, 0) = 255;
    					// break the loop
    					offsetY = offsetX = threshold;
    				}
    			}
    		}
    	}
    }
    final.save("final2.bmp");
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
	int x = 0, y = 0, sum = v[0].sum;
	for (int i = 0; i < v.size(); i++) {
		x += v[i].x;
		y += v[i].y;
		if (v[i].sum > sum) {
			sum = v[i].sum;
		}
	}
	return Position(x/v.size(), y/v.size(), sum);
}


