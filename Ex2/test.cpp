#include "CImg.h"
#include "gaussian.h"
#include "canny.h"
#include "hough.h"
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>
#include <fstream>
using namespace cimg_library;
using namespace std;

//#define TEST_DEBUG

struct Position {
    int x, y, sum;
    Position(int x, int y, int sum) {
        this->x = x;
        this->y = y;
        this->sum = sum;
    }
};

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
    cannyDetectImg.save("canny.bmp");
	
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
    sort(v.begin(), v.end(), cmp);

    double rate = 0.4;

    int threshold = v[0].sum * rate;
    for (int i = 0; i < v.size(); i++) {
        if (v[i].sum < threshold) {
            v.erase(v.begin()+i, v.end());
            break;
        }
    }
    
    #ifdef TEST_DEBUG
    CImg<float> o(width, height, 1, 1, 0);
    ofstream fout("set.txt");
    for (int i = 0; i < v.size(); i++) {
        Position p = v[i];
        fout << "(" << p.x << ", " << p.y << ")" << " " << p.sum << "\n";
        draw_line(o, p.x, p.y);
    }
    o.save("o.bmp");
    #endif

    int errorTheta = 10, errorP = 125;
    //cin >> errorTheta >> errorP;
    multimap<int, Position> cluster;
    multimap<int, Position>::iterator it;
    for (int i = 0; i < v.size(); ++i) {
        if (cluster.size() == 0) {
            cluster.insert(make_pair(v[i].sum, v[i]));
            continue;
        }
        bool flag = false;
        for (it = cluster.begin(); it != cluster.end(); ++it) {
            Position p = it->second;
            if ((abs(p.x-v[i].x) <= errorTheta || abs(abs(p.x-v[i].x) - 360) <= errorTheta)  && abs(p.y-v[i].y) <= errorP) {
                #ifdef TEST_DEBUG
                fout << "merge===>*it :(" << v[i].x << ", " << v[i].y << ")\n";
                fout << "     position:(" << p.x << ", " << p.y << ")\n";
                #endif
                cluster.erase(it);
                if (abs(abs(p.x-v[i].x) - 360) <= errorTheta) {
                    Position newPos((p.x+v[i].x+360)/2, (p.y+v[i].y)/2, (p.sum+v[i].sum)/2);    
                    cluster.insert(make_pair(newPos.sum, newPos));
                } else {
                    Position newPos((p.x+v[i].x)/2, (p.y+v[i].y)/2, (p.sum+v[i].sum)/2);
                    cluster.insert(make_pair(newPos.sum, newPos));
                }
                flag = true;
                break;
            }
        }
        if (!flag) {
            cluster.insert(make_pair(v[i].sum, v[i]));
        }
    }

    print_map(cluster);

    CImg<float> result(width, height, 1, 1, 0);
    multimap<int, Position>::reverse_iterator reverseIt = cluster.rbegin();
    for (int count = 1; reverseIt != cluster.rend() && count <= 4; reverseIt++, count++) {
    	Position p = reverseIt->second;
    	draw_line(result, p.x, p.y);
        #ifdef TEST_DEBUG
        cout << "(" << p.x << ", " << p.y << ")" << endl;
        result.display();
        #endif
    }
    result.save("result.bmp");

}


void draw_line(CImg<float> &img, int theta, int p) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double temp_p = (double)i * cos(PI*(double)theta/180) + (double)j * sin(PI*(double)theta/180);
            if (p >= temp_p-0.5 && p <= temp_p+0.5) {
                for (int channel = 0; channel < 1; channel++) {
                    img(i, j, 0, channel) = 0xff;
                }
            }
        }
    }
}

Position clusterPos(vector<Position> &v) {
	int x = 0, y = 0, sum = 0;
	for (int i = 0; i < v.size(); i++) {
		x += v[i].x;
		y += v[i].y;
        sum += v[i].sum;
	}
	return Position((double)x/v.size(), (double)y/v.size(), sum/v.size());
}


