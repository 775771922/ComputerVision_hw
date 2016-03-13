#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
#include <cassert>
using namespace cimg_library;
using namespace std;

#define OFFSET 180

#define PI 3.141592653

bool check_point(int x, int y, int width, int height) {
	return !(x < 0 || x >= width || y < 0 || y >= height);
}

void draw_point(CImg<unsigned char> &srcImg, int x, int y) {
	if (srcImg != NULL) {
		int width = srcImg.width();
		int height = srcImg.height();
		if (check_point(x, y, width, height)) {
			for (int i = -5; i <= 5; i++) {
				for (int j = -5; j < 5; ++j) {
					if (check_point(x+i, y+j, width, height)) {
						for (int channel = 0; channel < 3; channel++) {
							srcImg(x+i, y+j, 0, channel) = 0;
						}
					}
				}
			}
		}
	}
}

struct Position {
	int x, y;
	Position(int x, int y) {
		this->x = x; this->y = y;
	}
};

void draw_line(CImg<unsigned char> &srcImg, int theta, int p) {
	int width = srcImg.width();
	int height = srcImg.height();
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			double temp_p = (double)i * cos(PI*(theta)/OFFSET) + (double)j * sin(PI*(theta)/OFFSET);
			//cout << temp_p << endl;
			if (p >= temp_p-0.5 && p <= temp_p+0.5) {
				//cout << "i====>" << i << " j=====>" << j << endl;
				for (int channel = 0; channel < 3; channel++) {
					srcImg(i, j, 0, channel) = 150;
				}
			}
		}
	}
}

void draw_hough_space(CImg<unsigned char> &houghSpace, int x, int y) {	
	int width = houghSpace.width();
	int height = houghSpace.height();
	
	for (int theta = 0; theta < 2*OFFSET; theta++) {
		//for (int p = 0; p < height; p++) {
			int temp_p = (double)x * cos(PI*theta/OFFSET) + (double)y * sin(PI*theta/OFFSET);
			// if (temp_p < 0) {
			// 	cout << "x===>" << x << " y===>" << y << " theta===>" << theta << " p===>" << p << " temp_p====>" << temp_p << endl;
			// }
			if (temp_p >= 0) {
				//cout << "x == " << x << " y == " << y << " theta == " << theta << " temp_p == " << temp_p << endl;
				for (int channel = 0; channel < 3; channel++) {
					houghSpace(theta, temp_p, 0, channel) = (houghSpace(theta, temp_p, 0, channel)+50) <= 255?(houghSpace(theta, temp_p, 0, channel)+50):255;
				}
			}
		//}
	}
	
}

void find_best_param(CImg<unsigned char> &houghSpace, int& theta, int& p) {
	int width = houghSpace.width();
	int height = houghSpace.height();
	theta = 0;
	p = 0;
	int max = houghSpace(0, 0, 0, 0);
	for (int i = 0; i < 2*OFFSET; i++) {
		for (int j = 0; j < height; j++) {
			if (houghSpace(i, j, 0, 0) > max) {
				max = houghSpace(i, j, 0, 0);
				theta = i;
				p = j;
			}
		}
	}
}

int main() {
	CImg<unsigned char> srcImg("canvas.bmp");

	draw_point(srcImg, 20, 30);
	draw_point(srcImg, 40, 50);
	draw_point(srcImg, 60, 70);
	draw_point(srcImg, 250, 360);
	draw_point(srcImg, 450, 80);
	draw_point(srcImg, 320, 120);
	draw_point(srcImg, 50, 400);
	draw_point(srcImg, 600, 330);
	draw_point(srcImg, 700, 200);

	//srcImg.display();
	srcImg.save("point.bmp");

    int srcWidth = srcImg.width();
    int srcHeight = srcImg.height();
    int nmax = sqrt(srcWidth*srcWidth+srcHeight*srcHeight);

	CImg<unsigned char> houghSpace(2*OFFSET, nmax, 1, 3, 0);
	draw_hough_space(houghSpace, 20, 30);
	draw_hough_space(houghSpace, 40, 50);
	draw_hough_space(houghSpace, 60, 70);
	draw_hough_space(houghSpace, 250, 360);
	draw_hough_space(houghSpace, 450, 80);
	draw_hough_space(houghSpace, 320, 120);
	draw_hough_space(houghSpace, 50, 400);
	draw_hough_space(houghSpace, 600, 330);
	draw_hough_space(houghSpace, 700, 200);
    houghSpace.save("houghSpace.bmp");

    int theta, p;
    find_best_param(houghSpace, theta, p);

    cout << "theta===>"  << theta << "  p===>" << p << endl;

    draw_line(srcImg, theta, p);
    srcImg.save("line.bmp");

    return 0;

}