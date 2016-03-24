#include "hough.h"
#include "global.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

HoughTransform::HoughTransform(int w, int h): houghSpace(w, h, 1, 1, 0), halfCircle(180) {
	width = w;
	height = h;
}

CImg<float> HoughTransform::get_hough_space() {
	return houghSpace;
}

void HoughTransform::draw_hough_space(const CImg<float> &srcImg) {
	int w = srcImg.width();
	int h = srcImg.height();
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			if (srcImg(i, j, 0, 0) <= 0) {
				continue;
			}
			draw_hough_space(i, j);
		}
	}
}

/**
* hough变换公式：p = x*cosT + y*sinT
* 这里角度的单位取 1/180*PI ，即1度
**/
void HoughTransform::draw_hough_space(int x, int y) {
	int minWidth = min(2*halfCircle, width);
	for (int theta = 0; theta < minWidth; theta++) {
		int p = (double)x * cos(PI*(double)theta/(double)halfCircle) + (double)y * sin(PI*(double)theta/(double)halfCircle);
		if (p >= 0 && p < height) {
			houghSpace(theta, p, 0, 0) = houghSpace(theta, p, 0, 0) + 1;
		}
	}
}