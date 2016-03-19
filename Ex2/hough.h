#ifndef _HOUGH_H_
#define _HOUGH_H_

#include "CImg.h"
using namespace cimg_library;

const double PI = 3.1415926;

class HoughTransform {
private:
	int width, height;
	CImg<float> houghSpace;
public:
	const int halfCircle;
	HoughTransform(int w, int h);
	void draw_hough_space(const CImg<float> &srcImg);
	void draw_hough_space(int x, int y);
	CImg<float> get_hough_space();
};

#endif