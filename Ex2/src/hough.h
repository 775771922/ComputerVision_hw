#ifndef _HOUGH_H_
#define _HOUGH_H_

#include "CImg.h"
using namespace cimg_library;


// hough变换类
class HoughTransform {
private:
	int width, height;
	CImg<float> houghSpace;
public:
	const int halfCircle;
	HoughTransform(int w, int h);
	// 对输入图片的每一个像素点坐标做hough变换
	void draw_hough_space(const CImg<float> &srcImg);
	// 对输入的像素坐标做hough变换
	void draw_hough_space(int x, int y);
	CImg<float> get_hough_space();
};

#endif