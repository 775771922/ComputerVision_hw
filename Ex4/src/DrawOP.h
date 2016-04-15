#ifndef DRAW_OP_H
#define DRAW_OP_H

#include "CImg.h"
using namespace cimg_library;

class DrawOP {
public:
	DrawOP() {};
	CImg<unsigned char> draw_triangle(const CImg<unsigned char>& srcImg, int x1, int y1, 
		int x2, int y2, int x3, int y3);
	CImg<unsigned char> draw_rectangle(const CImg<unsigned char>& srcImg, int left, int top, 
		int right, int bottom);
	CImg<unsigned char> draw_circle(const CImg<unsigned char>& srcImg, int centerX, int centerY, double radius);
};

#endif