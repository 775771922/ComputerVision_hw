#ifndef SCALE_OP_H
#define SCALE_OP_H

#include "CImg.h"
using namespace cimg_library;

class ScaleOP {
public:
	ScaleOP() {};
	int valueWidth(double srcX, int width);
	int valueHeight(double srcY, int height);
	CImg<unsigned char> nearest_scale(const CImg<unsigned char>& srcImg, int width, int height);
	CImg<unsigned char> bilinear_scale(const CImg<unsigned char>& srcImg, int width, int height);
};

#endif