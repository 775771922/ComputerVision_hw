#include "DrawOP.h"

/**
* draw triangle
* params: 
* srcImg: the source Image
* x1 y1 x2 y3 x3 y3: the position of three vertex
* return:
* the new image after drawn
*/
CImg<unsigned char> DrawOP::draw_triangle(const CImg<unsigned char>& srcImg, int x1, int y1, 
	int x2, int y2, int x3, int y3) {
	if (srcImg != NULL) {
		CImg<unsigned char> outImg(srcImg);
		int width = srcImg.width();
		int height = srcImg.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				double vx1 = x2 - x1, vy1 = y2 - y1, vx2 = x3 - x1, vy2 = y3 - y1;
				double vx0 = i - x1, vy0 = j - y1;
				double u = (vx2*vy0 - vx0*vy2) / (double)(vx2*vy1 - vx1*vy2),
				       v = (vx1*vy0 - vx0*vy1) / (double)(vx1*vy2 - vx2*vy1);
				if (u >= 0 && v >= 0 && u+v <= 1) {
					for (int channel = 0; channel < 3; channel++) {
						outImg(i, j, 0, channel) = 0;
					}
				}
			}
		}
		return outImg;
	} else {
		return srcImg;
	}
}

/**
* draw rectangle
* params: 
* srcImg: the source Image
* left top right bottom: the position of left top point and right bottom point
* return:
* the new image after drawn
*/
CImg<unsigned char> DrawOP::draw_rectangle(const CImg<unsigned char>& srcImg, int left, int top, 
	int right, int bottom) {
	if (srcImg != NULL) {
		CImg<unsigned char> outImg(srcImg);
		int width = srcImg.width();
		int height = srcImg.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				if (i >= left && i <= right && j >= top && j <= bottom) {
					for (int channel = 0; channel < 3; channel++) {
						outImg(i, j, 0, channel) = 0;
					}
				}
			}
		}
		return outImg;
	} else {
		return srcImg;
	}
}

/**
* draw circle
* params: 
* srcImg: the source Image
* centerX centerY:  center of circle
* radius:  radius of corcle
* return:
* the new image after drawn
*/
CImg<unsigned char> DrawOP::draw_circle(const CImg<unsigned char>& srcImg, int centerX, 
	int centerY, double radius) {
	if (srcImg != NULL) {
		CImg<unsigned char> outImg(srcImg);
		int width = srcImg.width();
		int height = srcImg.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				if ((i - centerX)*(i - centerX)+(j-centerY)*(j-centerY) <= radius*radius) {
					for (int channel = 0; channel < 3; channel++) {
						outImg(i, j, 0, channel) = 0;
					}
				}
			}
		}
		return outImg;
	} else {
		return srcImg;
	}
}








