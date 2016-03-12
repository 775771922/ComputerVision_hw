#ifndef ROTATE_OP_H
#define ROTATE_OP_H

#include "CImg.h"
using namespace cimg_library;

// 像素位置结构体
struct Position {
	int x, y;
	Position(int x, int y) {
		this->x = x;
		this->y = y;
	}
};

class RotateOP {
public:
	// 旋转图片，旋转角为theta，并采用最邻近插值减少锯齿
    CImg<unsigned char> rotate_nearest(const CImg<unsigned char>& srcImg, double theta);
    // 旋转图片，旋转角为theta，并采用双线性插值减少锯齿
    CImg<unsigned char> rotate_biliinear(const CImg<unsigned char>& srcImg, double theta);
    RotateOP() {} ;

private:
	// 获得旋转后图片的像素对应于原图的像素位置，用于最邻近插值
	bool get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
	                int& srcX, int& srcY);
	// 获得旋转后图片的像素对应于原图的像素位置，用于双线性插值
	bool get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
		                double& srcX, double& srcY);
    void biliinear_interpolation(CImg<unsigned char>& outImg, 
    	const CImg<unsigned char>& srcImg, double theta);
    void nearest_interpolation(CImg<unsigned char>& outImg, 
    	const CImg<unsigned char>& srcImg, double theta);
    // 检查像素位置，防止超过图片宽度
	int valueWidth(double srcX, int width);
	// 检查像素位置，防止超过图片高度
	int valueHeight(double srcY, int height);
};

#endif 