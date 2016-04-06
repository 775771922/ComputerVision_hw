#ifndef ROTATE_OP_H
#define ROTATE_OP_H

#include "CImg.h"
#include "global.h"
using namespace cimg_library;

class RotateOP {
public:
    // 旋转图片，旋转角为theta，并采用双线性插值减少锯齿
    RotateOP() {};
    CImg<float> rotate_biliinear(const CImg<float>& srcImg, double theta);
    Position rotate_pos(const Position &srcPos, double theta, int width, int height);
private:
	// 获得旋转后图片的像素对应于原图的像素位置，用于双线性插值
	bool get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
		                double& srcX, double& srcY);
    void biliinear_interpolation(CImg<float>& outImg, 
    	const CImg<float>& srcImg, double theta);
    void nearest_interpolation(CImg<float>& outImg, 
    	const CImg<float>& srcImg, double theta);
    // 检查像素位置，防止超过图片宽度
	int valueWidth(double srcX, int width);
	// 检查像素位置，防止超过图片高度
	int valueHeight(double srcY, int height);
};

#endif 