#ifndef ROTATE_OP_H
#define ROTATE_OP_H

#include "CImg.h"
#include "global.h"
using namespace cimg_library;

class RotateOP {
public:
    RotateOP() {};
    Position rotate_pos(const Position &srcPos, double theta, int width, int height);
    CImg<float> rotate(const CImg<float>& srcImg, double theta);
private:
    // 旋转图片，旋转角为theta，并采用双线性插值减少锯齿
    CImg<float> rotate_biliinear(const CImg<float>& srcImg, double theta);
	// 获得旋转后图片的像素对应于原图的像素位置，用于双线性插值
	bool get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
		            double& srcX, double& srcY);
    // 获得旋转后图片的像素对应于原图的像素位置，用于最邻近插值
    bool get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
                    int& srcX, int& srcY);
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