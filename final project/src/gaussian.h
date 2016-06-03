#ifndef _GAUSSIAN_H_
#define _GAUSSIAN_H_

#include "CImg.h"
using namespace cimg_library;

class Gaussian {
private:
	// 高斯函数窗口大小
	int gaussianWinSize;
	// 高斯函数卷积
	float **mask;
	// 高斯一阶导窗口大小
	int firstDerWinSize;
	// 高斯一阶导，x和y方向的卷积
	float *maskX, *maskY;
	float sigma;
public:
	Gaussian(float sigma = 1.5, int winSize = 1, int firstDerWinSize = 1);
	~Gaussian();
	float* get_first_der_maskX() const;
	float* get_first_der_maskY() const;
	float** get_gaussian_mask() const;
	// 对输入的图片做高斯函数x方向的一阶导卷积
    CImg<float> filter_img_by_first_derX(const CImg<float> &srcImg);
    // 对输入的图片做高斯函数y方向的一阶导卷积
    CImg<float> filter_img_by_first_derY(const CImg<float> &srcImg);
    // 对输入的图片做高斯函数卷积
    CImg<float> filter_img_by_gaussian(const CImg<float> &srcImg);

};

#endif