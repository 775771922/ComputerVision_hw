#ifndef _CANNY_H_
#define _CANNY_H_

#include "CImg.h"
#include "gaussian.h"

using namespace cimg_library;

namespace xyz {

class Canny {
public:
	Canny(float sigma = 1.5, int winSize = 1, int firstDerWinSize = 1);
	~Canny() {};
	// canny边缘检测
	CImg<float> detect_edge(CImg<float> &srcImg);
	// 设置滞后阈值
	void set_threshold_value(int high, int low);
private:
	// 滞后阈值
	int high, low;
	Gaussian gaussian;
	// canny检测前对图像进行灰度化
	CImg<float> get_gray_image(const CImg<float> &srcImg);
	// 对图片进行高斯一阶导处理
	CImg<float> get_gaussian_first_derX(const CImg<float> &srcImg);
	CImg<float> get_gaussian_first_derY(const CImg<float> &srcImg);
	// 对x，y方向卷积处理的图片做梯度图
	CImg<float> get_gaussian_gradient(const CImg<float> &dx, const CImg<float> &dy);
	// 极大值抑制处理
	CImg<float> non_maximun_suppression(const CImg<float> &gradient, const CImg<float> &dx, const CImg<float> &dy);
	// 极大值抑制辅助函数
	void non_maximun_suppression(CImg<float> &ret, const CImg<float> &srcImg, int w, int h, double tanTheta);
	// 按照滞后阈值将极大值抑制后离散的点连接起来
	CImg<float> link_edge(const CImg<float> &srcImg);
	
};
}

#endif