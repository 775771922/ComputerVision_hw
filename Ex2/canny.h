#ifndef _CANNY_H_
#define _CANNY_H_

#include "CImg.h"
#include "gaussian.h"

using namespace cimg_library;

class Canny {
public:
	Canny(float sigma = 1.5, int winSize = 1, int firstDerWinSize = 1);
	~Canny() {};
	CImg<float> detect_edge(CImg<float> &srcImg);
	void set_threshold_value(int high, int low);
private:
	// 滞后阈值
	int high, low;
	Gaussian gaussian;
	CImg<float> get_gaussian_first_derX(const CImg<float> &srcImg);
	CImg<float> get_gaussian_first_derY(const CImg<float> &srcImg);
	CImg<float> get_gaussian_gradient(const CImg<float> &dx, const CImg<float> &dy);
	CImg<float> non_maximun_suppression(const CImg<float> &gradient, const CImg<float> &dx, const CImg<float> &dy);
	void non_maximun_suppression(CImg<float> &ret, const CImg<float> &srcImg, int w, int h, double tanTheta);
	CImg<float> link_edge(const CImg<float> &srcImg);
	CImg<float> get_gray_image(const CImg<float> &srcImg);
};

#endif