#include "canny.h"
#include <cassert>

//#define CANNY_DEBUG

Canny::Canny(float sigma, int winSize, int firstDerWinSize)
     :gaussian(sigma, winSize, firstDerWinSize) {
      	high = 35;
      	low = 15;
}

void Canny::set_threshold_value(int high, int low) {
	this->high = high;
	this->low = low;
}

CImg<float> Canny::detect_edge(CImg<float> &srcImg) {
	CImg<float> grayImg = get_gray_image(srcImg);
	CImg<float> dx = get_gaussian_first_derX(grayImg);
	CImg<float> dy = get_gaussian_first_derY(grayImg);
	CImg<float> gradientImg = get_gaussian_gradient(dx, dy);
	CImg<float> nmsImg = non_maximun_suppression(gradientImg, dx, dy);
	CImg<float> final = link_edge(nmsImg);

	#ifdef CANNY_DEBUG
	dx.save("dx.bmp");
	dy.save("dy.bmp");
	grayImg.save("grayImg.bmp");
	nmsImg.save("nms.bmp");
	gradientImg.save("gradient.bmp");
	final.save("cannyDetect.bmp");
	#endif

	return final;
}

CImg<float> Canny::get_gaussian_first_derX(const CImg<float> &srcImg) {
	return gaussian.filter_img_by_first_derX(srcImg);
}

CImg<float> Canny::get_gaussian_first_derY(const CImg<float> &srcImg) {
	return gaussian.filter_img_by_first_derY(srcImg);
}

CImg<float> Canny::get_gaussian_gradient(const CImg<float> &dx, const CImg<float> &dy) {
	assert(dx.width() == dy.width());
	assert(dx.height() == dy.height());
	assert(dx.spectrum() == 1);
	assert(dy.spectrum() == 1);

    int width = dx.width();
    int height = dy.height();
    CImg<float> ret(width, height, 1, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
	        ret(i, j, 0, 0) = sqrt(dx(i,j,0,0)*dx(i,j,0,0)+dy(i,j,0,0)*dy(i,j,0,0));
	        if (ret(i, j, 0, 0) < 25) {
	            ret(i, j, 0, 0) = 0;
	        }
        }
    }
    return ret;
}

CImg<float> Canny::non_maximun_suppression(const CImg<float> &gradient, const CImg<float> &dx, const CImg<float> &dy) {
	assert(gradient.spectrum() == 1);

	int width = gradient.width();
	int height = gradient.height();
	CImg<float> ret(width, height, 1, 1);
    for (int i = 1; i < width-1; i++) {
        for (int j = 1; j < height-1; j++) {
            double tanTheta;
            // 90度的时候，tanTheta取一个较大值
            if (dx(i, j, 0, 0) == 0) {
                tanTheta = 2222;
            } else {
                tanTheta = (double)dy(i, j, 0, 0) / dx(i, j, 0, 0);
            }
            non_maximun_suppression(ret, gradient, i, j, tanTheta);
        }
    }
    return ret;
}

void Canny::non_maximun_suppression(CImg<float> &ret, const CImg<float> &gradient, int w, int h, double tanTheta) {
    if (tanTheta > -0.4142 && tanTheta <= 0.4142) {
        if (gradient(w, h, 0, 0) >= gradient(w-1, h, 0, 0) && gradient(w, h, 0, 0) >= gradient(w+1,h,0,0)) {
            ret(w, h, 0, 0) = gradient(w, h, 0, 0);
        } else {
            ret(w, h, 0, 0) = 0;
        }
    } else if (tanTheta > 0.4142 && tanTheta < 2.4142) {
        if (gradient(w, h, 0, 0) >= gradient(w-1, h+1, 0, 0) && gradient(w, h, 0, 0) >= gradient(w+1,h-1,0,0)) {
            ret(w, h, 0, 0) = gradient(w, h, 0, 0);
        } else {
            ret(w, h, 0, 0) = 0;
        }
    } else if (fabs(tanTheta) >= 2.4142) {
        if (gradient(w, h, 0, 0) >= gradient(w, h+1, 0, 0) && gradient(w, h, 0, 0) >= gradient(w,h-1,0,0)) {
            ret(w, h, 0, 0) = gradient(w, h, 0, 0);
        } else {
            ret(w, h, 0, 0) = 0;
        }
    } else {
        if (gradient(w, h, 0, 0) >= gradient(w-1, h-1, 0, 0) && gradient(w, h, 0, 0) >= gradient(w+1,h+1,0,0)) {
            ret(w, h, 0, 0) = gradient(w, h, 0, 0);
        } else {
            ret(w, h, 0, 0) = 0;
        }

    }
}

CImg<float> Canny::link_edge(const CImg<float> &srcImg) {
    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();
    CImg<float> ret(width, height, depth, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (srcImg(i, j, 0, 0) > high) {
                ret(i, j, 0, 0) = 255;
            } else if (srcImg(i, j, 0, 0) >= low) {
            	// 标记是否在邻居中找到超过阈值的像素
                bool hasHighNeighbor = false;

                for (int neighborX = -1; neighborX <= 1; neighborX++) {
                    for (int neighborY = -1; neighborY <= 1; neighborY++) {
                        if (hasHighNeighbor) {
                            break;
                        }
                        int tempX = i+neighborX;
                        int tempY = j+neighborY;
                        tempX = tempX < 0 ? 0 : tempX;
                        tempX = tempX >= width ? width-1 : tempX;
                        tempY = tempY < 0 ? 0 : tempY;
                        tempY = tempY >= height ? height-1 : tempY;
                        if (srcImg(tempX, tempY, 0, 0) > high) {
                            ret(i, j, 0, 0) = 255;
                            hasHighNeighbor = true;
                        }   
                    }
                }
            } else {
                //ret(i, j, 0, 0) = 0;
            }
        }
    }
    return ret;
}

CImg<float> Canny::get_gray_image(const CImg<float> &srcImg) {
	if (srcImg.spectrum() == 1) {
		return srcImg;
	}
    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();
    CImg<float> grayImg(width, height, depth, 1);
    float r, g, b, gr;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            r = srcImg(i, j, 0, 0);
            g = srcImg(i, j, 0, 1);
            b = srcImg(i, j, 0, 2);
            gr = 0.299*(r) + 0.587*(g) + 0.114*(b);
            grayImg(i, j, 0, 0) = gr;
        }
    }  
    return grayImg;
}

