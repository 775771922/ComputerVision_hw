#include "gaussian.h"
#include <cmath>
	
Gaussian::Gaussian(float sigma, int winSize, int firstDerWinSize): PI(3.1415926) {
	this->sigma = sigma;
	this->gaussianWinSize = winSize;
	this->firstDerWinSize = firstDerWinSize;
	mask = new float*[winSize*2+1];
	for (int i = 0; i < winSize*2+1; ++i) {
		mask[i] = new float[winSize*2+1];
	}
	maskX = new float[firstDerWinSize*2+1];
	maskY = new float[firstDerWinSize*2+1];
	for (int x = -winSize; x <= winSize; x++) {
		for (int y = -winSize; y <= winSize; y++) {
			float sum = (float)(x*x+y*y) / (2*sigma*sigma);
			mask[x+winSize][y+winSize] = (float)1/(2*PI*sigma*sigma) * exp(-sum);
		}
	}
	for (int x = -firstDerWinSize; x <= firstDerWinSize; x++) {
        float r = x*x;
        float sum = -(r/(2*sigma*sigma));
        maskX[x+firstDerWinSize] = -((double)x/(sigma*sigma))*exp(sum);
	}
    for (int y = -firstDerWinSize; y <= firstDerWinSize; y++) {
        float r = y*y;
        float sum = -(r/(2*sigma*sigma));
        maskY[y+firstDerWinSize] = -((double)y/(sigma*sigma))*exp(sum);
    }
}

Gaussian::~Gaussian() {
	delete [] maskX;
	maskX = NULL;
	delete [] maskY;
	maskY = NULL;
	for (int i = 0; i < gaussianWinSize*2+1; i++) {
		delete [] mask[i];
		mask[i] = NULL;
	}
}


float* Gaussian::get_first_der_maskX() const {
	return maskX;
}

float* Gaussian::get_first_der_maskY() const {
	return maskY;
}

float** Gaussian::get_gaussian_mask() const {
	return mask;
}

CImg<float> Gaussian::filter_img_by_first_derX(const CImg<float> &srcImg) {
	int width = srcImg.width();
	int height = srcImg.height();
	int spectrum = srcImg.spectrum();
	CImg<float> ret(width, height, 1, 1, 0);
    for (int i = firstDerWinSize; i < width-firstDerWinSize; i++) {
    	for (int j = 0; j < height; j++) {
    		for (int channel = 0; channel < spectrum; channel++) {
                for (int win = -firstDerWinSize; win <= firstDerWinSize; win++) {
                    ret(i, j, 0, channel) += srcImg(i+win, j, 0, channel) * maskX[win+firstDerWinSize]; 
                }
    		}
    	}
    }
	return ret;
}

CImg<float> Gaussian::filter_img_by_first_derY(const CImg<float> &srcImg) {
	int width = srcImg.width();
	int height = srcImg.height();
	int spectrum = srcImg.spectrum();
    CImg<float> ret(width, height, 1, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = firstDerWinSize; j < height-firstDerWinSize; j++) {
            for (int channel = 0; channel < spectrum; channel++) {
                for (int win = -firstDerWinSize; win <= firstDerWinSize; win++) {
                    ret(i, j, 0, channel) += srcImg(i, j+win, 0, channel) * maskY[win+firstDerWinSize];
                }
            }
        }
    }
    return ret;
}

CImg<float> Gaussian::filter_img_by_gaussian(const CImg<float> &srcImg) {
	int width = srcImg.width();
	int height = srcImg.height();
	int spectrum = srcImg.spectrum();
    CImg<float> ret(srcImg);
    for (int i = gaussianWinSize; i < width - gaussianWinSize; i++) {
    	for (int j = gaussianWinSize; j < height - gaussianWinSize; j++) {
    		for (int channel = 0; channel < spectrum; channel++) {
    			for (int win = -gaussianWinSize; win <= gaussianWinSize; win++) {
    				ret(i, j, 0, channel) += srcImg(i+win, j+win, 0, channel) * mask[win+gaussianWinSize][win+gaussianWinSize];
    			}
    		}
    	}
    }
    return ret;
}









