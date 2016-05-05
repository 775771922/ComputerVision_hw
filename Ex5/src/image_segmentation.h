#ifndef IMAGE_SEGMENTATION_H
#define IMAGE_SEGMENTATION_H

#include <memory.h>
#include <cassert>
#include <iostream>
#include <vector>
#include <string>
#include <float.h>
using namespace std;

#include "CImg.h"
using namespace cimg_library;

#define DEBUG

template<class T>
class ImageSeg {
public:
	CImg<T> segment_image(const CImg<T> &img);
	vector<CImg<T> > segment_images(const vector<CImg<T> > &imgs);
private:
	void get_histogram(const CImg<T> &img, int histogram[], float probability[], int L);
	float get_probability_of_class(int k, float probability[], int L, int type);
	float get_mean_intensity_of_class(int k, float probability[], float P, int L, int type);
	float get_mean_intensity_of_global(float probability[], int L);
	float get_mean_intensity_of_level(float probability[], int k);
	float get_global_variance(float probability[], float mg, int L);
	float get_between_class_variance(float P, float mg, float m);
	CImg<T> get_gray_image(const CImg<T> &srcImg);
};

template<class T>
vector<CImg<T> > ImageSeg<T>::segment_images(const vector<CImg<T> > &imgs) {
	vector<CImg<T> > ret;
	for (int i = 0; i < imgs.size(); i++) {
		ret.push_back(segment_image(imgs[i]));
	}
	return ret;
}

template<class T>
CImg<T> ImageSeg<T>::segment_image(const CImg<T> &img) {

	CImg<T> temp;

    if (img.spectrum() > 1) {
    	temp = get_gray_image(img);
    } else {
    	temp.assign(img);
    }

    int width = temp.width(), height = temp.height();

	int histogram[255], L = 256;
	float probability[255];
	memset(histogram, 0, sizeof(histogram));
	memset(probability, 0.0, sizeof(probability));
	get_histogram(temp, histogram, probability, L);
	int kk = 1;
	float max = FLT_MIN;
	for (int k = 1; k < L-1; k++) {
		float P1 = get_probability_of_class(k, probability, L, 1), 
	          P2 = get_probability_of_class(k, probability, L, 2);
	    float m1 = get_mean_intensity_of_class(k, probability, P1, L, 1),
	          m2 = get_mean_intensity_of_class(k, probability, P2, L, 2),
	       	  mg = get_mean_intensity_of_global(probability, L),
	      	  m = get_mean_intensity_of_level(probability, k),
	          sigmaG = get_global_variance(probability, mg, L),
	          sigmaB = get_between_class_variance(P1, mg, m);
	          //eta = sigmaB / sigmaG;
	    if (sigmaB > max) {
	    	max = sigmaB;
	    	kk = k;
	    }
	}

	cout << "max=" << max << "   " << "k=" << kk << endl;

    for (int i = 0; i < width; i++) {
    	for (int j = 0; j < height; j++) {
    		if (temp(i, j, 0, 0) >= kk) {
    			temp(i, j, 0, 0) = 255;
    		} else {
    			temp(i, j, 0, 0) = 0;
    		}
    	}
    }

    #ifdef DEBUG
    cout << "after test=======>" << endl;
    for (int i = 0; i < width; i++) {
    	for (int j = 0; j < height; j++) {
    		assert(temp(i, j, 0, 0) == 255 || temp(i, j, 0, 0) == 0);
    		if (temp(i, j, 0, 0) != 255 && temp(i, j, 0, 0) != 0) 
    			cout << temp(i, j, 0, 0) << endl;
    	}
    }
    #endif

    return temp;

}

template<class T>
void ImageSeg<T>::get_histogram(const CImg<T> &img, int histogram[], float probability[], int L) {
	assert(img.spectrum() == 1);
	int M = img.width(), N = img.height(), sum = 0;
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			int l = (int)img(i, j, 0, 0);
			assert(l >= 0 && l < L);
			histogram[l]++;
		}
	}

	for (int i = 0; i < L; i++) {
		sum += histogram[i];
	}

	for (int i = 0; i < L; i++) {
		probability[i] = (float)histogram[i] / sum;
	}

}

template<class T>
float ImageSeg<T>::get_probability_of_class(int k, float probability[], int L, int type) {
	assert(type == 1 || type == 2);
	assert(k < L && k >= 0);

	float P = 0;
	switch (type) {
		case 1:
		for (int i = 0; i <= k; i++) {
			P += probability[i];
		}
		break;
		case 2:
		for (int i = k+1; i < L; i++) {
			P += probability[i];
		}
		break;
		default:
		assert(false);    // this should not happen
		break;
	}

	return P;
}

template<class T>
float ImageSeg<T>::get_mean_intensity_of_class(int k, float probability[], float P, int L, int type) {
	assert(type == 1 || type == 2);
	assert(k < L && k >= 0);

	float m = 0;
	switch (type) {
		case 1:
		for (int i = 0; i <= k; i++) {
			m += probability[i] * i;
		}
		m /= P;
		break;
		case 2:
		for (int i = k+1; i < L; i++) {
			m += probability[i] * i;
		}
		m /= P;
		break;
		default:
		assert(false);   // this should not happen
		break;
	}

	return m;
}

template<class T>
float ImageSeg<T>::get_mean_intensity_of_global(float probability[], int L) {
	float mg = 0;
	for (int i = 0; i < L; i++) {
		mg += probability[i] * i;
	}
	return mg;
}

template<class T>
float ImageSeg<T>::get_mean_intensity_of_level(float probability[], int k) {
	
	float m = 0;

	for (int i = 0; i <= k; i++) {
		m += probability[i] * i;
	}

	return m;

}

template<class T>
float ImageSeg<T>::get_global_variance(float probability[], float mg, int L) {
	float sigmaG = 0;
	for (int i = 0; i < L; i++) {
		sigmaG += ((i-mg)*(i-mg))*probability[i];
	}
	return sigmaG;
}

template<class T>
float ImageSeg<T>::get_between_class_variance(float P, float mg, float m) {
	float denominator = P*(1-P);
	float numerator = (mg*P-m)*(mg*P-m);
	return numerator / denominator;
}

template<class T>
CImg<T> ImageSeg<T>::get_gray_image(const CImg<T> &srcImg) {
	if (srcImg.spectrum() == 1) {
		return srcImg;
	}
    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();
    CImg<T> grayImg(width, height, depth, 1);
    T r, g, b, gr;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            r = srcImg(i, j, 0, 0);
            g = srcImg(i, j, 0, 1);
            b = srcImg(i, j, 0, 2);
            gr = 0.299*(r) + 0.587*(g) + 0.114*(b);
            grayImg(i, j, 0, 0) = (T)gr;
        }
    }
    return grayImg;
}


#endif