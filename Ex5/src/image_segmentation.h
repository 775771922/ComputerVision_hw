#ifndef IMAGE_SEGMENTATION_H
#define IMAGE_SEGMENTATION_H

#include <memory.h>

#include "CImg.h"
using namespace cimg_library;

template<class T>
class ImageSeg {
public:
	CImg<T> segment_image(const CImg<T> &img);
	vector<CImg<T> > segment_images(const vector<CImg<T> > &imgs);
};

template<class T>
CImg<T> ImageSeg<T>::segment_images(const CImg<T> &img) {
	int histogram[255], L = 256;
	float probability[255];
	memset(histogram, 0, sizeof(histogram));
	memset(probability, 0.0, sizeof(probability));
	get_histogram(img, histogram, probability, L);
	int k;
	float P1 = get_probability_of_class(k, probability, L, 1), 
	      P2 = get_probability_of_class(k, probability, L, 2);
	float m1 = get_mean_intensity_of_class(k, probability, P1, L, 1),
	      m2 = get_mean_intensity_of_class(k, probability, P2, L, 2);
	      mg = get_mean_intensity_of_global(probability, L);
	      m = get_mean_intensity_of_level(probability, k);
	      sigmaG = get_global_variance(probability, mg, L);
	      sigmaB = get_between_class_variance(P1, mg, m);
	      eta = sigmaB / sigmaG;




}

#endif