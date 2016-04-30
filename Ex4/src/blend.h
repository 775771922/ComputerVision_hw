#ifndef BLEND_H
#define BLEND_H

#include "CImg.h"
#include <vector>
#include <cassert>
using namespace cimg_library;
using namespace std;

template<class T>
class Blend {
public:
	CImg<T> image_blend(CImg<T> &leftImg, CImg<T> &rightImg, int middle, bool leftToRight);
	vector<CImg<T> > get_gaussian_pyramin(const CImg<T> &img, int level);
	vector<CImg<T> > get_laplacian_pyramin(vector<CImg<T> > &g);
	vector<CImg<T> > get_laplacian_pyramin(const CImg<T> &img, int level);
	vector<CImg<T> > laplacian_combine(vector<CImg<T> > &la, 
		vector<CImg<T> > &lb, vector<CImg<T> > &gr, bool leftToRight);
};

template<class T>
CImg<T> Blend<T>::image_blend(CImg<T> &leftImg, CImg<T> &rightImg, int middle, bool leftToRight) {

	assert(leftImg.width() == rightImg.width() && leftImg.height() == rightImg.height());
	int width = leftImg.width(), height = leftImg.height(), spectrum = leftImg.spectrum();
	CImg<T> g(width, height, 1, 1);

	int level = 6;

	vector<CImg<T> > la = get_laplacian_pyramin(leftImg, level);
	vector<CImg<T> > lb = get_laplacian_pyramin(rightImg, level);
    vector<CImg<T> > ls;
	if (leftToRight) {

		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				if (col <= middle) {
					g(col, row, 0, 0) = 1;
				} else {
					g(col, row, 0, 0) = 0;
				}
			}
		}

		vector<CImg<T> > gr = get_gaussian_pyramin(g, level);

	    ls = laplacian_combine(la, lb, gr, true);

	    assert(ls[0].width() == width && ls[0].height() == height);

	    return ls[0];
	    
	} else {

		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				if (row <= middle) {
					g(col, row, 0, 0) = 1;
				} else {
					g(col, row, 0, 0) = 0;
				}
			}
		}

		vector<CImg<T> > gr = get_gaussian_pyramin(g, level);

		ls = laplacian_combine(la, lb, gr, false);
		
	    assert(ls[0].width() == width && ls[0].height() == height);

	    return ls[0];
	}
}

template<class T>
vector<CImg<T> > Blend<T>::get_gaussian_pyramin(const CImg<T> &img, int level) {
	vector<CImg<T> > ret;
	vector<CImg<T> > g(level);
	g[0] = img;
	for (int i = 1; i < level; i++) {
		g[i] = g[i-1].get_blur(2).get_resize(g[i-1].width()/2, g[i-1].height()/2, g[i-1].depth(), g[i-1].spectrum(), 3);
	}

	for (int i = 0; i < level; i++) {
		ret.push_back(g[i]);
	}
	return ret;
}

template<class T>
vector<CImg<T> > Blend<T>::get_laplacian_pyramin(vector<CImg<T> > &g) {

	int level = g.size();
	
	vector<CImg<T> > ls(level);
	ls[level-1] = g[level-1];
	for (int i = g.level-2; i >= 0; i--) {
		ls[i] = g[i] - g[i+1].get_resize(g[i].width(), g[i].height(), g[i].depth(), g[i].spectrum(), 3);
	}
	return ls;
}

template<class T>
vector<CImg<T> > Blend<T>::get_laplacian_pyramin(const CImg<T> &img, int level) {
	int width = img.width(), height = img.height(), spectrum = img.spectrum();
	vector<CImg<T> > g(level);
	g[0] = img;
	for (int i = 1; i < level; i++) {
		g[i] = g[i-1].get_blur(2).get_resize(g[i-1].width()/2, g[i-1].height()/2, g[i-1].depth(), 
			g[i-1].spectrum(), 3);
	}
	vector<CImg<T> > ls(level);
	ls[level-1] = g[level-1];
	for (int i = level-2; i >= 0; i--) {
		ls[i] = g[i] - g[i+1].get_resize(g[i].width(), g[i].height(), g[i].depth(), g[i].spectrum(), 3);
	}
	return ls;
}

template<class T>
vector<CImg<T> > Blend<T>::laplacian_combine(vector<CImg<T> > &la, 
	vector<CImg<T> > &lb, vector<CImg<T> > &gr, bool leftToRight) {
	assert(la.size() == lb.size());
	int level = la.size();
	vector<CImg<T> > ls(level);
	
	for (int i = 0; i < level; i++) {
		ls[i].assign(la[i].width(), la[i].height(), la[i].depth(), la[i].spectrum(), 0);
		for (int col = 0; col < la[i].width(); col++) {
			for (int row = 0; row < la[i].height(); row++) {
				for (int channel = 0; channel < la[i].spectrum(); channel++) {
					ls[i](col, row, 0, channel) = la[i](col, row, 0, channel)*gr[i](col, row, 0, 0) +
					       lb[i](col, row, 0, channel)*(1-gr[i](col, row, 0, 0));
				}
			}
		}
	}

	vector<CImg<T> > ret(level);
	ret[level-1] = ls[level-1];
	for (int i = level-2; i >= 0; i--) {
		ret[i] = ret[i+1].get_resize(ls[i].width(), ls[i].height(), ls[i].depth(), ls[i].spectrum(), 3) + ls[i];
		for (int row = 0; row < ret[i].height(); row++) {
			for (int col = 0; col < ret[i].width(); col++) {
				for (int channel = 0; channel < ret[i].spectrum(); channel++) {
					if (ret[i](col, row, 0, channel) > 255) {
						ret[i](col, row, 0, channel) = 255;
					} else if (ret[i](col, row, 0, channel) < 0) {
						ret[i](col, row, 0, channel) = 0;
					}	
				}			
			}
		}
	}

	return ret;

}

#endif