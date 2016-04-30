#ifndef IMAGE_STITCH_H
#define IMAGE_STITCH_H
#include <vector>
#include <map>
#include <cmath>
#include <memory.h>
#include <float.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <list>
#include <queue>
#include <map>
#include "CImg.h"
#include "blend.h"
#include "homography.h"
#include "opencv2/opencv.hpp"
using namespace cimg_library;
using namespace std;
using namespace cv;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
	#include "vl/kdtree.h"
}

//#define DEBUG

// deminsion for sift descriptor
const int dimen = 128;

struct ImgFeature {
	vector<VlSiftKeypoint> keypoints;
	vector<vl_sift_pix*> descr;
	void clear() {
		keypoints.clear();
		for (int i = 0; i < descr.size(); i++) {
			delete [] descr[i];
		}		
	}
};

struct Pair {
	int k1;
	int k2;
	Pair(int k1, int k2) {
		this->k1 = k1;
		this->k2 = k2;
	}
};


template<class T>
class ImageStitch {
private:
	int noctaves, nlevels, o_min;
    int valueWidth(double srcX, int width);
    int valueHeight(double srcY, int height);
	CImg<T> get_gray_image(const CImg<T> &srcImg);
	void ransac(double forward[9], double backward[9], vector<Pair>& pairs, 
	    vector<VlSiftKeypoint> &keypoints1, vector<VlSiftKeypoint>& keypoints2, float epsilon);

	vector<Pair> randomly_select(vector<Pair> &pairs);
	void calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
        vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
        vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon, 
        vector<Pair> &forwardInliers);
	void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
	    vector<VlSiftKeypoint> &keypoints2, vector<Pair> &inliers, double h[9]);
	void calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<T> &img);
	vector<int> find_nearest_neighbor(int cur, const bool* isProjected, 
		vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs);
    CImg<T> get_cylindrical_proj(CImg<T> &img, ImgFeature &imgFeature);

    void scale_imgs(vector<CImg<T> > &imgs);

    int calc_iterations(double p, double P, int n);
    CImg<T> image_stitch_and_blend(CImg<T> &res, int cur, int neighbor,
	    vector<ImgFeature> &imgFeatures, const vector<CImg<T> > &imgs, double forward_h[],
	    double backward_h[], bool* isProjected);
    CImg<T> image_stitch(CImg<T> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures,
	    map<int, vector<Pair> > &pointPairs, const vector<CImg<T> > &imgs, bool *isProjected);
    
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<T> image_stitch(const vector<CImg<T> > &imgs);
};


template<class T>
ImageStitch<T>::ImageStitch(int octaves, int levels, int o_min)
           : noctaves(octaves), nlevels(levels), o_min(o_min) {
}

template<class T>
CImg<T> ImageStitch<T>::image_stitch(const vector<CImg<T> > &inputImgs) {
	srand((unsigned)time(0));

	vector<CImg<T> > imgs(inputImgs);

	scale_imgs(imgs);

	vector<ImgFeature> imgFeatures;
	bool* isProjected = new bool[imgs.size()];
	memset(isProjected, 0, sizeof(bool)*imgs.size());
	for (int i = 0; i < imgs.size(); i++) {
		calc_img_feature(imgFeatures, imgs[i]);
	}

	for (int i = 0; i < imgs.size(); i++) {
		imgs[i] = get_cylindrical_proj(imgs[i], imgFeatures[i]);
	}

	queue<int> q;
	int randomIndex = rand() % imgs.size();
	q.push(randomIndex);
	isProjected[randomIndex] = true;

    string name = "A.jpg";

    CImg<T> res(imgs[randomIndex]);

	// 要求所有图片都能找到匹配的
	while (!q.empty()) {
		int cur = q.front();
		q.pop();
		map<int, vector<Pair> > pointPairs;

		vector<int> neighbors = find_nearest_neighbor(cur, isProjected, imgFeatures, pointPairs);

		for (int i = 0; i < neighbors.size(); i++) {
			q.push(neighbors[i]);
			res = image_stitch(res, cur, neighbors[i], imgFeatures, pointPairs, imgs, isProjected);
			isProjected[neighbors[i]] = true;
			
            #ifdef DEBUG
            res.display();
            cout << "finish stitch" << endl;
            res.normalize(0, 255).save_jpeg(name.c_str());
            name[0]++;
            #endif

		}
		
	}

	res.normalize(0, 255).save_jpeg("final1.jpg");

	delete [] isProjected;
	isProjected = NULL;
	return res;
}

template<class T>
void ImageStitch<T>::scale_imgs(vector<CImg<T> > &imgs) {
	double scale;
	for (int i = 0; i < imgs.size(); i++) {
		if (imgs[i].width() > 500 || imgs[i].height() > 500) {
			if (imgs[i].width() > imgs[i].height()) {
				scale = 500.0 / imgs[i].width();
				imgs[i].resize((imgs[i].width()*scale), (int)(imgs[i].height()*scale), 1, 3, 3);
			} else {
				scale = (float)500 / imgs[i].height();
				imgs[i].resize((int)(imgs[i].width()*scale), (int)(imgs[i].height()*scale), 1, 3, 3);
			}

		}
	}
}

template<class T>
CImg<T> ImageStitch<T>::get_cylindrical_proj(CImg<T> &img, ImgFeature &imgFeature) {
	
	double alpha = 1.0 * 180 / 360;
	int width = img.width(), height = img.height(), spectrum = img.spectrum();
	double R = width / (2*tan(alpha/2));

    for (int i = 0; i < imgFeature.keypoints.size(); i++) {
    	double x = imgFeature.keypoints[i].x, y = imgFeature.keypoints[i].y;
    	double k = R / sqrt(R*R+(x-width/2)*(x-width/2));
    	imgFeature.keypoints[i].x = (x-width/2)*k + width/2;
    	imgFeature.keypoints[i].y = (y-height/2)*k + height/2;
    }

    Point2f lt((0-width/2)*(R/sqrt(R*R+(0-width/2)*(0-width/2))) + width/2,
               (0-height/2)*(R/sqrt(R*R+(0-width/2)*(0-width/2))) + height/2),
            rb((width-1-width/2)*(R/sqrt(R*R+(width-1-width/2)*(width-1-width/2))) + width/2, 
               (height-1-height/2)*(R/sqrt(R*R+(width-1-width/2)*(width-1-width/2))) + height/2);
    width = abs(rb.x-lt.x);
    height = abs(rb.y-lt.y);

    CImg<T> ret(width, height, 1, spectrum, 0);
    for (int i = 0; i < width; i++) {
    	for (int j = 0; j < height; j++) {
    		double k = R / sqrt(R*R+(i-width/2)*(i-width/2));
    		double x = (i-width/2)/k + width/2;
    		double y = (j-height/2)/k + height/2;

			if (x >= 0 && x < width && y >= 0 && y < height) {
				double u = x - (int)x, v = y - (int)y;
				for (int channel = 0; channel < spectrum; channel++) {
					ret(i, j, 0, channel) = 
						(T)((1-u)*(1-v)*img(valueWidth(x, img.width()), valueHeight(y, img.height()), 0, channel)
						    +(1-u)*v*img(valueWidth(x, img.width()), valueHeight(y+1, img.height()), 0, channel)
						    +u*(1-v)*img(valueWidth(x+1, img.width()), valueHeight(y, img.height()), 0, channel)
						    +u*v*img(valueWidth(x+1, img.width()), valueHeight(y+1, img.height()), 0, channel));
				}
			}    		
    	}
    }
    return ret;
}

template<class T>
void ImageStitch<T>::calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<T> &img) {
	CImg<T> tempImg(img);

	if (tempImg.spectrum() != 1) {
		tempImg = get_gray_image(img);
	}

	ImgFeature feature;
	VlSiftFilt* siftFilt = vl_sift_new(tempImg.width(), tempImg.height(), noctaves, nlevels, o_min);
	vl_sift_pix* imageData = new vl_sift_pix[tempImg.width()*tempImg.height()];
	for (int i = 0; i < tempImg.height(); i++) {
		for (int j = 0; j < tempImg.width(); j++) {
			imageData[i*tempImg.width()+j] = tempImg(j, i, 0);
		}
	}
	if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
		while (true) {
			vl_sift_detect(siftFilt);
			VlSiftKeypoint *keyPoint = siftFilt->keys;
			for (int i = 0; i < siftFilt->nkeys; i++) {
				VlSiftKeypoint tempKeyPoint = *keyPoint;

				VlSiftKeypoint t = tempKeyPoint;

				feature.keypoints.push_back(t);
				keyPoint++;
				double angles[4];
				vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);
				// 默认只取第一个角度的描述符
				vl_sift_pix* descriptors = new vl_sift_pix[dimen];
				vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, angles[0]);
				feature.descr.push_back(descriptors);
			}
			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}
	imgsFeature.push_back(feature);
	vl_sift_delete(siftFilt);
	delete [] imageData;
	imageData = NULL;
	siftFilt = NULL;
}

/**
* find all the images that match to cur image
**/
template<class T>
vector<int> ImageStitch<T>::find_nearest_neighbor(int cur, const bool* isProjected, 
	vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs) {

	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, dimen, 1, VlDistanceL1);
    // 为当前image构建kd树
    vector<vl_sift_pix*> descr = imgFeatures[cur].descr;
	float *data = new float[dimen*descr.size()];
	int k = 0;
	for (auto it = descr.begin(); it != descr.end(); it++) {
		vl_sift_pix* originData = *it;
		for (int index = 0; index < dimen; index++) {
			data[index+k*dimen] = originData[index];
		}
		k++;
	}
	vl_kdforest_build(forest, descr.size(), data);

	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbors[2];

	vector<int> indexs;
	for (int i = 0; i < imgFeatures.size(); i++) {

		if (i != cur && isProjected[i] != true) {
			vector<Pair> pairs;

            // 遍历待搜索的图片特征，找到最匹配的点对
			descr = imgFeatures[i].descr;
			for (int j = 0; j < descr.size(); j++) {
				int nvisited = vl_kdforestsearcher_query(searcher, neighbors, 2, descr[j]);
				if (neighbors[0].distance < neighbors[1].distance * 0.5) {
					// neighbors[0].index表示kd树中最佳匹配点的位置
					pairs.push_back(Pair(neighbors[0].index, j));
				}
			}	

            // 匹配点对超过20个才认为两张图有公共部分
			if (pairs.size() > 20) {
				pointPairs.insert(make_pair(i, pairs));
				indexs.push_back(i);
			}	
		}
	}

    vl_kdforestsearcher_delete(searcher);
	vl_kdforest_delete(forest);
	delete [] data;
	forest = NULL;
	searcher = NULL;
	data = NULL;
	return indexs;
}

template<class T>
CImg<T> ImageStitch<T>::image_stitch(CImg<T> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures,
	map<int, vector<Pair> > &pointPairs, const vector<CImg<T> > &imgs, bool *isProjected) {
	double forward_h[9], backward_h[9];
	float epsilon = 2.0;
	map<int, vector<Pair> >::iterator it = pointPairs.find(neighbor);

	ImgFeature lf = imgFeatures[cur];
	ImgFeature rf = imgFeatures[neighbor];

	ransac(forward_h, backward_h, it->second, lf.keypoints, rf.keypoints, epsilon);

	return image_stitch_and_blend(res, cur, neighbor, imgFeatures, imgs, forward_h, backward_h, isProjected);
}

template<class T>
CImg<T> ImageStitch<T>::image_stitch_and_blend(CImg<T> &res, int cur, int neighbor,
	vector<ImgFeature> &imgFeatures, const vector<CImg<T> > &imgs, double forward_h[],
	double backward_h[], bool* isProjected) {

	CImg<T> neighborImg(imgs[neighbor]);

    assert(res.spectrum() == neighborImg.spectrum());

	int width = neighborImg.width()-1, height = neighborImg.height()-1;

	Point2f lt(Homography::calc_X(0, 0, backward_h), Homography::calc_Y(0, 0, backward_h)),
            lb(Homography::calc_X(0, height, backward_h), Homography::calc_Y(0, height, backward_h)),
            rt(Homography::calc_X(width, 0, backward_h), Homography::calc_Y(width, 0, backward_h)),
            rb(Homography::calc_X(width, height, backward_h), 
               Homography::calc_Y(width, height, backward_h));

    int maxX = max(lt.x, max(lb.x, max(rt.x, rb.x)));
    int minX = min(lt.x, min(lb.x, min(rt.x, rb.x)));
    int maxY = max(lt.y, max(lb.y, max(rt.y, rb.y)));
    int minY = min(lt.y, min(lb.y, min(rt.y, rb.y)));

    int offsetX = 0, offsetY = 0;
    width = res.width();
    height = res.height();
    // neighborImg projects to the left of res
    if (minX <= 0) {
    	offsetX = 0 - minX;
    	width = res.width() - minX;
    }  
    if (maxX >= res.width()) {  // neighborImg projects to the right of res
    	width = maxX + offsetX;
    }

    // neighborImg projects to the top of res
    if (minY <= 0) {
    	offsetY = 0 - minY;
    	height = res.height() - minY;
    } 
    if (maxY >= res.height()) {  // neighborImg projects to the bottom of res
    	height = maxY + offsetY;
    }

    CImg<T> img1(width, height, 1, res.spectrum(), 0);
    CImg<T> img2(width, height, 1, res.spectrum(), 0);

    // the overlap area of two images
    minX = 100000;
    maxX = 0;
    minY = 100000;
    maxY = 0;

    for (int i = 0; i < img1.width(); i++) {
    	for (int j = 0; j < img1.height(); j++) {    		
			int projectI = i - offsetX, projectJ = j - offsetY;
			if (projectI >= 0 && projectI < res.width() && projectJ >= 0 && projectJ < res.height()) {
    			for (int channel = 0; channel < res.spectrum(); channel++) {
    				img1(i, j, 0, channel) = res(projectI, projectJ, 0, channel);
    			}
    		}
    	}
    }

    // update res's keypoint, note that all images that has projected onto the res image should update
    for (int i = 0; i < imgFeatures.size(); i++) {
    	if (isProjected[i] == true) {
    		for (int j = 0; j < imgFeatures[i].keypoints.size(); j++) {
    		    imgFeatures[i].keypoints[j].x += offsetX;
    		    imgFeatures[i].keypoints[j].y += offsetY;
            }
    	}
    }

    for (int i = 0; i < img2.width(); i++) {
    	for (int j = 0; j < img2.height(); j++) {
    		int ii = i - offsetX, jj = j - offsetY;
    		double x = Homography::calc_X(ii, jj, forward_h), y = Homography::calc_Y(ii, jj, forward_h);
			if (x >= 0 && x < neighborImg.width() && y >= 0 && y < neighborImg.height()) {
				double u = x - (int)x, v = y - (int)y;
    			for (int channel = 0; channel < res.spectrum(); channel++) {
                    img2(i, j, 0, channel) = 
                        (T)((1-u)*(1-v)*neighborImg(valueWidth(x, neighborImg.width()), valueHeight(y, neighborImg.height()), 0, channel)
                        +(1-u)*v*neighborImg(valueWidth(x, neighborImg.width()), valueHeight(y+1, neighborImg.height()), 0, channel)
                        +u*(1-v)*neighborImg(valueWidth(x+1, neighborImg.width()), valueHeight(y, neighborImg.height()), 0, channel)
                        +u*v*neighborImg(valueWidth(x+1, neighborImg.width()), valueHeight(y+1, neighborImg.height()), 0, channel));    				
    			}
    			if (img1(i, j, 0, 0) > 0) {
    				minX = i < minX ? i : minX, maxX = i > maxX ? i : maxX;
    				minY = j < minY ? j : minY, maxY = j > maxY ? j : maxY;
    			}
			}
    	}
    }

    // update neighbor image's keypoint.
    for (int i = 0; i < imgFeatures[neighbor].keypoints.size(); i++) {
    	double x = imgFeatures[neighbor].keypoints[i].x;
    	double y = imgFeatures[neighbor].keypoints[i].y;
    	imgFeatures[neighbor].keypoints[i].x = Homography::calc_X(x, y, backward_h, offsetX);
    	imgFeatures[neighbor].keypoints[i].y = Homography::calc_Y(x, y, backward_h, offsetY);
    }

    Blend<T> blend;

    if (img1.width() > img1.height()) {  // stitch images from left to right
     	if (offsetX == 0) {
     		return blend.image_blend(img1, img2, (minX+maxX)/2, true);
     	} else {
     		return blend.image_blend(img2, img1, (minX+maxX)/2, true);
     	}
     } else if (img1.height() > img1.width()) {   // stitch images from top to bottom
     	if (offsetY == 0) {
     		return blend.image_blend(img1, img2, (minY+maxY)/2, false);
     	} else {
     		return blend.image_blend(img2, img1, (minY+maxY)/2, false);
     	}
     } else {
     	assert(false);
     }

}

template<class T>
void ImageStitch<T>::ransac(double forward[9], double backward[9], vector<Pair>& pairs, 
	vector<VlSiftKeypoint> &keypoints1, vector<VlSiftKeypoint>& keypoints2, float epsilon) {
	int loop_times = calc_iterations(0.3, 0.99, 4);

	int max_inliers = 0;
	double tempFH[9], tempBH[9];
	// used to recompute least squares H
	vector<Pair> forwardInliers;
	vector<Pair> backwardInliers;

	while (loop_times--) {
		vector<Pair> randomPairs = randomly_select(pairs);

		calc_homography(randomPairs, keypoints1, keypoints2, tempFH);

		vector<Pair> tempPairs;
		for (int i = 0; i < randomPairs.size(); i++) {
			tempPairs.push_back(Pair(randomPairs[i].k2, randomPairs[i].k1));
		}
		calc_homography(tempPairs, keypoints2, keypoints1, tempBH);

		vector<Pair> tempForwardInliers, tempBackwardInliers;

		int inliers = calc_inliers(pairs, keypoints1, keypoints2, tempFH, epsilon, tempForwardInliers);

		if (inliers > max_inliers) {

			for (int i = 0; i < 9; i++) {
				forward[i] = tempFH[i];
			}
			
			for (int i = 0; i < 9; i++) {
				backward[i] = tempBH[i];
			}

			max_inliers = inliers;

            forwardInliers = tempForwardInliers;
            backwardInliers.clear();
			for (int i = 0; i < tempForwardInliers.size(); i++) {
				Pair pair = tempForwardInliers[i];
				backwardInliers.push_back(Pair(pair.k2, pair.k1));
			}
		}
	}
}

template<class T>
int ImageStitch<T>::calc_iterations(double p, double P, int n) {
	return log(1-P)/log(1-pow(p, n));
}

template<class T>
vector<Pair> ImageStitch<T>::randomly_select(vector<Pair> &pairs) {
	vector<Pair> ret;
	bool flag[pairs.size()];
	memset(flag, 0, sizeof(flag));

	for (int i = 0; i < 4; i++) {
		while (true) {
			int randomIndex = rand() % pairs.size();
			if (flag[randomIndex] == false) {
				flag[randomIndex] = true;
				ret.push_back(pairs[randomIndex]);
				break;
			}
		}
	}
	return ret;
}

template<class T>
void ImageStitch<T>::calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
    vector<VlSiftKeypoint> &keypoints2, double h[9]) {
	assert(randomPairs.size() == 4);

	vector<Point2f> srcV, destV;
	for (int i = 0; i < randomPairs.size(); i++) {
		srcV.push_back(Point2f(keypoints1[randomPairs[i].k1].x, keypoints1[randomPairs[i].k1].y));
		destV.push_back(Point2f(keypoints2[randomPairs[i].k2].x, keypoints2[randomPairs[i].k2].y));
	}
	Homography::calc_homography(srcV, destV, h);
}

template<class T>
int ImageStitch<T>::calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon, vector<Pair> &forwardInliers) {
	int inliers = 0;
	for (int i = 0; i < pairs.size(); i++) {
		VlSiftKeypoint srcP = keypoints1[pairs[i].k1];
		VlSiftKeypoint destP = keypoints2[pairs[i].k2];
		double x, y;
		x = Homography::calc_X(srcP.x, srcP.y, tempH);
		y = Homography::calc_Y(srcP.x, srcP.y, tempH);
		if ((x-destP.x)*(x-destP.x)+(y-destP.y)*(y-destP.y) < epsilon*epsilon) {
			++inliers;
			forwardInliers.push_back(pairs[i]);
		}
	}
	return inliers;
}
    
template<class T>
void ImageStitch<T>::recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, vector<Pair> &inliers, double h[9]) {
	int N = inliers.size();
	double *A = new double[2*N*8];
	double *b = new double[2*N];
	memset(A, 0, sizeof(int)*(2*N*8));
	h[8] = 1;
	for (int row = 0; row < N; row++) {
		Pair p = inliers[row];
		A[row*16] = keypoints1[p.k1].x;
		A[row*16+1] = keypoints1[p.k1].y;
		A[row*16+2] = 1;
		A[row*16+6] = -1*keypoints1[p.k1].x*keypoints2[p.k2].x;
		A[row*16+7] = -1*keypoints1[p.k1].y*keypoints2[p.k2].x;
		A[row*16+11] = keypoints1[p.k1].x;
		A[row*16+12] = keypoints1[p.k1].y;
		A[row*16+13] = 1;
		A[row*16+14] = -1*keypoints1[p.k1].x*keypoints2[p.k2].y;
		A[row*16+15] = -1*keypoints1[p.k1].y*keypoints2[p.k2].y;
		b[row*2] = keypoints2[p.k2].x;
		b[row*2+1] = keypoints2[p.k2].y;
	}

	Homography::least_squares(A, b, h, N);

	delete [] A;
	delete [] b;
	A = NULL;
	b = NULL;
}

// 检查像素位置，防止超过图片宽度
template<class T>
int ImageStitch<T>::valueWidth(double srcX, int width) {
    if (srcX < 0) srcX = 0;
    if (srcX >= width) srcX--;
    return srcX;
}

// 检查像素位置，防止超过图片高度
template<class T>
int ImageStitch<T>::valueHeight(double srcY, int height) {
    if (srcY < 0) srcY = 0;
    if (srcY >= height) srcY--;
    return srcY;
}

template<class T>
CImg<T> ImageStitch<T>::get_gray_image(const CImg<T> &srcImg) {
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