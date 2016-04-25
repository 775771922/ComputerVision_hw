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
#include "opencv2/opencv.hpp"
using namespace cimg_library;
using namespace std;
using namespace cv;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
	#include "vl/kdtree.h"
}

//#define Image_Stitch_DEBUG
#define DEBUG

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

enum Direct {Top, Bottom, Left, Right};

template<class T>
class ImageStitch {
private:
	int noctaves, nlevels, o_min;
    int valueWidth(double srcX, int width);
    int valueHeight(double srcY, int height);
	CImg<T> get_gray_image(const CImg<T> &srcImg);
	void ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
		vector<VlSiftKeypoint>& keypoints2, float epsilon);
	vector<Pair> randomly_select(vector<Pair> &pairs);
	void calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
        vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon);
    void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double h[9]);
	void calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<T> &img);
    void image_stitch(CImg<T> &res, int cur, int target, vector<int> &neighbors,
	    vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs, 
	    const vector<CImg<T> > &imgs, bool* isUsed);
	CImg<T> image_stitch(CImg<T> &res, int target, vector<int> &neighbors,
	    const vector<CImg<T> > &imgs, double h[], vector<ImgFeature> &imgFeatures, bool *isUsed);
	vector<int> find_nearest_neighbor(int cur, const bool* isProjected, 
		vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs);
    vector<CImg<T> > get_laplacian_pyramin(const CImg<T> &img);
    CImg<T> laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, 
	    double h[], int width, int height, int spectrum, int offsetX, int offsetY);
    CImg<T> get_cylindrical_proj(const CImg<T> &img);

    int calc_iterations(double p, double P, int n);
    CImg<T> image_blend(CImg<T> &img1, CImg<T> &img2);
    CImg<T> image_stitch_and_blend(CImg<T> &res, int cur, int neighbor,
	    vector<ImgFeature> &imgFeatures, const vector<CImg<T> > &imgs, double forward_h[],
	    double backward_h[], bool* isProjected);
    CImg<T> image_stitch(CImg<T> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures,
	    map<int, vector<Pair> > &pointPairs, const vector<CImg<T> > &imgs, bool *isProjected);
    vector<CImg<T> > laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, bool leftToRight);
    CImg<T> image_blend(CImg<T> &leftImg, CImg<T> &rightImg, bool leftToRight);
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<T> image_stitch(const vector<CImg<T> > &imgs);


};


template<class T>
ImageStitch<T>::ImageStitch(int octaves, int levels, int o_min)
           : noctaves(octaves), nlevels(levels), o_min(o_min) {
}

template<class T>
CImg<T> ImageStitch<T>::image_stitch(const vector<CImg<T> > &imgs) {
	srand((unsigned)time(0));

	// vector<CImg<T> > imgs(inputImgs.size());

    // for (int i = 0; i < imgs.size(); i++) {
    // 	imgs[i] = get_cylindrical_proj(inputImgs[i]);
    // 	imgs[i].display();
    // 	char name[3];
    // 	sprintf(name, "%d", i);
    // 	string s(string(name)+".jpg");
    // 	imgs[i].save_jpeg(s.c_str());
    // }

	vector<ImgFeature> imgFeatures;
	bool* isProjected = new bool[imgs.size()];
	memset(isProjected, 0, sizeof(bool)*imgs.size());
	for (int i = 0; i < imgs.size(); i++) {
		calc_img_feature(imgFeatures, imgs[i]);
	}

	#ifdef DEBUG
	cout << "feature size====>" << endl;
	for (int i = 0; i < imgFeatures.size(); i++) {
		cout << imgFeatures[i].keypoints.size() << endl;
	}
	#endif

	queue<int> q;
	int randomIndex = rand() % imgs.size();
	q.push(randomIndex);
	isProjected[randomIndex] = true;

	//CImg<T> res(imgs[randomIndex]);


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
            cout << "neighbor=====>" << neighbors[i] << endl;
            res.display();
            res.save_jpeg(name.c_str());
            name[0]++;
            #endif

		}
		
	}

	delete [] isProjected;
	isProjected = NULL;
	return res;
}

template<class T>
CImg<T> ImageStitch<T>::get_cylindrical_proj(const CImg<T> &img) {
	int R = 1000;
	int width = img.width(), height = img.height(), spectrum = img.spectrum();
	CImg<T> ret(width, height, 1, spectrum, 0);
	for (int i = 0; i < img.width(); i++) {
		for (int j = 0; j < img.height(); j++) {
			double k = sqrt(R*R+(i-width/2)*(i-width/2)) / R;
			double x = (i-width/2)*k + width/2;
			double y = (j-height/2)*k + height/2;
			if (x >= 0 && x < width && y >= 0 && y < height) {
				double u = x - (int)x, v = y - (int)y;
				for (int channel = 0; channel < spectrum; channel++) {
					ret(i, j, 0, channel) = 
						(int)((1-u)*(1-v)*img(valueWidth(x, img.width()), valueHeight(y, img.height()), 0, channel)
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
				feature.keypoints.push_back(tempKeyPoint);
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

		// if (indexs.size() >= 2) {
		// 	break;
		// }

		if (i != cur && isProjected[i] != true) {
			vector<Pair> pairs;

            // 遍历待搜索的图片特征，找到最匹配的点对
			descr = imgFeatures[i].descr;
			for (int j = 0; j < descr.size(); j++) {
				int nvisited = vl_kdforestsearcher_query(searcher, neighbors, 2, descr[j]);
				if (neighbors[0].distance < neighbors[1].distance * 0.4) {
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
	float epsilon = 4.0;
	map<int, vector<Pair> >::iterator it = pointPairs.find(neighbor);

	ImgFeature lf = imgFeatures[cur];
	ImgFeature rf = imgFeatures[neighbor];
	ransac(forward_h, it->second, lf.keypoints, rf.keypoints, epsilon);

	// vector<Pair> tempPairs;
	// vector<Pair> origin = it->second;
	// for (int i = 0; i < origin.size(); i++) {
	// 	tempPairs.push_back(Pair(origin[i].k2, origin[i].k1));
	// }
	// ransac(backward_h, tempPairs, rf.keypoints, lf.keypoints, epsilon);
	Mat m = Mat(3, 3, CV_64FC1, forward_h);
    Mat inv = m.inv();
    for (int i = 0; i < 3; i++) {
    	for (int j = 0; j < 3; j++) {
    		backward_h[i*3+j] = inv.at<double>(i,j);
    	}
    }
	return image_stitch_and_blend(res, cur, neighbor, imgFeatures, imgs, forward_h, backward_h, isProjected);
}


template<class T>
CImg<T> ImageStitch<T>::image_stitch_and_blend(CImg<T> &res, int cur, int neighbor,
	vector<ImgFeature> &imgFeatures, const vector<CImg<T> > &imgs, double forward_h[],
	double backward_h[], bool* isProjected) {

	CImg<T> neighborImg(imgs[neighbor]);

    assert(res.spectrum() == neighborImg.spectrum());

    forward_h[8] = 1;
    backward_h[8] = 1;

	int width = neighborImg.width()-1, height = neighborImg.height()-1;
	Point2f lt(backward_h[2], backward_h[5]),
            lb(backward_h[1]*height+backward_h[2], backward_h[4]*height+backward_h[5]),
            rt(backward_h[0]*width+backward_h[1]*0+backward_h[2], 
            	backward_h[3]*width+backward_h[4]*0+backward_h[5]),
            rb(backward_h[0]*width+backward_h[1]*height+backward_h[2], 
            	backward_h[3]*width+backward_h[4]*height+backward_h[5]);

	// Point2f lt(backward_h[2]/backward_h[8], backward_h[5]/backward_h[8]),
 //            lb((backward_h[1]*height+backward_h[2])/(backward_h[7]*height+backward_h[8]),
 //               (backward_h[4]*height+backward_h[5])/(backward_h[7]*height+backward_h[8])),
 //            rt((backward_h[0]*width+backward_h[1]*0+backward_h[2])/(backward_h[6]*width+backward_h[8]), 
 //               (backward_h[3]*width+backward_h[4]*0+backward_h[5])/(backward_h[6]*width+backward_h[8])),
 //            rb((backward_h[0]*width+backward_h[1]*height+backward_h[2])/(backward_h[6]*width+backward_h[7]*height+backward_h[8]), 
 //               (backward_h[3]*width+backward_h[4]*height+backward_h[5])/(backward_h[6]*width+backward_h[7]*height+backward_h[8]));


    int maxX = max(lt.x, max(lb.x, max(rt.x, rb.x)));
    int minX = min(lt.x, min(lb.x, min(rt.x, rb.x)));
    int maxY = max(lt.y, max(lb.y, max(rt.y, rb.y)));
    int minY = min(lt.y, min(lb.y, min(rt.y, rb.y)));

    int offsetX, offsetY;
    width = res.width();
    height = res.height();
    // neighborImg projects to the left of res
    if (minX <= 0) {
    	offsetX = 0 - minX;
    	width = res.width() - minX;
    } else if (maxX >= res.width()) {  // neighborImg projects to the right of res
    	offsetX = 0;
    	width = maxX;
    } else {
    	
    }

    // neighborImg projects to the top of res
    if (minY <= 0) {
    	offsetY = 0 - minY;
    	height = res.height() - minY;
    } else if (maxY >= res.height()) {  // neighborImg projects to the bottom of res
    	offsetY = 0;
    	height = maxY;
    } else {
    	
    }

    #ifdef DEBUG
    cout << "offsetX====>" << offsetX << endl;
    cout << "offsetY====>" << offsetY << endl;
    #endif

    CImg<T> img1(width, height, 1, res.spectrum(), 0);
    CImg<T> img2(width, height, 1, res.spectrum(), 0);

    CImg<T> subA(width, height, 1, res.spectrum(), 0);
    CImg<T> subB(width, height, 1, res.spectrum(), 0);

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
    		double x = ii*forward_h[0]+jj*forward_h[1]+forward_h[2], y = ii*forward_h[3]+jj*forward_h[4]+forward_h[5];
    		// double x = (ii*forward_h[0]+jj*forward_h[1]+forward_h[2])/(ii*forward_h[6]+jj*forward_h[7]+forward_h[8]), 
    		//        y = (ii*forward_h[3]+jj*forward_h[4]+forward_h[5])/(ii*forward_h[6]+jj*forward_h[7]+forward_h[8]);
			if (x >= 0 && x < neighborImg.width() && y >= 0 && y < neighborImg.height()) {
				double u = x - (int)x, v = y - (int)y;
    			for (int channel = 0; channel < res.spectrum(); channel++) {
                    img2(i, j, 0, channel) = 
                        (int)((1-u)*(1-v)*neighborImg(valueWidth(x, neighborImg.width()), valueHeight(y, neighborImg.height()), 0, channel)
                        +(1-u)*v*neighborImg(valueWidth(x, neighborImg.width()), valueHeight(y+1, neighborImg.height()), 0, channel)
                        +u*(1-v)*neighborImg(valueWidth(x+1, neighborImg.width()), valueHeight(y, neighborImg.height()), 0, channel)
                        +u*v*neighborImg(valueWidth(x+1, neighborImg.width()), valueHeight(y+1, neighborImg.height()), 0, channel));    				
    			}
			}
    	}
    }

    // update neighbor image's keypoint.
    for (int i = 0; i < imgFeatures[neighbor].keypoints.size(); i++) {
    	double x = imgFeatures[neighbor].keypoints[i].x;
    	double y = imgFeatures[neighbor].keypoints[i].y;
    	imgFeatures[neighbor].keypoints[i].x = backward_h[0]*x+backward_h[1]*y+backward_h[2]+offsetX;
    	imgFeatures[neighbor].keypoints[i].y = backward_h[3]*x+backward_h[4]*y+backward_h[5]+offsetY;
    	// imgFeatures[neighbor].keypoints[i].x = (backward_h[0]*x+backward_h[1]*y+backward_h[2])/(backward_h[6]*x+backward_h[7]*y+backward_h[8])+offsetX;
    	// imgFeatures[neighbor].keypoints[i].y = (backward_h[3]*x+backward_h[4]*y+backward_h[5])/(backward_h[6]*x+backward_h[7]*y+backward_h[8])+offsetY;
    }

    (img1, img2).display("test");

    img1.save_jpeg("img1.jpg");
    img2.save_jpeg("img2.jpg");

     if (img1.width() > img1.height()) {  // stitch images from left to right
     	if (offsetX == 0) {
     		return image_blend(img1, img2, true);
     	} else {
     		return image_blend(img2, img1, true);
     	}
     } else if (img1.height() > img1.width()) {   // stitch images from top to bottom
     	if (offsetY == 0) {
     		return image_blend(img1, img2, false);
     	} else {
     		return image_blend(img2, img1, false);
     	}
     } else {
     	assert(false);
     }

}

template<class T>
CImg<T> image_combine(CImg<T> &leftImg, CImg<T> &rightImg, bool leftToRight) {
	CImg<T> ret(leftImg.width(), leftImg.height(), leftImg.depth(), leftImg.spectrum(), 0);
	if (leftToRight) {
		for (int i = 0; i < ret.width(); i++) {
			for (int j = 0; j < ret.height(); j++) {
				for (int channel = 0; channel < 3; channel++) {
					ret(i, j, 0, channel) = leftImg(i, j, 0, channel);

				}
			}
		}
		for (int i = 0; i < ret.width(); i++) {
			for (int j = 0; j < ret.height(); j++) {
				for (int channel = 0; channel < 3; channel++) {
					if (ret(i, j, 0, channel) == 0) {
						ret(i, j, 0, channel) = rightImg(i, j, 0, channel);
					}
				}
			}
		}
	} else {

	}
	return ret;
}

template<class T>
CImg<T> ImageStitch<T>::image_blend(CImg<T> &leftImg, CImg<T> &rightImg, bool leftToRight) {
	assert(leftImg.width() == rightImg.width() && leftImg.height() == rightImg.height());
	int width = leftImg.width(), height = leftImg.height(), spectrum = leftImg.spectrum();
	CImg<T> ret(width, height, 1, spectrum);
	if (leftToRight) {

		vector<CImg<T> > la = get_laplacian_pyramin(leftImg);
	    vector<CImg<T> > lb = get_laplacian_pyramin(rightImg);
	    vector<CImg<T> > ls = laplacian_combine(la, lb, true);
	    assert(ls[0].width() == width && ls[0].height() == height);
	    return ls[0];
		//return image_combine(leftImg, rightImg, true);
	} else {
		vector<CImg<T> > la = get_laplacian_pyramin(leftImg);
		vector<CImg<T> > lb = get_laplacian_pyramin(rightImg);
		vector<CImg<T> > ls = laplacian_combine(la, lb, false);
		assert(ls[0].width() == width && ls[0].height() == height);
		return ls[0];
		//return image_combine(leftImg, rightImg, false);
	}

}

template<class T>
vector<CImg<T> > ImageStitch<T>::get_laplacian_pyramin(const CImg<T> &img) {
	vector<CImg<T> > ret;
	int width = img.width(), height = img.height();
	CImg<T> g0 = img.get_blur(2.5);
	CImg<T> g1 = g0.get_resize(g0.width()/2, g0.height()/2);
	CImg<T> g2 = g1.get_resize(g1.width()/2, g1.height()/2);
	CImg<T> g3 = g2.get_resize(g2.width()/2, g2.height()/2);
	CImg<T> l2 = g2 - g3.get_resize(g2.width(), g2.height());
	CImg<T> l1 = g1 - g2.get_resize(g1.width(), g1.height());
	CImg<T> l0 = g0 - g1.get_resize(g0.width(), g0.height());
	ret.push_back(l0);
	ret.push_back(l1);
	ret.push_back(l2);
	ret.push_back(g3);
	return ret;
}

template<class T>
vector<CImg<T> > ImageStitch<T>::laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, bool leftToRight) {
	assert(la.size() == lb.size());
	int spectrum = la[0].spectrum();
	vector<CImg<T> > ls(4);

	if (leftToRight) {
		for (int i = 0; i < la.size(); i++) {
			ls[i].assign(la[i].width(), la[i].height(), la[i].depth(), la[i].spectrum(), 0);
			for (int col = 0; col < la[i].width(); ++col) {
				for (int row = 0; row < la[i].height(); ++row) {
					if (col < la[i].width()/2) {
						for (int channel = 0; channel < spectrum; channel++) {
							ls[i](col, row, 0, channel) = la[i](col ,row, 0, channel);
						}
					} else {
						for (int channel = 0; channel < spectrum; channel++) {
							ls[i](col, row, 0, channel) = lb[i](col, row, 0, channel);
						}
					}
				}
			}
		}
	} else {
		for (int i = 0; i < la.size(); i++) {
			ls[i].assign(la[i].width(), la[i].height(), la[i].depth(), la[i].spectrum(), 0);
			for (int col = 0; col < la[i].width(); col++) {
				for (int row = 0; row < la[i].height(); row++) {
					if (row < la[i].height()/2) {
						for (int channel = 0; channel < spectrum; channel++) {
							ls[i](col, row, 0, channel) = la[i](col, row, 0, channel);
						}
					} else {
						for (int channel = 0; channel < spectrum; channel++) {
							ls[i](col, row, 0, channel) = lb[i](col, row, 0, channel);
						}
					}
				}
			}
		}
	}

	vector<CImg<T> > ret(4);
	ret[3] = ls[3];
	for (int i = 2; i >= 0; i--) {
		ret[i] = ret[i+1].get_resize(ls[i].width(), ls[i].height()) + ls[i];
	}
	return ret;
}

template<class T>
void ImageStitch<T>::image_stitch(CImg<T> &res, int cur, int target, vector<int> &neighbors,
	vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs, 
	const vector<CImg<T> > &imgs, bool* isUsed) {
	double h[9];
	float epsilon = 6.0;
	map<int, vector<Pair> >::iterator it = pointPairs.find(neighbors[target]);

	assert(it != pointPairs.end());

	ImgFeature lf = imgFeatures[cur];
	ImgFeature rf = imgFeatures[neighbors[target]];
	ransac(h, it->second, lf.keypoints, rf.keypoints, epsilon);
	res = image_stitch(res, neighbors[target], neighbors, imgs, h, imgFeatures, isUsed);
}



/**
* Project neighbor image onto res image to form a new image.
* Don't forget to update the keypoint of neighbor image,
* since their positions have been projected onto the new image.
*/
template<class T>
CImg<T> ImageStitch<T>::image_stitch(CImg<T> &res, int target, vector<int> &neighbors,
	const vector<CImg<T> > &imgs, double h[], vector<ImgFeature> &imgFeatures, bool* isUsed) {

	CImg<T> ret;

	CImg<T> projImg(imgs[target]);
    assert(res.spectrum() == projImg.spectrum());

    Mat m = Mat(3, 3, CV_64FC1, h);
    Mat inv = m.inv();

    int width = projImg.width()-1, height = projImg.height()-1;
    Point2f lt(inv.at<double>(0,2), inv.at<double>(1,2)),
            lb(inv.at<double>(0,0)*0+inv.at<double>(0,1)*height+inv.at<double>(0,2),
            	inv.at<double>(1,0)*0+inv.at<double>(1,1)*height+inv.at<double>(1,2)),
            rt(inv.at<double>(0,0)*width+inv.at<double>(0,1)*0+inv.at<double>(0,2),
            	inv.at<double>(1,0)*width+inv.at<double>(1,1)*0+inv.at<double>(1,2)),
            rb(inv.at<double>(0,0)*width+inv.at<double>(0,1)*height+inv.at<double>(0,2),
            	inv.at<double>(1,0)*width+inv.at<double>(1,1)*height+inv.at<double>(1,2));

    int maxX = max(lt.x, max(lb.x, max(rt.x, rb.x)));
    int minX = min(lt.x, min(lb.x, min(rt.x, rb.x)));
    int maxY = max(lt.y, max(lb.y, max(rt.y, rb.y)));
    int minY = min(lt.y, min(lb.y, min(rt.y, rb.y)));

    int offsetX, offsetY;
    width = res.width();
    height = res.height();
    // projImg projects to the left of res
    if (minX <= 0) {
    	offsetX = 0 - minX;
    	width = res.width() - minX;
    } else if (maxX >= res.width()) {  // projImg projects to the right of res
    	offsetX = 0;
    	width = maxX;
    } else {
    	
    }

    // projImg projects to the top of res
    if (minY <= 0) {
    	offsetY = 0 - minY;
    	height = res.height() - minY;
    } else if (maxY >= res.height()) {  // projImg projects to the bottom of res
    	offsetY = 0;
    	height = maxY;
    } else {
    	
    }

    // update the keypoint of neighbor image
    ImgFeature imgFeature = imgFeatures[target];
    for (int i = 0; i < imgFeatures[target].keypoints.size(); i++) {
    	double x = imgFeatures[target].keypoints[i].x;
    	double y = imgFeatures[target].keypoints[i].y;
    	imgFeatures[target].keypoints[i].x = inv.at<double>(0,0)*x+inv.at<double>(0,1)*y+inv.at<double>(0,2)+offsetX;
    	imgFeatures[target].keypoints[i].y = inv.at<double>(1,0)*x+inv.at<double>(1,1)*y+inv.at<double>(1,2)+offsetY;
    }
    for (int i = 0; i < neighbors.size(); i++) {
    	// isUsed[neighbors[i]]==true表示已经拼接上去，此时才需要更新坐标
    	if (neighbors[i] != target && isUsed[neighbors[i]] == true) {
    		for (int j = 0; j < imgFeatures[neighbors[i]].keypoints.size(); j++) {
    			imgFeatures[neighbors[i]].keypoints[j].x += offsetX;
    			imgFeatures[neighbors[i]].keypoints[j].y += offsetY;
    		}
    	}
    }

    // CImg<T> subA(res.width(), res.height(), res.depth(), res.spectrum(), 0);
    // CImg<T> subB(projImg.width(), projImg.height(), projImg.depth(), projImg.spectrum(), 0);
    // get_overlap(subA, res, m, projImg);
    // get_overlap(subB, projImg, inv, res);

    // vector<CImg<T> > lA = get_laplacian_pyramin(subA);
    // vector<CImg<T> > lB = get_laplacian_pyramin(subB);
    // ret = laplacian_combine(lA, lB, h, width, height, res.spectrum(), offsetX, offsetY);

    ret.assign(width, height, 1, res.spectrum(), 0);

	for (int i = 0; i < ret.width(); i++) {
		for (int j = 0; j < ret.height(); j++) {
			int ii = i - offsetX, jj = j - offsetY;
			double x = ii*h[0]+jj*h[1]+h[2], y = ii*h[3]+jj*h[4]+h[5];
			if (x >= 0 && x < projImg.width() && y >= 0 && y < projImg.height()) {
				double u = x - (int)x, v = y - (int)y;
    			for (int channel = 0; channel < res.spectrum(); channel++) {
                    ret(i, j, 0, channel) = 
                        (int)((1-u)*(1-v)*projImg(valueWidth(x, projImg.width()), valueHeight(y, projImg.height()), 0, channel)
                        +(1-u)*v*projImg(valueWidth(x, projImg.width()), valueHeight(y+1, projImg.height()), 0, channel)
                        +u*(1-v)*projImg(valueWidth(x+1, projImg.width()), valueHeight(y, projImg.height()), 0, channel)
                        +u*v*projImg(valueWidth(x+1, projImg.width()), valueHeight(y+1, projImg.height()), 0, channel));    				
    			}
			} else if (ii >= 0 && ii < res.width() && jj >= 0 && jj < res.height()) {
			    for (int channel = 0; channel < res.spectrum(); channel++) {
				    ret(i, j, 0, channel) = res(ii, jj, 0, channel);    				
		        }
		    }
     	}
    }
   
	return ret;
}

template<class T>
CImg<T> ImageStitch<T>::laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, 
	double h[], int width, int height, int spectrum, int offsetX, int offsetY) {
	vector<CImg<T> > ls(4);
	vector<CImg<T> > ret(4);

	for (int i = 0; i < ls.size(); i++) {
		ls[i].assign(width/(i+1), height/(i+1), 1, spectrum, 0);

		for (int col = 0; col < ls[i].width(); col++) {
			for (int row = 0; row < ls[i].height(); row++) {
				int c = col - offsetX/(i+1), r = row - offsetY/(i+1);
				double x = c*h[0]+r*h[1]+h[2], y = c*h[3]+r*h[4]+h[5];
				if (x >= 0 && x < lb[i].width() && y >= 0 && y < lb[i].height()) {
					double u = x - (int)x, v = y - (int)y;
					for (int channel = 0; channel < spectrum; channel++) {
						ls[i](col, row, 0, channel) = 
						    (int)((1-u)*(1-v)*lb[i](valueWidth(x, lb[i].width()), valueHeight(y, lb[i].height()), 0, channel)
						    +(1-u)*v*lb[i](valueWidth(x, lb[i].width()), valueHeight(y+1, lb[i].height()), 0, channel)
						    +u*(1-v)*lb[i](valueWidth(x+1, lb[i].width()), valueHeight(y, lb[i].height()), 0, channel)
						    +u*v*lb[i](valueWidth(x+1, lb[i].width()), valueHeight(y+1, lb[i].height()), 0, channel));
					}
				} else if (c >= 0 && c < la[i].width() && r >= 0 && r < la[i].height()) {
					for (int channel = 0; channel < la[i].spectrum(); channel++) {
						ls[i](col, row, 0, channel) = la[i](c, r, 0, channel);
					}
				}
			}
		}
	}

    ret[3] = ls[3];
    for (int i = 2; i >= 0; i--) {
    	ret[i] = ls[i] + ls[i+1].resize(ls[i].width(), ls[i].height());
    }

	return ret[0];
}

template<class T>
void ImageStitch<T>::ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
	vector<VlSiftKeypoint>& keypoints2, float epsilon) {
	int loop_times = calc_iterations(0.2, 0.99, 4);
	#ifdef DEBUG
	cout << "loop_times===>" << loop_times << endl;
	#endif

	int max_inliers = 0;
	double tempH[9];

	while (loop_times--) {
		vector<Pair> randomPairs = randomly_select(pairs);
		calc_homography(randomPairs, keypoints1, keypoints2, tempH);
		int inliers = calc_inliers(pairs, keypoints1, keypoints2, tempH, epsilon);
		if (inliers > max_inliers) {
			for (int i = 0; i < 9; i++) {
				h[i] = tempH[i];
			}
			max_inliers = inliers;
		}
	}
	#ifdef DEBUG
	cout << "max_inliers====>" << max_inliers << endl;
	#endif
	//recomputer_least_squares(keypoints1, keypoints2, h);
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

	Mat matrix = findHomography(srcV, destV);
	//Mat matrix = findHomography(destV, srcV);
	int nRows = matrix.rows;
	int nCols = matrix.cols;
	for (int i = 0; i < nRows; i++) {
		for (int j = 0; j < nCols; j++) {
			h[i*nCols+j] = matrix.at<double>(i, j);
		}
	}
}

template<class T>
int ImageStitch<T>::calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon) {
	int inliers = 0;
	for (int i = 0; i < pairs.size(); i++) {
		VlSiftKeypoint srcP = keypoints1[pairs[i].k1];
		VlSiftKeypoint destP = keypoints2[pairs[i].k2];
		double x, y;
		x = srcP.x*tempH[0] + srcP.y*tempH[1] + tempH[2];
		y = srcP.x*tempH[3] + srcP.y*tempH[4] + tempH[5];
		if ((x-destP.x)*(x-destP.x)+(y-destP.y)*(y-destP.y) < epsilon*epsilon) {
			++inliers;
		}
	}
	return inliers;
}
    
template<class T>
void ImageStitch<T>::recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double h[9]) {

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
    #ifdef DEBUG
    //grayImg.display();
    #endif
    return grayImg;
}



#endif