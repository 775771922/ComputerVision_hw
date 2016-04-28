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
	void ransac(double forward[9], double backward[9], vector<Pair>& pairs, 
	    vector<VlSiftKeypoint> &keypoints1, vector<VlSiftKeypoint>& keypoints2, float epsilon);

	vector<Pair> randomly_select(vector<Pair> &pairs);
	void calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
        vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon);
    void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double h[9]);
	void calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<T> &img);
	vector<int> find_nearest_neighbor(int cur, const bool* isProjected, 
		vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs);
    vector<CImg<T> > get_laplacian_pyramin(const CImg<T> &img);
    CImg<T> laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, 
	    double h[], int width, int height, int spectrum, int offsetX, int offsetY);
    CImg<T> get_cylindrical_proj(const CImg<T> &img);

    vector<CImg<float> > get_gaussian_pyramin(const CImg<float> &R);
    vector<CImg<T> > laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, vector<CImg<float> > &R);
    int calc_iterations(double p, double P, int n);
    CImg<T> image_blend(CImg<T> &img1, CImg<T> &img2);
    CImg<T> image_stitch_and_blend(CImg<T> &res, int cur, int neighbor,
	    vector<ImgFeature> &imgFeatures, const vector<CImg<T> > &imgs, double forward_h[],
	    double backward_h[], bool* isProjected);
    CImg<T> image_stitch(CImg<T> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures,
	    map<int, vector<Pair> > &pointPairs, const vector<CImg<T> > &imgs, bool *isProjected);
    vector<CImg<T> > laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, bool leftToRight);
    CImg<T> image_blend(CImg<T> &leftImg, CImg<T> &rightImg, bool leftToRight);
    CImg<T> image_blend(CImg<T> &leftImg, CImg<T> &rightImg, CImg<float> &R, bool leftToRight);
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<T> image_stitch(const vector<CImg<T> > &imgs);

    #ifdef DEBUG
    vector<CImg<T> > hh;
    #endif
};


template<class T>
ImageStitch<T>::ImageStitch(int octaves, int levels, int o_min)
           : noctaves(octaves), nlevels(levels), o_min(o_min) {
}

template<class T>
CImg<T> ImageStitch<T>::image_stitch(const vector<CImg<T> > &inputImgs) {
	srand((unsigned)time(0));

	vector<CImg<T> > imgs(inputImgs);

    // for (int i = 0; i < imgs.size(); i++) {
    // 	imgs[i] = get_cylindrical_proj(inputImgs[i]);
    // 	imgs[i].display();
    // 	char name[3];
    // 	sprintf(name, "%d", i);
    // 	string s(string(name)+".jpg");
    // 	imgs[i].save_jpeg(s.c_str());
    // }

	#ifdef DEBUG
	hh = imgs;
	#endif

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

    string name = "A.jpg";

    CImg<T> res(imgs[randomIndex]);

	// 要求所有图片都能找到匹配的
	while (!q.empty()) {
		int cur = q.front();
		q.pop();
		map<int, vector<Pair> > pointPairs;

		vector<int> neighbors = find_nearest_neighbor(cur, isProjected, imgFeatures, pointPairs);

		#ifdef DEBUG
		cout << "neighbors size===>" << neighbors.size() << endl;
		#endif

		for (int i = 0; i < neighbors.size(); i++) {
			q.push(neighbors[i]);
			res = image_stitch(res, cur, neighbors[i], imgFeatures, pointPairs, imgs, isProjected);
			isProjected[neighbors[i]] = true;
			
            #ifdef DEBUG
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
	double alpha = 1.2*180/360;
	int width = img.width(), height = img.height(), spectrum = img.spectrum();
	//double R = 900.0;
	//cin >> R;
    double R = width / (2*tan(alpha/2));
    cout << "R====>" << R << endl;
 
    Point2f lt((0-width/2)*(R/sqrt(R*R+(0-width/2)*(0-width/2))) + width/2,
               (0-height/2)*(R/sqrt(R*R+(0-width/2)*(0-width/2))) + height/2),
            rb((width-1-width/2)*(R/sqrt(R*R+(width-1-width/2)*(width-1-width/2))) + width/2, 
               (height-1-height/2)*(R/sqrt(R*R+(width-1-width/2)*(width-1-width/2))) + height/2);

    //int offsetX = lt.x, offsetY = rb.y;
    width = abs(rb.x - lt.x);
    height = abs(rb.y - lt.y);
    cout << "width====>" << width << endl;
    cout << "height====>" << height << endl;
	
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

	float scale;
	bool isScaled = false;
	if (tempImg.width() > 500 || tempImg.height() > 500) {
		isScaled = true;
		if (tempImg.width() > tempImg.height()) {
			scale = 500.0 / tempImg.width();
			tempImg.resize((tempImg.width()*scale), (int)(tempImg.height()*scale), 1, 1, 3);
		} else {
			scale = (float)500 / tempImg.height();
			tempImg.resize((int)(tempImg.width()*scale), (int)(tempImg.height()*scale), 1, 1, 3);
		}

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

				//restore to original scale if necessary
				if (isScaled) {
					t.x = (int)(tempKeyPoint.x / scale);
					t.y = (int)(tempKeyPoint.y / scale);
				}

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

		// if (indexs.size() >= 2) {
		// 	break;
		// }

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
	float epsilon = 4.0;
	map<int, vector<Pair> >::iterator it = pointPairs.find(neighbor);

	ImgFeature lf = imgFeatures[cur];
	ImgFeature rf = imgFeatures[neighbor];
	ransac(forward_h, backward_h, it->second, lf.keypoints, rf.keypoints, epsilon);

    #ifdef DEBUG
    cout << "start stitch====>\n";
    #endif

	return image_stitch_and_blend(res, cur, neighbor, imgFeatures, imgs, forward_h, backward_h, isProjected);
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
	}
	return ret;
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

    // for mutilband blend
    CImg<float> R(width, height, 1, 1, 0);

    for (int i = 0; i < img1.width(); i++) {
    	for (int j = 0; j < img1.height(); j++) {    		
			int projectI = i - offsetX, projectJ = j - offsetY;
			if (projectI >= 0 && projectI < res.width() && projectJ >= 0 && projectJ < res.height()) {
    			for (int channel = 0; channel < res.spectrum(); channel++) {
    				img1(i, j, 0, channel) = res(projectI, projectJ, 0, channel);
    			}
    			double x = Homography::calc_X(projectI, projectJ, forward_h),
    		          y = Homography::calc_Y(projectI, projectJ, forward_h);

	    		// if (x >= 0 && x < neighborImg.width() && y >= 0 && y < neighborImg.height()) {
	    		// 	R(i, j, 0, 0) = 1.0;
	    		// }
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

    (img1, img2).display("hhhhhhhhhhhhhhhh!");

    img1.save_jpeg("img1.jpg");
    img2.save_jpeg("img2.jpg");


     if (img1.width() > img1.height()) {  // stitch images from left to right
     	if (offsetX == 0) {
     		return image_blend(img1, img2, R, true);
     	} else {
     		return image_blend(img2, img1, R, true);
     	}
     } else if (img1.height() > img1.width()) {   // stitch images from top to bottom
     	if (offsetY == 0) {
     		return image_blend(img1, img2, R, false);
     	} else {
     		return image_blend(img2, img1, R, false);
     	}
     } else {
     	assert(false);
     }

}

template<class T>
CImg<T> ImageStitch<T>::image_blend(CImg<T> &leftImg, CImg<T> &rightImg, 
	CImg<float> &R, bool leftToRight) {

    #ifdef DEBUG
    // R.display("R");
    // R.save_jpeg("R.jpg");
    #endif

	assert(leftImg.width() == rightImg.width() && leftImg.height() == rightImg.height());
	int width = leftImg.width(), height = leftImg.height(), spectrum = leftImg.spectrum();
	CImg<T> ret(width, height, 1, spectrum);
	vector<CImg<T> > la;
    vector<CImg<T> > lb;
    vector<CImg<float> > gau_pyramin_R;
    vector<CImg<T> > ls;
	if (leftToRight) {

		// la = get_laplacian_pyramin(leftImg);
	 //    lb = get_laplacian_pyramin(rightImg);
	 //    //gau_pyramin_R = get_gaussian_pyramin(R);
	 //    //ls = laplacian_combine(la, lb, gau_pyramin_R);
	 //    ls = laplacian_combine(la, lb, true);
	 //    assert(ls[0].width() == width && ls[0].height() == height);
	 //    return ls[0];
	    
		return image_combine(leftImg, rightImg, true);
	} else {
		// la = get_laplacian_pyramin(leftImg);
		// lb = get_laplacian_pyramin(rightImg);
		// gau_pyramin_R = get_gaussian_pyramin(R);
		// ls = laplacian_combine(la, lb, gau_pyramin_R);
		// assert(ls[0].width() == width && ls[0].height() == height);
		
		return image_combine(leftImg, rightImg, false);
	}
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			if (R(i, j, 0, 0) > 0) {
				for (int channel = 0; channel < spectrum; channel++) {
					ret(i, j, 0, channel) = ls[0](i, j, 0, channel);
				}
			} else if (leftImg(i, j, 0, 0) > 0) {
				for (int channel = 0; channel < spectrum; channel++) {
					ret(i, j, 0, channel) = leftImg(i, j, 0, channel);
				}
			} else if (rightImg(i, j, 0, 0) > 0) {
				for (int channel = 0; channel < spectrum; channel++) {
					ret(i, j, 0, channel) = rightImg(i, j, 0, channel);
				}
			} else {
				//assert(false);
			}
		}
	}
	return ret;

}

template<class T>
vector<CImg<float> > ImageStitch<T>::get_gaussian_pyramin(const CImg<float> &R) {
	vector<CImg<float> > ret;
	int width = R.width(), height = R.height();
	CImg<float> g0 = R;
	CImg<float> g1 = g0.get_blur(1.5).get_resize(g0.width()/2, g0.height()/2, g0.depth(), g0.spectrum(), 3);
	CImg<float> g2 = g1.get_blur(1.5).get_resize(g1.width()/2, g1.height()/2, g1.depth(), g1.spectrum(), 3);
	CImg<float> g3 = g2.get_blur(1.5).get_resize(g2.width()/2, g2.height()/2, g2.depth(), g2.spectrum(), 3);
	ret.push_back(g0);
	ret.push_back(g1);
	ret.push_back(g2);
	ret.push_back(g3);
	return ret;
}

template<class T>
vector<CImg<T> > ImageStitch<T>::get_laplacian_pyramin(const CImg<T> &img) {
	vector<CImg<T> > ret;
	int width = img.width(), height = img.height();
	CImg<T> g0 = img;
	CImg<T> g1 = g0.get_blur(1.5).get_resize(g0.width()/2, g0.height()/2, g0.depth(), g0.spectrum(), 3);
	CImg<T> g2 = g1.get_blur(1.5).get_resize(g1.width()/2, g1.height()/2, g1.depth(), g1.spectrum(), 3);
	CImg<T> g3 = g2.get_blur(1.5).get_resize(g2.width()/2, g2.height()/2, g2.depth(), g2.spectrum(), 3);
	CImg<T> l2 = g2 - g3.get_resize(g2.width(), g2.height(), g2.depth(), g2.spectrum(), 3);
	CImg<T> l1 = g1 - g2.get_resize(g1.width(), g1.height(), g1.depth(), g1.spectrum(), 3);
	CImg<T> l0 = g0 - g1.get_resize(g0.width(), g0.height(), g0.depth(), g0.spectrum(), 3);
	ret.push_back(l0);
	ret.push_back(l1);
	ret.push_back(l2);
	ret.push_back(g3);
	return ret;
}

template<class T>
vector<CImg<T> > ImageStitch<T>::laplacian_combine(vector<CImg<T> > &la, vector<CImg<T> > &lb, 
	vector<CImg<float> > &R) {
	assert(la.size() == lb.size());
	int spectrum = la[0].spectrum();
	vector<CImg<T> > ls(4);
	
	for (int i = 0; i < la.size(); i++) {
		ls[i].assign(la[i].width(), la[i].height(), la[i].depth(), la[i].spectrum(), 0);
		for (int col = 0; col < la[i].width(); col++) {
			for (int row = 0; row < la[i].height(); row++) {
				for (int channel = 0; channel < spectrum; channel++) {
					float u = R[i](col, row, 0, 0);
					ls[i](col, row, 0, channel) = u*la[i](col, row, 0, channel) + (1-u)*lb[i](col ,row, 0, channel);
				}
			}
		}
	}

	vector<CImg<T> > ret(4);
	ret[3] = ls[3];
	for (int i = 2; i >= 0; i--) {
		ret[i] = ret[i+1].get_resize(ls[i].width(), ls[i].height()) + ls[i];
	}


    #ifdef DEBUG
    ret[0].display("ret[0]");
    ret[0].save_jpeg("ret[0].jpg");
    #endif

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
		ret[i] = ret[i+1].get_resize(ls[i].width(), ls[i].height(), ls[i].depth(), ls[i].spectrum(), 3) + ls[i];
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
void draw_point(CImg<T> &img, int x, int y, double circle) {
	assert(x >= 0 && x < img.width() && y >= 0 && y < img.height());
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
        	for (int channel = 0; channel < img.spectrum(); channel++) {
        		if (sqrt((i-x)*(i-x)+(j-y)*(j-y)) < circle) {
                    img(i, j, 0, channel) = 0;
                }
        	}
        }
    }
}

template<class T>
void ImageStitch<T>::ransac(double forward[9], double backward[9], vector<Pair>& pairs, 
	vector<VlSiftKeypoint> &keypoints1, vector<VlSiftKeypoint>& keypoints2, float epsilon) {
	int loop_times = calc_iterations(0.3, 0.99, 4);
	#ifdef DEBUG
	cout << "loop_times===>" << loop_times << endl;
	#endif

	int max_inliers = 0;
	double tempFH[9], tempBH[9];

	while (loop_times--) {
		vector<Pair> randomPairs = randomly_select(pairs);

		calc_homography(randomPairs, keypoints1, keypoints2, tempFH);

		vector<Pair> tempPairs;
		for (int i = 0; i < randomPairs.size(); i++) {
			tempPairs.push_back(Pair(randomPairs[i].k2, randomPairs[i].k1));
		}
		calc_homography(tempPairs, keypoints2, keypoints1, tempBH);

		int inliers = calc_inliers(pairs, keypoints1, keypoints2, tempFH, epsilon);

		if (inliers > max_inliers) {

			#ifdef DEBUG
			cout << "inliers=====>" << inliers << endl;
			// CImg<T> a(hh[0]), b(hh[1]);
			// for (int i = 0; i < randomPairs.size(); i++) {
			// 	cout << "(" << keypoints1[randomPairs[i].k1].x << "," <<  keypoints1[randomPairs[i].k1].y << ")\n";
			// 	cout << "(" << keypoints2[randomPairs[i].k2].x << "," <<  keypoints2[randomPairs[i].k2].y << ")\n";
			// 	draw_point(a, keypoints1[randomPairs[i].k1].x, keypoints1[randomPairs[i].k1].y, 5);
			// 	draw_point(b, keypoints2[randomPairs[i].k2].x, keypoints2[randomPairs[i].k2].y, 5);
			// 	(a, b).display("test");
			// }
			#endif

			for (int i = 0; i < 9; i++) {
				forward[i] = tempFH[i];
			}
			max_inliers = inliers;

			for (int i = 0; i < 9; i++) {
				backward[i] = tempBH[i];
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

	//Mat matrix = findHomography(srcV, destV);
	//Mat matrix = getPerspectiveTransform(srcV, destV);
	Homography::calc_homography(srcV, destV, h);

}

template<class T>
int ImageStitch<T>::calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon) {
	int inliers = 0;
	for (int i = 0; i < pairs.size(); i++) {
		VlSiftKeypoint srcP = keypoints1[pairs[i].k1];
		VlSiftKeypoint destP = keypoints2[pairs[i].k2];
		double x, y;
		x = Homography::calc_X(srcP.x, srcP.y, tempH);
		y = Homography::calc_Y(srcP.x, srcP.y, tempH);
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