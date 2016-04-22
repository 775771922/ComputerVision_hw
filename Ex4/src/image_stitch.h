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
	CImg<T> image_stitch(CImg<T> &res, int neighbor, const vector<CImg<T> > &imgs, 
		double h[], vector<ImgFeature> &imgFeatures);
	void image_stitch(CImg<T> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures, 
		map<int, vector<Pair> > &pointPairs, const vector<CImg<T> > &imgs);
	vector<int> find_nearest_neighbor(int cur, const bool* isProjected, 
		vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs);
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
	vector<ImgFeature> imgFeatures;
	bool* isProjected = new bool[imgs.size()];
	memset(isProjected, 0, sizeof(bool)*imgs.size());
	for (int i = 0; i < imgs.size(); i++) {
		calc_img_feature(imgFeatures, imgs[i]);
	}

	int a;
	cin >> a;

	#ifdef DEBUG
	cout << "finish calc_img_feature" << endl;
	#endif

	queue<int> q;
	int randomIndex = rand() % imgs.size();
	q.push(randomIndex);
	isProjected[randomIndex] = true;
	CImg<T> res(imgs[randomIndex]);


string name = "A.jpg";

	// 要求所有图片都能找到匹配的
	while (!q.empty()) {
		int cur = q.front();
		q.pop();
		map<int, vector<Pair> > pointPairs;

        #ifdef DEBUG
        cout << "current===>" << cur << endl;
        #endif


		vector<int> neighbors = find_nearest_neighbor(cur, isProjected, imgFeatures, pointPairs);

		#ifdef DEBUG
		cout << "neighbors====>" << neighbors.size() << endl;
		cout << endl;
		cout << "start stitch===>" << endl;
		#endif


		for (int i = 0; i < neighbors.size(); i++) {
			q.push(neighbors[i]);
			image_stitch(res, cur, neighbors[i], imgFeatures, pointPairs, imgs);
			isProjected[neighbors[i]] = true;


            #ifdef DEBUG
            
            res.display();
            res.save_jpeg(name.c_str());
            name[0]++;
            #endif

		}
		imgFeatures[cur].clear();
	}

	delete [] isProjected;
	isProjected = NULL;

	return res;
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
	#ifdef DEBUG
	cout << "find_nearest_neighbor===>" << endl;
	cout << "cur===>" << cur << endl;
	#endif

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

        #ifdef DEBUG
        cout << "i====>" << i << endl;
        #endif

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
void ImageStitch<T>::image_stitch(CImg<T> &res, int cur, int neighbor, 
	vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs,
	const vector<CImg<T> > &imgs) {
	double h[9];
	float epsilon = 6.0;
	map<int, vector<Pair> >::iterator it = pointPairs.find(neighbor);

	assert(it != pointPairs.end());

	ImgFeature lf = imgFeatures[cur];
	ImgFeature rf = imgFeatures[neighbor];
	ransac(h, it->second, lf.keypoints, rf.keypoints, epsilon);
	res = image_stitch(res, neighbor, imgs, h, imgFeatures);
}



/**
* Project neighbor image onto res image to form a new image.
* Don't forget to update the keypoint of neighbor image,
* since their positions have been projected onto the new image.
*/
template<class T>
CImg<T> ImageStitch<T>::image_stitch(CImg<T> &res, int neighbor, 
	const vector<CImg<T> > &imgs, double h[], vector<ImgFeature> &imgFeatures) {

	CImg<T> ret;

	CImg<T> projImg(imgs[neighbor]);
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

    #ifdef DEBUG
    cout << "width===>" << res.width() << endl
         << "height===>" << res.height() << endl; 
    cout << "maxX====>" << maxX << endl
         << "minX====>" << minX << endl
         << "maxY====>" << maxY << endl
         << "minY====>" << minY << endl;
    #endif

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
    ImgFeature imgFeature = imgFeatures[neighbor];
    for (int i = 0; i < imgFeatures[neighbor].keypoints.size(); i++) {
    	double x = imgFeatures[neighbor].keypoints[i].x;
    	double y = imgFeatures[neighbor].keypoints[i].y;
    	imgFeatures[neighbor].keypoints[i].x = inv.at<double>(0,0)*x+inv.at<double>(0,1)*y+inv.at<double>(0,2)+offsetX;
    	imgFeatures[neighbor].keypoints[i].y = inv.at<double>(1,0)*x+inv.at<double>(1,1)*y+inv.at<double>(1,2)+offsetY;
    }

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
void ImageStitch<T>::ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
	vector<VlSiftKeypoint>& keypoints2, float epsilon) {
	int loop_times = 800;
	int max_inliers = 0;
	double tempH[9];
	bool isFound = false;

    #ifdef Image_Stitch_DEBUG
    ofstream fout("point.txt");
    fout << "all keyPoint===>" << keypoints1.size() << endl;
    #endif

	while (loop_times--&&!isFound) {
		vector<Pair> randomPairs = randomly_select(pairs);
		calc_homography(randomPairs, keypoints1, keypoints2, tempH);
		int inliers = calc_inliers(pairs, keypoints1, keypoints2, tempH, epsilon);
		if (inliers > max_inliers) {
			for (int i = 0; i < 9; i++) {
				h[i] = tempH[i];
			}
			max_inliers = inliers;
		}

		if (inliers > pairs.size()*0.8) {
			isFound = true;
		}

		#ifdef Image_Stitch_DEBUG
		cout << "loop_times====>" << loop_times << endl;
		fout << "inliers===>" << inliers << endl;
		#endif
	}
	recomputer_least_squares(keypoints1, keypoints2, h);
}

template<class T>
vector<Pair> ImageStitch<T>::randomly_select(vector<Pair> &pairs) {
	vector<Pair> ret;
	bool flag[pairs.size()];
	memset(flag, 0, sizeof(flag));

	#ifdef Image_Stitch_DEBUG
	// ofstream fout("random.txt", ofstream::app);
	// fout << "randomly_select" << endl;
	#endif

	for (int i = 0; i < 4; i++) {
		while (true) {
			int randomIndex = rand() % pairs.size();
			if (flag[randomIndex] == false) {
				flag[randomIndex] = true;
				ret.push_back(pairs[randomIndex]);

                #ifdef Image_Stitch_DEBUG
                //fout << "random number====>" << randomIndex << endl;
                #endif

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