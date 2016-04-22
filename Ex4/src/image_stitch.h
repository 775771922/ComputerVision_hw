#ifndef IMAGE_STITCH_H
#define IMAGE_STITCH_H
#include <vector>
#include <map>
#include "CImg.h"
using namespace cimg_library;
using namespace std;

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

class ImageStitch {
private:
	int noctaves, nlevels, o_min;
    int valueWidth(double srcX, int width);
    int valueHeight(double srcY, int height);
	CImg<float> get_gray_image(const CImg<float> &srcImg);
	void ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
		vector<VlSiftKeypoint>& keypoints2, float epsilon);
	vector<Pair> randomly_select(vector<Pair> &pairs);
	void calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
        vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon);
    void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double h[9]);
	void calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<float> &img);
	CImg<float> image_stitch(CImg<float> &res, int neighbor, const vector<CImg<float> > &imgs, 
		double h[], vector<ImgFeature> &imgFeatures);
	void image_stitch(CImg<float> &res, int cur, int neighbor, vector<ImgFeature> &imgFeatures, 
		map<int, vector<Pair> > &pointPairs, const vector<CImg<float> > &imgs);
	vector<int> find_nearest_neighbor(int cur, const bool* isProjected, 
		vector<ImgFeature> &imgFeatures, map<int, vector<Pair> > &pointPairs);
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<float> image_stitch(const vector<CImg<float> > &imgs);	

};

#endif