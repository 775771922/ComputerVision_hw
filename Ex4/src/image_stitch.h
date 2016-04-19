#ifndef IMAGE_STITCH_H
#define IMAGE_STITCH_H
#include <vector>
#include "CImg.h"
using namespace cimg_library;
using namespace std;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
	#include "vl/kdtree.h"
}

#define Image_Stitch_DEBUG

// deminsion for sift descriptor
const int dimen = 128;

struct ImgFeature {
	vector<VlSiftKeypoint> keypoints;
	vector<vl_sift_pix*> descr;
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
	vector<Pair> compare(const vector<vl_sift_pix*>& descr1, 
		const vector<vl_sift_pix*>& descr2, float thresh);
	void calc_descriptor(vector<vl_sift_pix*>& descr, VlSiftFilt* siftFilt, 
		vector<VlSiftKeypoint> &keypoints, const CImg<float> &img);
	void ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
		vector<VlSiftKeypoint>& keypoints2, float epsilon);
	vector<Pair> randomly_select(vector<Pair> &pairs);
	void calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
        vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon);
    void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double h[9]);
	CImg<float> image_stitch(const CImg<float> &img1, const CImg<float> &img2, double h[]);
	double calc_euclidean_distance(vl_sift_pix* descr1, vl_sift_pix* descr2);
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<float> image_stitch(const CImg<float> &img1, const CImg<float> &img2);
	CImg<float> image_stitch(vector<CImg<float> > &imgs);

	CImg<float> image_stitch(const vector<CImg<float> > &imgs);
	vector<CImg<float> > image_merge(vector<CImg<float> > &imgs);
	void calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<float> &img);
	bool all_matched(bool* isMatched, int size);
    int random_index(bool *isMatched, int size);
    int find_nearest_neighbor(int cur, vector<CImg<float> > &imgs, bool *isMatched, 
	    vector<ImgFeature> &imgsFeature, vector<Pair> &pointPairs);
    CImg<float> image_stitch(CImg<float> &l, CImg<float> &r, ImgFeature &lf, 
    	ImgFeature &rf, vector<Pair> &pairs);
	

};

#endif