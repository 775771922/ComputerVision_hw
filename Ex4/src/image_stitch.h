#ifndef IMAGE_STITCH_H
#define IMAGE_STITCH_H
#include <vector>
#include "CImg.h"
using namespace cimg_library;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
}

#define Image_Stitch_DEBUG

// deminsion for sift descriptor
const int dimen = 128;

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
	CImg<float> get_gray_image(const CImg<float> &srcImg);
	vector<Pair> compare(const vector<vl_sift_pix*>& descr1, 
		const vector<vl_sift_pix*>& descr2, float thresh);
	void calc_descriptor(vector<vl_sift_pix*>& descr, VlSiftFilt* siftFilt, 
		vector<VlSiftKeypoint> &keypoints, const CImg<float> &img);
	void ransac(double h[9], const vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
		vector<VlSiftKeypoint>& keypoints2, float epsilon);
	vector<VlSiftKeypoint> randomly_select(vector<VlSiftKeypoint> keypoints);
	void calc_homography(vector<VlSiftKeypoint> &keypoints1, 
		vector<VlSiftKeypoint> &keypoints2, double h[9]);
    int calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon);
    void recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    	vector<VlSiftKeypoint> &keypoints2, double h[9]);

	CImg<float> image_stitch(const CImg<float> &img1, const CImg<float> &img2, 
		double h[], vector<Pair> &pairs);
	double calc_euclidean_distance(vl_sift_pix* descr1, vl_sift_pix* descr2);
public:
	ImageStitch(int octaves, int levels, int o_min);
	CImg<float> image_stitch(const CImg<float> &img1, const CImg<float> &img2);
	

};

#endif