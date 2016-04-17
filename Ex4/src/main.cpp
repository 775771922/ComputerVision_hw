#include <vector>
#include <cmath>
#include <float.h>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <fstream>
#include "CImg.h"
using namespace cimg_library;
using namespace std;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
}

const int dimen = 128;
int noctaves = 5, nlevels = 3, o_min = 0;

struct Pair {
	int k1;
	int k2;
	Pair(int k1, int k2) {
		this->k1 = k1;
		this->k2 = k2;
	}
};

void draw_point(CImg<float> &img, int x, int y, double circle) {
	assert(x >= 0 && x < img.width() && y >= 0 && y < img.height());
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (sqrt((i-x)*(i-x)+(j-y)*(j-y)) < circle) {
                img(i, j, 0, 0) = 0xff;
            }
        }
    }
}

CImg<float> get_gray_image(const CImg<float> &srcImg) {
	if (srcImg.spectrum() == 1) {
		return srcImg;
	}
    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();
    CImg<float> grayImg(width, height, depth, 1);
    float r, g, b, gr;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            r = srcImg(i, j, 0, 0);
            g = srcImg(i, j, 0, 1);
            b = srcImg(i, j, 0, 2);
            gr = 0.299*(r) + 0.587*(g) + 0.114*(b);
            grayImg(i, j, 0, 0) = gr;
        }
    }  
    return grayImg;
}

void calc_descriptor(vector<vl_sift_pix*>& descr, VlSiftFilt* siftFilt, 
	vector<VlSiftKeypoint> &keypoints, const CImg<float> &img) {
	vl_sift_pix* imageData = new vl_sift_pix[img.width()*img.height()];
	for (int i = 0; i < img.height(); i++) {
		for (int j = 0; j < img.width(); j++) {
			imageData[i*img.width()+j] = img(j, i, 0);
		}
	}

	if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
		while (true) {
			vl_sift_detect(siftFilt);
			// 遍历该层的每个点
			VlSiftKeypoint *keyPoint = siftFilt->keys;
			for (int i = 0; i < siftFilt->nkeys; i++) {
				VlSiftKeypoint tempKeyPoint = *keyPoint;
				keypoints.push_back(tempKeyPoint);
				keyPoint++;
				double angles[4];
				vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);
				// 默认只取第一个角度的描述符
				vl_sift_pix* descriptors = new vl_sift_pix[dimen];
				vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, angles[0]);
				descr.push_back(descriptors);
			}

			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}

	delete [] imageData;
	imageData = NULL;
	vl_sift_delete(siftFilt);
	siftFilt = NULL;
}

vector<Pair> compare(const vector<vl_sift_pix*>& descr1, 
	const vector<vl_sift_pix*>& descr2, float thresh) {
	vector<Pair> pairs;
	for (int k1 = 0; k1 < descr1.size(); k1++) {
		float best = FLT_MAX;
		float second_best = best;
		int bestK = -1;

		for (int k2 = 0; k2 < descr2.size(); ++k2) {
			float acc = 0;
			for (int d = 0; d < dimen; ++d) {
				float delta = descr1[k1][d] - descr2[k2][d];
				acc += delta*delta;
				if (acc >= second_best) {
					break;
				}
			}

			if (acc < best) {
				second_best = best;
				best = acc;
				bestK = k2;
			} else if (acc < second_best) {
				second_best = acc;
			}

		}

		if (thresh * second_best > best && bestK != -1) {
			pairs.push_back(Pair(k1, bestK));
		}
	}
	return pairs;
}

void image_stitch(const CImg<float> &img1, const CImg<float> &img2) {
	CImg<float> grayImg1 = get_gray_image(img1);
	CImg<float> grayImg2 = get_gray_image(img2);

	assert(grayImg1.spectrum() == 1);
	assert(grayImg2.spectrum() == 1);


	VlSiftFilt* siftFilt1 = vl_sift_new(img1.width(), img1.height(), noctaves, nlevels, o_min);
	VlSiftFilt* siftFilt2 = vl_sift_new(img2.width(), img2.height(), noctaves, nlevels, o_min);
	vector<vl_sift_pix*> descr1;
	vector<vl_sift_pix*> descr2;
	vector<VlSiftKeypoint> keypoints1;
	vector<VlSiftKeypoint> keypoints2;
	calc_descriptor(descr1, siftFilt1, keypoints1, img1);
	calc_descriptor(descr2, siftFilt2, keypoints2, img2);
	float thresh = 0.5;
	vector<Pair> pairs = compare(descr1, descr2, thresh);

	CImg<float> tmp_img1(img1);
	CImg<float> tmp_img2(img2);
	CImgDisplay cd1(tmp_img1);
	CImgDisplay cd2(tmp_img2);
	int i = 0;
	while (!cd1.is_closed() && !cd2.is_closed() && i < pairs.size()) {
		cd1.wait();
		if (cd1.button() && cd1.mouse_y() >= 0) {
			cout << "(" << keypoints1[pairs[i].k1].x << "," << keypoints1[pairs[i].k1].y << ")\n";
			cout << "(" << keypoints2[pairs[i].k2].x << "," << keypoints2[pairs[i].k2].y << ")\n";
			draw_point(tmp_img1, keypoints1[pairs[i].k1].x, keypoints1[pairs[i].k1].y, 2);
			cd1.display(tmp_img1);
			draw_point(tmp_img2, keypoints2[pairs[i].k2].x, keypoints1[pairs[i].k2].y, 2);
			cd2.display(tmp_img2);	
			i++;	
		}
	}


}

int main(int argc, char **argv) {
	if (argc <= 2) {
		return 0;
	}
	CImg<float> img1(argv[1]);
	CImg<float> img2(argv[2]);
	image_stitch(img1, img2);
}






