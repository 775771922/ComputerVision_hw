#include "image_stitch.h"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <float.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
using namespace cv;


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


ImageStitch::ImageStitch(int octaves, int levels, int o_min)
           : noctaves(octaves), nlevels(levels), o_min(o_min) {
}

CImg<float> ImageStitch::image_stitch(const CImg<float> &img1, const CImg<float> &img2) {
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

    #ifdef Image_Stitch_DEBUG
    cout << "begin ransac" << endl;
    #endif 

	double h[9];
	float epsilon = 10.0;
	ransac(h, pairs, keypoints1, keypoints2, epsilon);

	// vl_sift_delete(siftFilt1);
	// vl_sift_delete(siftFilt2);
	descr1.clear();
	descr2.clear();

	#ifdef Image_Stitch_DEBUG
	cout << "sitf detect finish" << endl;
	#endif

	return image_stitch(img1, img2, h);
}

void ImageStitch::calc_descriptor(vector<vl_sift_pix*>& descr, VlSiftFilt* siftFilt, 
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

vector<Pair> ImageStitch::compare(const vector<vl_sift_pix*>& descr1, 
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

double ImageStitch::calc_euclidean_distance(vl_sift_pix* descr1, vl_sift_pix* descr2) {
	double acc = 0;
	for (int i = 0; i < dimen; i++) {
		double delta = descr1[i] - descr2[i];
		acc += delta*delta;
	}
	return acc;
}

void ImageStitch::ransac(double h[9], vector<Pair>& pairs, vector<VlSiftKeypoint> &keypoints1, 
	vector<VlSiftKeypoint>& keypoints2, float epsilon) {
	int loop_times = 10;
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

		#ifdef Image_Stitch_DEBUG
		cout << "loop_times====>" << loop_times << endl;
		#endif
	}
	recomputer_least_squares(keypoints1, keypoints2, h);
}

vector<Pair> ImageStitch::randomly_select(vector<Pair> &pairs) {
	vector<Pair> ret;
	bool flag[pairs.size()];
	memset(flag, 0, sizeof(flag));
	for (int i = 0; i < 4; i++) {
		while (true) {
			srand((unsigned)time(0));
			int randomIndex = rand() % pairs.size();
			if (flag[randomIndex] == false) {
				flag[randomIndex] = true;
				ret.push_back(pairs[randomIndex]);

                #ifdef Image_Stitch_DEBUG
                cout << "random number====>" << randomIndex << endl;
                #endif

				break;
			}
		}
	}
	return ret;
}

void ImageStitch::calc_homography(vector<Pair> &randomPairs, vector<VlSiftKeypoint> &keypoints1,
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

int ImageStitch::calc_inliers(vector<Pair> &pairs, vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double tempH[9], float epsilon) {
	int inliers = 0;
	for (int i = 0; i < pairs.size(); i++) {
		VlSiftKeypoint srcP = keypoints1[pairs[i].k1];
		VlSiftKeypoint destP = keypoints2[pairs[i].k2];
		double x, y;
		x = srcP.x*tempH[0] + srcP.y*tempH[1] + tempH[2];
		y = srcP.x*tempH[3] + srcP.y*tempH[4] + tempH[5];
		if ((x-destP.x)*(x-destP.x)+(y-destP.y)*(y-destP.y) < epsilon) {
			++inliers;
		}
	}
	return inliers;
}
    
void ImageStitch::recomputer_least_squares(vector<VlSiftKeypoint> &keypoints1, 
    vector<VlSiftKeypoint> &keypoints2, double h[9]) {

}

CImg<float> ImageStitch::image_stitch(const CImg<float> &img1, const CImg<float> &img2, 
	double h[]) {

    assert(img1.spectrum() == img2.spectrum());

    Mat m = Mat(3, 3, CV_64FC1, h);
    Mat inv = m.inv();

    int width2 = img2.width()-1, height2 = img2.height()-1;
    Point2f lt(inv.at<double>(0,2), inv.at<double>(1,2)),
            lb(inv.at<double>(0,0)*0+inv.at<double>(0,1)*height2+inv.at<double>(0,2),
            	inv.at<double>(1,0)*0+inv.at<double>(1,1)*height2+inv.at<double>(1,2)),
            rt(inv.at<double>(0,0)*width2+inv.at<double>(0,1)*0+inv.at<double>(0,2),
            	inv.at<double>(1,0)*width2+inv.at<double>(1,1)*0+inv.at<double>(1,2)),
            rb(inv.at<double>(0,0)*width2+inv.at<double>(0,1)*height2+inv.at<double>(0,2),
            	inv.at<double>(1,0)*width2+inv.at<double>(1,1)*height2+inv.at<double>(1,2));

    #ifdef Image_Stitch_DEBUG
    cout << "inv===>\n" << inv << endl;
    cout << "lt====>(" << lt.x << "," << lt.y << ")\n";
    cout << "lb====>(" << lb.x << "," << lb.y << ")\n";
    cout << "rt====>(" << rt.x << "," << rt.y << ")\n";
    cout << "rb====>(" << rb.x << "," << rb.y << ")\n";
    #endif

    int maxWidth = max(lt.x, max(lb.x, max(rt.x, rb.x)));

	CImg<float> ret(maxWidth, img1.height(), 1, img1.spectrum(), 0);

	for (int i = 0; i < ret.width(); i++) {
		for (int j = 0; j < ret.height(); j++) {
			double x = i*h[0]+j*h[1]+h[2], y = i*h[3]+j*h[4]+h[5];
			if (x >= 0 && x < img2.width() && y >= 0 && y < img2.height()) {
				double u = x - (int)x, v = y - (int)y;
				for (int channel = 0; channel < img1.spectrum(); channel++) {
                    ret(i, j, 0, channel) = 
                        (int)((1-u)*(1-v)*img2(valueWidth(x, img2.width()), valueHeight(y, img2.height()), 0, channel)
                        +(1-u)*v*img2(valueWidth(x, img2.width()), valueHeight(y+1, img2.height()), 0, channel)
                        +u*(1-v)*img2(valueWidth(x+1, img2.width()), valueHeight(y, img2.height()), 0, channel)
                        +u*v*img2(valueWidth(x+1, img2.width()), valueHeight(y+1, img2.height()), 0, channel));
				}
			} else if (i >= 0 && i <img1.width() && j >= 0 && j < img1.height()) {
				for (int channel = 0; channel < img1.spectrum(); channel++) {
					ret(i, j, 0, channel) = img1(i, j, 0, channel);
				}
			}
		}
	}
	return ret;
}

// 检查像素位置，防止超过图片宽度
int ImageStitch::valueWidth(double srcX, int width) {
    if (srcX < 0) srcX = 0;
    if (srcX >= width) srcX--;
    return srcX;
}

// 检查像素位置，防止超过图片高度
int ImageStitch::valueHeight(double srcY, int height) {
    if (srcY < 0) srcY = 0;
    if (srcY >= height) srcY--;
    return srcY;
}

CImg<float> ImageStitch::get_gray_image(const CImg<float> &srcImg) {
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







