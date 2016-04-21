#include "image_stitch.h"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <memory.h>
#include <float.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <list>
#include <map>
using namespace cv;

int num = 0;

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

CImg<float> ImageStitch::image_stitch(const vector<CImg<float> > &imgs) {
	srand((unsigned)time(0));
	vector<CImg<float> > merge(imgs);
	while (merge.size() != 1) {
		assert(merge.size() > 0);
		merge = image_merge(merge);

		#ifdef Image_Stitch_DEBUG
		//printf("merge====> %lu\n", merge.size());
		for (int i = 0; i < merge.size(); i++) {
			merge[i].display();
			char c[2] = "0";
			c[0] += num++;
			string name = "merge" + string(c) + ".jpg";
			merge[i].save_jpeg(name.c_str());
		}
		#endif
	}
	return merge[0];
}

vector<CImg<float> > ImageStitch::image_merge(vector<CImg<float> > &imgs) {
	bool *isMatched = new bool[imgs.size()];
	memset(isMatched, 0, sizeof(bool)*imgs.size());
	vector<CImg<float> > res;
	vector<ImgFeature> imgsFeature;
	for (int i = 0; i < imgs.size(); i++) {
		calc_img_feature(imgsFeature, imgs[i]);
	}
	
	while (!all_matched(isMatched, imgs.size())) {
		int rIndex = random_index(isMatched, imgs.size());
		
		isMatched[rIndex] = true;
		vector<Pair> pointPairs;
		int neighbor = find_nearest_neighbor(rIndex, imgs, isMatched, imgsFeature, pointPairs);

		#ifdef Image_Stitch_DEBUG
		//char c;
		//cin >> c;
		cout << "rIndex====>" << rIndex << " neighbor====>" << neighbor << endl;
		for (int i = 0; i < imgs.size(); i++) {
			cout << isMatched[i] << " ";
		}
		cout << endl;
		#endif

		if (neighbor != -1) {
			CImg<float> stitchImg = image_stitch(imgs[rIndex], imgs[neighbor], imgsFeature[rIndex], 
				imgsFeature[neighbor], pointPairs);
			res.push_back(stitchImg);
	    } else {
	    	res.push_back(imgs[rIndex]);
	    }
	}

	delete [] isMatched;
	imgs.clear();
	return res;
}

void ImageStitch::calc_img_feature(vector<ImgFeature> &imgsFeature, const CImg<float> &img) {
	CImg<float> tempImg(img);
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
	delete [] imageData;
	imageData = NULL;
	siftFilt = NULL;
}

bool ImageStitch::all_matched(bool* isMatched, int size) {
	for (int i = 0; i < size; i++) {
		if (!isMatched[i]) {
			return false;
		}
	}
	return true;
}

int ImageStitch::random_index(bool *isMatched, int size) {
	//srand((unsigned)time(0));
	while (true) {
		//srand((unsigned)time(0));
		int randomIndex = rand() % size;
		if (isMatched[randomIndex] == false) {
			return randomIndex;
		}
	}
}

int ImageStitch::find_nearest_neighbor(int cur, vector<CImg<float> > &imgs, bool *isMatched, 
	vector<ImgFeature> &imgsFeature, vector<Pair> &pointPairs) {
	
	for (int i = 0; i < imgs.size(); i++) {
		if (isMatched[i] == false) {
			//vector<Pair> pairs;
			VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, dimen, 1, VlDistanceL1);

            // 为当前image构建kd树
            vector<vl_sift_pix*> descr = imgsFeature[cur].descr;
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

            // 遍历待搜索的图片特征，找到最匹配的点对
			descr = imgsFeature[i].descr;
			for (int j = 0; j < descr.size(); j++) {
				int nvisited = vl_kdforestsearcher_query(searcher, neighbors, 2, descr[j]);
				if (neighbors[0].distance < neighbors[1].distance * 0.5) {
					// neighbors[0].index表示kd树中最佳匹配点的位置
					pointPairs.push_back(Pair(neighbors[0].index, j));
				}
			}

            // 匹配点对超过20个才认为两张图有公共部分
			if (pointPairs.size() > 20) {
				isMatched[i] = true;
				return i;
			}

		}
	}
	return -1;
}

CImg<float> ImageStitch::image_stitch(CImg<float> &l, CImg<float> &r, ImgFeature &lf, ImgFeature &rf, 
	vector<Pair> &pairs) {
	double h[9];
	float epsilon = 6.0;
	ransac(h, pairs, lf.keypoints, rf.keypoints, epsilon);
	return image_stitch(l, r, h);
}


CImg<float> ImageStitch::image_stitch(const CImg<float> &img1, const CImg<float> &img2) {
	srand((unsigned)time(0));
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

vector<Pair> ImageStitch::randomly_select(vector<Pair> &pairs) {
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
		if ((x-destP.x)*(x-destP.x)+(y-destP.y)*(y-destP.y) < epsilon*epsilon) {
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

    int maxX = max(lt.x, max(lb.x, max(rt.x, rb.x)));
    int minX = min(lt.x, min(lb.x, min(rt.x, rb.x)));

    #ifdef Image_Stitch_DEBUG
    cout << "maxX===>" << maxX << endl;
    cout << "minX===>" << minX << endl;
    #endif

    int offsetX;
    CImg<float> ret;
    // img2映射在img1的左边
    if (minX < 0) {
    	offsetX = 0 - minX;
    	ret.assign(abs(minX)+img1.width(), img1.height(), 1, img1.spectrum(), 0);
    } else if (maxX > img1.width()) {  // img2映射在img1的右边
    	offsetX = 0;
    	ret.assign(maxX, img1.height(), 1, img1.spectrum(), 0);
    } else {
    	assert(false); // 不应该发生这种情况
    }

	for (int i = 0; i < ret.width(); i++) {
		for (int j = 0; j < ret.height(); j++) {
			int ii = i - offsetX;
			double x = ii*h[0]+j*h[1]+h[2], y = ii*h[3]+j*h[4]+h[5];
			if (ii >= 0 && ii < img1.width() && j >= 0 && j < img1.height()) {
				if (x >= 0 && x < img2.width() && y >= 0 && y < img2.height()) {  // 公共区域
				    double u = x - (int)x, v = y - (int)y;
				    for (int channel = 0; channel < img1.spectrum(); channel++) {
				    	double colorOfImg2 = (1-u)*(1-v)*img2(valueWidth(x, img2.width()), valueHeight(y, img2.height()), 0, channel)
                                  +(1-u)*v*img2(valueWidth(x, img2.width()), valueHeight(y+1, img2.height()), 0, channel)
                                  +u*(1-v)*img2(valueWidth(x+1, img2.width()), valueHeight(y, img2.height()), 0, channel)
                                  +u*v*img2(valueWidth(x+1, img2.width()), valueHeight(y+1, img2.height()), 0, channel);
                        if (abs(colorOfImg2-0.0) < 0.0001) {
                        	ret(i, j, 0, channel) = img1(ii, j, 0, channel); 
                        } else if (img1(ii, j, 0, channel) == 0) {
                        	ret(i, j, 0, channel) = (int)colorOfImg2;
                        } else {
                        	ret(i, j, 0, channel) = (int)(colorOfImg2*0.5+img1(ii, j, 0, channel)*0.5);
                        	//ret(i, j, 0, channel) = img1(ii, j, 0, channel); 
                        }
                        
				    }
				} else {
				    for (int channel = 0; channel < img1.spectrum(); channel++) {
					    ret(i, j, 0, channel) = img1(ii, j, 0, channel);    				
			        }
				}
		    } else if (x >= 0 && x < img2.width() && y >= 0 && y < img2.height()) {
				double u = x - (int)x, v = y - (int)y;
    			for (int channel = 0; channel < img1.spectrum(); channel++) {
                    ret(i, j, 0, channel) = 
                        (int)((1-u)*(1-v)*img2(valueWidth(x, img2.width()), valueHeight(y, img2.height()), 0, channel)
                        +(1-u)*v*img2(valueWidth(x, img2.width()), valueHeight(y+1, img2.height()), 0, channel)
                        +u*(1-v)*img2(valueWidth(x+1, img2.width()), valueHeight(y, img2.height()), 0, channel)
                        +u*v*img2(valueWidth(x+1, img2.width()), valueHeight(y+1, img2.height()), 0, channel));    				
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









