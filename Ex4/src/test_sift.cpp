#include <iostream>
#include <fstream>
#include <cassert>
#include <cstdio>
#include <memory.h>
#include "CImg.h"
#include "DrawOP.h"
using namespace cimg_library;
using namespace std;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
}

CImg<float> get_gray_image(const CImg<float> &srcImg);
void draw_point(CImg<float> &img, int x, int y, double circle);

bool flag[1000][1000];

int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}

    memset(flag, 0, sizeof(flag));
	VL_PRINT("Hello World!\n");
	CImg<float> srcImg(argv[1]);
	srcImg = get_gray_image(srcImg);
	srcImg.display();

	int noctaves = 4, nlevels = 2, o_min = 0;
	vl_sift_pix *imageData = new vl_sift_pix[srcImg.width()*srcImg.height()];

	for (int i = 0; i < srcImg.width(); i++) {
		for (int j = 0; j < srcImg.height(); j++) {
			imageData[j*srcImg.width()+i] = srcImg(i, j, 0);
		}
	}

    

	VlSiftFilt *siftFilt = NULL;
	siftFilt = vl_sift_new(srcImg.width(), srcImg.height(), noctaves, nlevels, o_min);
	if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
		while (true) {
			cout << "new siftFilt\n";
			vl_sift_detect(siftFilt);
			// 遍历并绘制每个点
			VlSiftKeypoint *pKeyPoint = siftFilt->keys;
			for (int i = 0; i < siftFilt->nkeys; i++) {
				VlSiftKeypoint tempKeyPoint = *pKeyPoint;
				pKeyPoint++;
				//draw_point(srcImg, tempKeyPoint.x, tempKeyPoint.y, tempKeyPoint.sigma/2);
				assert(flag[(int)tempKeyPoint.x][(int)tempKeyPoint.y] == false);
				flag[(int)tempKeyPoint.x][(int)tempKeyPoint.y] = true;
				cout << "(" << tempKeyPoint.x << "," << tempKeyPoint.y << ")" << endl;
				// 计算并遍历每个点的方向
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);
				//cout << "angleCount====>" << angleCount << endl;
				for (int j = 0; j < angleCount; j++) {
					double tempAngle = angles[j];
					//printf("%d: %f\n", j, tempAngle);
					vl_sift_pix* descriptors = new vl_sift_pix[128];
					vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, tempAngle);
					int k = 0;
					// while (k < 128) {
					// 	printf("%d: %f; ", k, descriptors[k]);
					// 	k++;
					// }
					// printf("\n");
					delete [] descriptors;
					descriptors = NULL;
				}
			}
			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}
	vl_sift_delete(siftFilt);

	delete [] imageData;
	imageData = NULL;
	srcImg.display();
	srcImg.save_jpeg("result.jpg");
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