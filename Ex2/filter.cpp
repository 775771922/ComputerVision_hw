#include "CImg.h"
#include "hough.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <cstring>
#include <string>
#include <cassert>
#include <algorithm>
#include <vector>
using namespace cimg_library;
using namespace std;

#define PI 3.1415926
const int GAUSSIAN_WIN_SIZE = 3;
const double sigma = 1.5;

struct Position {
    int x, y, sum;
    Position(int x, int y, int sum) {
        this->x = x;
        this->y = y;
        this->sum = sum;
    }
    // bool operator<(const Position& a, const Position &b) {
    //     return a.sum >= b.sum;
    // }
};

bool cmp(const Position& a, const Position& b) {
    return a.sum > b.sum;
}

void non_maximun_suppression(CImg<float> &out, CImg<float> &gradientImg, int w, int h, double tanTheta);
void gaussian_blur(CImg<float> &target);
CImg<float> get_gray_image(CImg<float> &srcImg);
CImg<float> hysteresis_thresholding(const CImg<float> &target, int high, int low);
CImg<float> draw_hough_space(const CImg<float> &srcImg);
void draw_line(CImg<float> &img, int theta, int p);

int main(int argc, char ** argv) {

	CImg<float> srcImg(argv[1]);

    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();

    CImg<float> grayImg = get_gray_image(srcImg);

    // 求x方向的卷积
    float maskX[GAUSSIAN_WIN_SIZE*2+1];

    for (int x = -GAUSSIAN_WIN_SIZE; x <= GAUSSIAN_WIN_SIZE; x++) {
        float r = x*x;
        float sum = -(r/(2*sigma*sigma));
        maskX[x+GAUSSIAN_WIN_SIZE] = -((double)x/(sigma*sigma))*exp(sum);
    }

    // 求y方向的卷积
    float maskY[GAUSSIAN_WIN_SIZE*2+1];
    for (int y = -GAUSSIAN_WIN_SIZE; y <= GAUSSIAN_WIN_SIZE; y++) {
        float r = y*y;
        float sum = -(r/(2*sigma*sigma));
        maskY[y+GAUSSIAN_WIN_SIZE] = -((double)y/(sigma*sigma))*exp(sum);
    }


    CImg<float> grayXImg(width, height, 1, 1, 0);
    for (int i = GAUSSIAN_WIN_SIZE; i < width-GAUSSIAN_WIN_SIZE; i++) {
    	for (int j = 0; j < height; j++) {
    		for (int channel = 0; channel < 1; channel++) {
                for (int win = -GAUSSIAN_WIN_SIZE; win <= GAUSSIAN_WIN_SIZE; win++) {
                    grayXImg(i, j, 0, channel) += grayImg(i+win, j, 0, channel) * maskX[win+GAUSSIAN_WIN_SIZE]; 
                }
    		}
    	}
    }
    grayXImg.save("GaussianGradBlur_X.bmp");


    CImg<float> grayYImg(width, height, 1, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = GAUSSIAN_WIN_SIZE; j < height-GAUSSIAN_WIN_SIZE; j++) {
            for (int channel = 0; channel < 1; channel++) {
                for (int win = -GAUSSIAN_WIN_SIZE; win <= GAUSSIAN_WIN_SIZE; win++) {
                    grayYImg(i, j, 0, channel) += grayImg(i, j+win, 0, channel) * maskY[win+GAUSSIAN_WIN_SIZE];
                }
            }
        }
    }
    grayYImg.save("GaussianGradBlur_Y.bmp");

    CImg<float> grayXYImg(width, height, 1, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int channel = 0; channel < 1; channel++) {

                grayXYImg(i, j, 0, channel) = sqrt(grayXImg(i,j,0,channel)*grayXImg(i,j,0,channel)
                                             +grayYImg(i,j,0,channel)*grayYImg(i,j,0,channel));
                if (grayXYImg(i, j, 0, channel) < 25) {
                    grayXYImg(i, j, 0, channel) = 0;
                }
            }
        }
    }
    grayXYImg.save("GaussianGrad.bmp");

    CImg<float> nonMaximunImg(width, height, 1, 1);
    for (int i = 1; i < width-1; i++) {
        for (int j = 1; j < height-1; j++) {
            double tanTheta;
            // 90度
            if (grayXImg(i, j, 0, 0) == 0) {
                tanTheta = 2222;
            } else {
                tanTheta = (double)grayYImg(i, j, 0, 0) / grayXImg(i, j, 0, 0);
            }
            non_maximun_suppression(nonMaximunImg, grayXYImg, i, j, tanTheta);
        }
    }
    //nonMaximunImg.display("nonMaximunImg");
    nonMaximunImg.save("nonMaximunSuppression.bmp");

    CImg<float> cannyImg = hysteresis_thresholding(nonMaximunImg, 35, 15);
    cannyImg.save("cannyImg.bmp");

    int nmax = sqrt(width*width + height*height);
    //cout << nmax << endl;
    HoughTransform hough(360, nmax);
    hough.draw_hough_space(cannyImg);

    CImg<float> houghSpace = hough.get_hough_space();
    houghSpace.save("houghSpace.bmp");

    vector<Position> pos;
    for (int i = 0; i < houghSpace.width(); i++) {
        for (int j = 0; j < houghSpace.height(); j++) {
            if (houghSpace(i, j, 0, 0) <= 0) {
                continue;
            }
            pos.push_back(Position(i, j, houghSpace(i, j, 0, 0)));
        }
    }
    cout << "pos.size() =====> " << pos.size() << endl;
    sort(pos.begin(), pos.end(), cmp);

    CImg<float> result(width, height, 1, 1, 0);
    for (int i = 0; i < 25; i++) {
        Position p = pos[i];
        draw_line(result, p.x, p.y);
    }
    result.save("result.bmp");


	return 0;
}

void non_maximun_suppression(CImg<float> &out, CImg<float> &gradientImg, int w, int h, double tanTheta) {
    if (tanTheta > -0.4142 && tanTheta <= 0.4142) {
        if (gradientImg(w, h, 0, 0) >= gradientImg(w-1, h, 0, 0) && gradientImg(w, h, 0, 0) >= gradientImg(w+1,h,0,0)) {
            out(w, h, 0, 0) = gradientImg(w, h, 0, 0);
        } else {
            out(w, h, 0, 0) = 0;
        }
    } else if (tanTheta > 0.4142 && tanTheta < 2.4142) {
        if (gradientImg(w, h, 0, 0) >= gradientImg(w-1, h+1, 0, 0) && gradientImg(w, h, 0, 0) >= gradientImg(w+1,h-1,0,0)) {
            out(w, h, 0, 0) = gradientImg(w, h, 0, 0);
        } else {
            out(w, h, 0, 0) = 0;
        }
    } else if (abs(tanTheta) >= 2.4142) {
        if (gradientImg(w, h, 0, 0) >= gradientImg(w, h+1, 0, 0) && gradientImg(w, h, 0, 0) >= gradientImg(w,h-1,0,0)) {
            out(w, h, 0, 0) = gradientImg(w, h, 0, 0);
        } else {
            out(w, h, 0, 0) = 0;
        }
    } else {
        if (gradientImg(w, h, 0, 0) >= gradientImg(w-1, h-1, 0, 0) && gradientImg(w, h, 0, 0) >= gradientImg(w+1,h+1,0,0)) {
            out(w, h, 0, 0) = gradientImg(w, h, 0, 0);
        } else {
            out(w, h, 0, 0) = 0;
        }

    }
}

CImg<float> get_gray_image(CImg<float> &srcImg) {
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


CImg<float> hysteresis_thresholding(const CImg<float> &target, int high, int low) {
    int width = target.width();
    int height = target.height();
    int depth = target.depth();
    CImg<float> ret(width, height, depth, 1, 0);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (target(i, j, 0, 0) > high) {
                ret(i, j, 0, 0) = 128;
            } else if (target(i, j, 0, 0) >= low) {
                bool hasHighNeighbor = false;
                for (int neighborX = -1; neighborX <= 1; neighborX++) {
                    for (int neighborY = -1; neighborY <= 1; neighborY++) {
                        if (hasHighNeighbor) {
                            break;
                        }
                        int tempX = i+neighborX;
                        int tempY = j+neighborY;
                        tempX = tempX < 0 ? 0 : tempX;
                        tempX = tempX >= width ? width-1 : tempX;
                        tempY = tempY < 0 ? 0 : tempY;
                        tempY = tempY >= height ? height-1 : tempY;
                        if (target(tempX, tempY, 0, 0) > high) {
                            ret(i, j, 0, 0) = 128;
                            hasHighNeighbor = true;
                        }   
                    }
                }
            } else {
                //ret(i, j, 0, 0) = 0;
            }
        }
    }
    return ret;
}


void draw_line(CImg<float> &img, int theta, int p) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double temp_p = (double)i * cos(PI*(theta)/180) + (double)j * sin(PI*(theta)/180);
            //cout << temp_p << endl;
            if (p >= temp_p-0.5 && p <= temp_p+0.5) {
                //cout << "i====>" << i << " j=====>" << j << endl;
                for (int channel = 0; channel < 1; channel++) {
                    img(i, j, 0, channel) = 255;
                }
            }
        }
    }
}










