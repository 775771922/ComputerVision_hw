#ifndef PAPER_CORECTION_
#define PAPER_CORECTION_

#include <vector>
#include <map>
using namespace std;
#include "CImg.h"
#include "global.h"
using namespace cimg_library;

//#define PAPER_CORECTION_DEBUG

class PaperCorection {
private:
	double rate;
    // 聚类时角度和距离的误差值，单独分两个误差是因为考虑到角度的变化比距离的变化更敏感
	int errorTheta, errorP;
	float sigma;
	int winSize; 
	int firstDerWinSize;
    void draw_result(CImg<float> &img, int theta, int p, int channel);
    CImg<float> erode(CImg<float> &img);
    vector<Position> detect_edge(vector<Position> &pos);
    CImg<float> clip_img(const CImg<float> &srcImg, vector<Position> &s);
    CImg<float> biliinear_interpolation(const CImg<float>& srcImg, int width, int height, double h[9]);
	// 检查像素位置，防止超过图片宽度
	int valueWidth(double srcX, int width);
	// 检查像素位置，防止超过图片高度
	int valueHeight(double srcY, int height);
public:
	PaperCorection(double r, int t, int p);
	PaperCorection(const PaperCorection& p);
    // 对canny处理后的图片检测直线
	vector<Position> detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg,
	            const CImg<float> &cannyImg);
	vector<Position> get_vertexs(vector<Position> &pos);
    vector<Position> get_standard_vertexs(vector<Position> &v, int srcWidth, int srcHeight);
    CImg<float> image_wrap(vector<Position> &v, vector<Position> &s, const CImg<float> &srcImg);
    CImg<float> paper_corection(const CImg<float> &srcImg);
    CImg<float> get_gray_image(const CImg<float> &srcImg);

    void draw_point(CImg<float> &img, const Position &p);
};

#endif