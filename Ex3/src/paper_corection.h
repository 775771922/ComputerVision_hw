#ifndef PAPER_CORECTION_
#define PAPER_CORECTION_

#include <vector>
#include <map>
using namespace std;
#include "CImg.h"
#include "global.h"
using namespace cimg_library;

#define PAPER_CORECTION_DEBUG

class PaperCorection {
private:
	double rate;
    // 聚类时角度和距离的误差值，单独分两个误差是因为考虑到角度的变化比距离的变化更敏感
	int errorTheta, errorP;
	// 寻找直线时，根据垂直的角度以及相距的距离做取舍
	int thetaError, disError;
	void draw_line(CImg<float> &img, int theta, int p);
    void print_map(multimap<int, Position> &cluster);
    void draw_result(CImg<float> &img, int theta, int p, int channel);
    void draw_point(CImg<float> &img, const Position &p);
    vector<Position> detect_edge(vector<Position> &pos);
    CImg<float> adjust_orientation(const CImg<float> &srcImg, vector<Position> &v, bool isVertical);
    CImg<float> clip_img(const CImg<float> &srcImg, vector<Position> &s);
    CImg<float> biliinear_interpolation(const CImg<float>& srcImg, int width, int height, double h[8]);
	// 检查像素位置，防止超过图片宽度
	int valueWidth(double srcX, int width);
	// 检查像素位置，防止超过图片高度
	int valueHeight(double srcY, int height);
public:
	PaperCorection(double r, int t, int p);
    // 对canny处理后的图片检测直线
	vector<Position> detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg,
	            const CImg<float> &cannyImg);
	vector<Position> get_vertexs(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
	            const CImg<float> &cannyImg);
    vector<Position> get_standard_vertexs(vector<Position> &v, int srcWidth, int srcHeight);
    CImg<float> image_wrap(vector<Position> &v, vector<Position> &s, const CImg<float> &srcImg);
    CImg<float> paper_corection(const CImg<float> &houghSpace, const CImg<float> &srcImg, const CImg<float> &cannyImg);
};

#endif