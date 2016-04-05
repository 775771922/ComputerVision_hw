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
	void draw_line(CImg<float> &img, int theta, int p);
    void print_map(multimap<int, Position> &cluster);
    void draw_result(CImg<float> &img, int theta, int p, int channel);
    vector<Position> detect_edge(vector<Position> &pos);
public:
	PaperCorection(double r, int t, int p);
    // 对canny处理后的图片检测直线
	vector<Position> detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg,
	            const CImg<float> &cannyImg);
	vector<Position> get_vertexs(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
	            const CImg<float> &cannyImg);

};

#endif