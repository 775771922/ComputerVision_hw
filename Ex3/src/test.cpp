#include "CImg.h"
#include "gaussian.h"
#include "canny.h"
#include "hough.h"
#include "paper_corection.h"
#include <vector>
#include <cassert>
using namespace cimg_library;
using namespace std;

int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}
	CImg<float> srcImg(argv[1]);

	Canny canny(1.5, 3, 3);
	CImg<float> cannyDetectImg = canny.detect_edge(srcImg);
    cannyDetectImg.save_jpeg("canny.jpg");
	
	int width = cannyDetectImg.width();
	int height = cannyDetectImg.height();
	int diagonal = sqrt(width*width + height*height);

	HoughTransform hough(360, diagonal);
	hough.draw_hough_space(cannyDetectImg);

    double rate = 0.4;
    int errorTheta = 10, errorP = 125;
    PaperCorection dect(rate, errorTheta, errorP);

    //vector<Position> result = dect.detect_edge(hough.get_hough_space(), srcImg, cannyDetectImg);
    //assert(result.size() == 4);
    dect.get_vertexs(hough.get_hough_space(), srcImg, cannyDetectImg);



}




