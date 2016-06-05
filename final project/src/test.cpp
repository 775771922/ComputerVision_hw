#include "CImg.h"
#include "paper_corection.h"
#include "number_recognition.h"
#include <vector>
#include <cassert>
using namespace cimg_library;
using namespace std;

int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}
	CImg<float> srcImg(argv[1]);

    double rate = 0.2;
    int errorTheta = 5, errorP = 50; //125;
    PaperCorection paperCorrection(rate, errorTheta, errorP);
    NumberReg<float> numberReg(paperCorrection);

    //paperCorrection.paper_corection(srcImg);

    //numberReg.project_to_x(paperCorrection.paper_corection(srcImg));
    //numberReg.project_to_y(paperCorrection.paper_corection(srcImg));
    numberReg.detect_number_area(srcImg);

}




