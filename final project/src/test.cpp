#include "CImg.h"
//#include "paper_corection.h"
//#include "number_recognition.h"
#include <vector>
#include <cassert>
#include <iostream>
using namespace cimg_library;
using namespace std;

void read_train_img(CImg<float> &srcImg) {
	cout << "spectrum: " << srcImg.spectrum() << endl;
	cimg_forY(srcImg, y) {
		cimg_forX(srcImg, x) {
			cout << srcImg(x, y, 0, 0) << " ";
		}
		cout << endl;
	}
}

int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}
	CImg<float> srcImg(argv[1]);

	read_train_img(srcImg);

	cout << endl << endl;
    CImg<float> testImg(argv[2]);
	read_train_img(testImg);

    // double rate = 0.2;
    // int errorTheta = 5, errorP = 50; //125;
    // PaperCorection paperCorrection(rate, errorTheta, errorP);
    // NumberReg<float> numberReg(paperCorrection);

    //paperCorrection.paper_corection(srcImg);

    //numberReg.project_to_x(paperCorrection.paper_corection(srcImg));
    //numberReg.project_to_y(paperCorrection.paper_corection(srcImg));
    //numberReg.detect_number_area(srcImg);
    //numberReg.test_project_to_x(srcImg);

    //numberReg.read_number_in(srcImg);
    //numberReg.test_svm(argv[1]);

}




