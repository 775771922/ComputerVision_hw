#include "CImg.h"
#include "paper_corection.h"
#include "number_recognition.h"
#include <vector>
#include <cassert>
#include <time.h>
#include <cstdio>
#include <iostream>
using namespace cimg_library;
using namespace std;


int main(int argc, char **argv) {
	if (argc == 1) {
		return 0;
	}
	CImg<float> srcImg(argv[1]);

    PaperCorection paperCorrection;
    NumberReg<float> numberReg(paperCorrection);

    time_t start, end;
    start = clock();

    numberReg.read_number_in(srcImg);

    end = clock();
    printf("the running time is : %f\n", double(end-start)/CLOCKS_PER_SEC);



}




