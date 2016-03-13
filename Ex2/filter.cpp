#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
using namespace cimg_library;
using namespace std;

int main(int argc, char ** argv) {
	CImg<unsigned char> origin(argv[1]);
	//CImg<float> origin("IMG_20150320_143145.jpg");
	//origin.display();
	// CImgDisplay old_disp(origin, "origin");

    //double mask[3][3] = {{1/16, 1/8, 1/16}, {1/8, 1/4, 1/8}, {1/16, 1/8, 1/16}};
    double mask[3][3] = {{0.0947416, 0.118318, 0.0947416}, {0.118318, 0.147761, 0.118318}, {0.0947416, 0.118318, 0.0947416}};

    int width = origin.width();
    int height = origin.height();

    CImg<unsigned char> new_img(origin);
    for (int i = 1; i < height-1; i++) {
    	for (int j = 1; j < width-1; j++) {
    		for (int channel = 0; channel < 3; channel++) {
    			// new_img(j, i, 0, channel) = -1 * origin(j-1, i-1, 0, channel) +
    			//                             -1 * origin(j+1, i-1, 0, channel) +
    			//                             4 * origin(j, i, 0, channel) +
    			//                             -1 * origin(j-1, i+1, 0, channel) +
    			//                             -1 * origin(j+1, i+1, 0, channel);

                // new_img(j, i, 0, channel) = -1 * origin(j-1, i-1, 0, channel) +
                //             1 * origin(j+1, i-1, 0, channel) +
                //             -2 * origin(j-1, i, 0, channel) +
                //             2 * origin(j+1, i, 0, channel) +
                //             -1 * origin(j-1, i+1, 0, channel) +
                //             1 * origin(j+1, i+1, 0, channel);

    			// new_img(j, i, 0, channel) = -1 * origin(j-1, i-1, 0, channel) +
    			//                             1 * origin(j+1, i-1, 0, channel) +
    			//                             -1 * origin(j-1, i, 0, channel) +
    			//                             1 * origin(j+1, i, 0, channel) +
    			//                             -1 * origin(j-1, i+1, 0, channel) +
    			//                             1 * origin(j+1, i+1, 0, channel);

    			new_img(j, i, 0, channel) = mask[0][0] * (double)origin(j-1, i-1, 0, channel) +
    			            mask[0][1] * (double)origin(j, i-1, 0, channel) +
    			            mask[0][2] * (double)origin(j+1, i-1, 0, channel) +
    			            mask[1][0] * (double)origin(j-1, i, 0, channel) +
    			            mask[1][1] * (double)origin(j, i, 0, channel) +
    			            mask[1][2] * (double)origin(j+1, i, 0, channel) +
    			            mask[2][0] * (double)origin(j-1, i+1, 0, channel) +
    			            mask[2][1] * (double)origin(j, i+1, 0, channel) +
    			            mask[2][2] * (double)origin(j+1, i+1, 0, channel);
                //cout << new_img(j, i, 0, channel) << " ";
    		}
    	}
    }
    new_img.display();
    new_img.save("GaussianBlur.jpg");

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            for (int channel = 0; channel < 3; channel++) {
                origin(i, j, 0, channel) -= new_img(i, j, 0, channel);
            }
        }
    }

    origin.save("sharpen.jpg");


	return 0;
}