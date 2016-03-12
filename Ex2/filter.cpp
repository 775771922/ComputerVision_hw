#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
using namespace cimg_library;
using namespace std;

int main() {
	//CImg<unsigned char> origin("circle.bmp");
	CImg<float> origin("IMG_20150320_143145.jpg");
	origin.display();
	// CImgDisplay old_disp(origin, "origin");

    int width = origin.width();
    int height = origin.height();

    CImg<unsigned char> new_img(origin);
    for (int i = 1; i < height-1; i++) {
    	for (int j = 1; j < width-1; j++) {
    		for (int channel = 0; channel < 3; channel++) {
    			new_img(j, i, 0, channel) = -1 * origin(j-1, i-1, 0, channel) +
    			                            -1 * origin(j+1, i-1, 0, channel) +
    			                            4 * origin(j, i, 0, channel) +
    			                            -1 * origin(j-1, i+1, 0, channel) +
    			                            -1 * origin(j+1, i+1, 0, channel);

    			// new_img(j, i, 0, channel) = -1 * origin(j-1, i-1, 0, channel) +
    			//                             1 * origin(j+1, i-1, 0, channel) +
    			//                             -1 * origin(j-1, i, 0, channel) +
    			//                             1 * origin(j+1, i, 0, channel) +
    			//                             -1 * origin(j-1, i+1, 0, channel) +
    			//                             1 * origin(j+1, i+1, 0, channel);

    			// new_img(j, i, 0, channel) = (double)1/16 * origin(j-1, i-1, 0, channel) +
    			//             (double)2/16 * origin(j, i-1, 0, channel) +
    			//             (double)1/16 * origin(j+1, i-1, 0, channel) +
    			//             (double)2/16 * origin(j-1, i, 0, channel) +
    			//             (double)4/16 * origin(j, i, 0, channel) +
    			//             (double)2/16 * origin(j+1, i, 0, channel) +
    			//             (double)1/16 * origin(j-1, i+1, 0, channel) +
    			//             (double)2/16 * origin(j, i+1, 0, channel) +
    			//             (double)1/16 * origin(j+1, i+1, 0, channel);
    		}
    	}
    }
    new_img.display();
    new_img.save("new.bmp");
	// while (!old_disp.is_closed()) {
	// 	old_disp.wait();
	// 	if (old_disp.button() && old_disp.mouse_y() >= 0) {
	// 		break;
	// 	}
	// }

	return 0;
}