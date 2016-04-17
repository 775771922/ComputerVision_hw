#include "image_stitch.h"

int main(int argc, char** argv) {
	if (argc <= 2) {
		return 0;
	}

	int noctaves = 5, nlevels = 3, o_min = 0;
	ImageStitch imageStitch(noctaves, nlevels, o_min);
	CImg<float> img1(argv[1]);
	CImg<float> img2(argv[2]);
	img1.display("img1");
	img2.display("img2");
	CImg<float> res = imageStitch.image_stitch(img1, img2);
	res.display();
	res.save_jpeg("res.jpg");

}