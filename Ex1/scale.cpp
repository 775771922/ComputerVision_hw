#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
using namespace cimg_library;
using namespace std;

int valueWidth(double protoX, int width) {
	if (protoX < 0) protoX = 0;
	if (protoX >= width) protoX--;
	return protoX;
}

int valueHeight(double protoY, int height) {
	if (protoY < 0) protoY = 0;
	if (protoY >= height) protoY--;
	return protoY;
}

CImg<unsigned char> nearestScale(CImg<unsigned char>& origin, int width, int height) {
	int origin_w = origin.width();
	int origin_h = origin.height();
	CImg<unsigned char> new_img(width, height, 1, 3, 0);
	double width_scale = origin_w / (double)width;
	double height_scale = origin_h / (double)height;
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			new_img(c, r, 0, 0) = origin((int)(c*width_scale), (int)(r*height_scale), 0, 0);
			new_img(c, r, 0, 1) = origin((int)(c*width_scale), (int)(r*height_scale), 0, 1);
			new_img(c, r, 0, 2) = origin((int)(c*width_scale), (int)(r*height_scale), 0, 2);
		}
	}
	return new_img;
}

CImg<unsigned char> bilinearScale(CImg<unsigned char>& origin, int width, int height) {
	int origin_w = origin.width();
	int origin_h = origin.height();
	CImg<unsigned char> new_img(width, height, 1, 3, 0);
	double width_scale = origin_w / (double)width;
	double height_scale = origin_h / (double)height;
	double protoX, protoY, u, v;
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			protoX = c * width_scale;
			protoY = r * height_scale;
			u = protoX - (int)protoX;
			v = protoY - (int)protoY;
			for (int channel = 0; channel < 3; channel++) {
				new_img(c, r, 0, channel) = (int)((1-u)*(1-v)*origin(valueWidth(protoX, origin_w), valueHeight(protoY, origin_h), 0, channel)
					                  +(1-u)*v*origin(valueWidth(protoX, origin_w), valueHeight(protoY+1, origin_h), 0, channel)
					                  +u*(1-v)*origin(valueWidth(protoX+1, origin_w), valueHeight(protoY, origin_h), 0, channel)
					                  +u*v*origin(valueWidth(protoX+1, origin_w), valueHeight(protoY+1, origin_h), 0, channel));
		    }
		}
	}
	return new_img;
}


int main() {
	string file = "旋转、缩放图像1.bmp";
	//string file = "new_image1.bmp";
	CImg<unsigned char> origin_img(file.c_str());
	CImgDisplay old_disp(origin_img, "origin");

	int width, height;
	cout << "intput new width and height" << endl;
	cin >> width >> height;

	//CImg<unsigned char> new_img = nearestScale(origin_img, width, height);
	CImg<unsigned char> new_img = bilinearScale(origin_img, width, height);
	CImgDisplay new_disp(new_img, "new image");
	while (!new_disp.is_closed()) {
		new_disp.wait();
		if (new_disp.button() && new_disp.mouse_y() >= 0) {
			break;
		}
	}

	return 0;
}