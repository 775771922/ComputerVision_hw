#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
using namespace cimg_library;
using namespace std;

#define PI 3.1415926

struct Position {
	int x, y;
	Position(int x, int y) {
		this->x = x;
		this->y = y;
	}
};

// 从新图像的像素位置获得原图中对应的像素位置
bool get_origin_pos(int x, int y, int ori_width, int ori_height, double theta, 
	                int& ori_x, int& ori_y) {
	ori_x = x*cos(theta) - y*sin(theta);
	ori_y = x*sin(theta) + y*cos(theta);
	if (ori_x >= (0-ori_width/2) && ori_x <= ori_width/2 && ori_y >= (0-ori_height/2) && ori_y <= ori_height/2) {
		ori_x += ori_width/2;
		ori_y += ori_height/2;
		return true;
	} else {
		return false;
	}
}

// 从新图像的像素位置获得原图中对应的像素位置
bool get_origin_pos(int x, int y, int ori_width, int ori_height, double theta, 
	                double& ori_x, double& ori_y) {
	ori_x = (double)x * cos(theta) - (double)y * sin(theta);
	ori_y = (double)x * sin(theta) + (double)y * cos(theta);
	if (ori_x >= (0-ori_width/2-1) && ori_x <= ori_width/2+1 && ori_y >= (0-ori_height/2-1) && ori_y <= ori_height/2+1) {
		ori_x += ori_width/2;
		ori_y += ori_height/2;
		return true;
	} else {
		return false;
	}
}

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

// 最邻近插值减少锯齿
void nearest_interpolation(CImg<unsigned char>& img, CImg<unsigned char>& origin_img, double theta) {
	int new_width = img.width(), new_height = img.height();
	int ori_width = origin_img.width(), ori_height = origin_img.height();
	int half_w = new_width / 2;
    int half_h = new_height / 2;
    int ori_x, ori_y, u, v;
    for (int r = 0; r < new_height; r++) {
    	for (int c = 0; c < new_width; c++) {
    		if (get_origin_pos(c-half_w, r-half_h, ori_width, ori_height, theta, ori_x, ori_y)) {
    			for (int channel = 0; channel < 3; channel++) {
    				img(c, r, 0, channel) = origin_img(ori_x, ori_y, 0, channel);
    			}
    		}
    	}
    }
}

// 双线性插值减少锯齿
void biliinear_interpolation(CImg<unsigned char>& img, CImg<unsigned char>& origin_img, double theta) {
	int new_width = img.width(), new_height = img.height();
	int ori_width = origin_img.width(), ori_height = origin_img.height();
	int half_w = new_width / 2;
    int half_h = new_height / 2;
    double ori_x, ori_y, u, v;
    for (int r = 0; r < new_height; r++) {
    	for (int c = 0; c < new_width; c++) {
    		if (get_origin_pos(c-half_w, r-half_h, ori_width, ori_height, theta, ori_x, ori_y)) {
    			u = ori_x - (int)ori_x;
    			v = ori_y - (int)ori_y;
    			for (int channel = 0; channel < 3; channel++) {
    				img(c, r, 0, channel) = 
    				    (int)((1-u)*(1-v)*origin_img(valueWidth(ori_x, ori_width), valueHeight(ori_y, ori_height), 0, channel)
    					+(1-u)*v*origin_img(valueWidth(ori_x, ori_width), valueHeight(ori_y+1, ori_height), 0, channel)
    					+u*(1-v)*origin_img(valueWidth(ori_x+1, ori_width), valueHeight(ori_y, ori_height), 0, channel)
    					+u*v*origin_img(valueWidth(ori_x+1, ori_width), valueHeight(ori_y+1, ori_height), 0, channel));
    			}
    		}
    	}
    }
}

CImg<unsigned char> rotate(CImg<unsigned char>& origin_img, double theta) {
	int width = origin_img.width();
    int height = origin_img.height();
    Position lt(0-width/2, 0+height/2), lb(0-width/2, 0-height/2), 
         rt(0+width/2, 0+height/2), rb(0+width/2, 0-height/2);
    Position new_lt((int)(lt.x*cos(theta)+lt.y*sin(theta)), (int)(lt.y*cos(theta)-lt.x*sin(theta))), 
             new_lb((int)(lb.x*cos(theta)+lb.y*sin(theta)), (int)(lb.y*cos(theta)-lb.x*sin(theta))),
             new_rt((int)(rt.x*cos(theta)+rt.y*sin(theta)), (int)(rt.y*cos(theta)-rt.x*sin(theta))), 
             new_rb((int)(rb.x*cos(theta)+rb.y*sin(theta)), (int)(rb.y*cos(theta)-rb.x*sin(theta)));
    int new_width = max(abs(new_rt.x-new_lb.x), abs(new_lt.x-new_rb.x));
    int new_height = max(abs(new_lt.y-new_rb.y), abs(new_lb.y-new_rt.y));

    CImg<unsigned char> new_img(new_width, new_height, 1, 3, 0);

    biliinear_interpolation(new_img, origin_img, theta);
    //nearest_interpolation(new_img, origin_img, theta);

    return new_img;

}

int main() {

	string file = "旋转、缩放图像1.bmp";

	CImg<unsigned char> origin_img(file.c_str());

    char* file_name = new char[20];
    for (int rotate_times = 1; rotate_times <= 6; rotate_times++) {
    	CImg<unsigned char> new_img = rotate(origin_img, PI*rotate_times/6);
    	memset(file_name, '\0', sizeof(char)*20);
        strcpy(file_name, "rotate_image");
	    file_name[strlen("rotate_image")] = rotate_times + '0';
	    strcpy(file_name+strlen(file_name), ".bmp");
	    new_img.save_bmp(file_name);
    }
    delete [] file_name;

	return 0;
}



