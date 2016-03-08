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

bool get_origin_pos(int x, int y, int ori_width, int ori_height, double theta, Position &pos) {
	int ori_x = x*cos(theta) - y*sin(theta);
	int ori_y = x*sin(theta) + y*cos(theta);
	if (ori_x >= (0-ori_width/2) && ori_x <= ori_width/2 && ori_y >= (0-ori_height/2) && ori_y <= ori_height/2) {
		pos.x = ori_x + ori_width/2;
		pos.y = ori_y + ori_height/2;
		return true;
	} else {
		return false;
	}
}

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

int main() {

	string file = "旋转、缩放图像1.bmp";

	CImg<unsigned char> origin_img(file.c_str());
	CImgDisplay old_disp(origin_img,"origin");

    int width = origin_img.width();
    int height = origin_img.height();
    Position lt(0-width/2, 0+height/2), lb(0-width/2, 0-height/2), 
             rt(0+width/2, 0+height/2), rb(0+width/2, 0-height/2);
    int rotate_times = 1;
    
    char* filename = new char[20];
    while (rotate_times <= 6) {
    	memset(filename, '\0', sizeof(char)*20);

        double theta = (-1*PI*rotate_times) / 6;  // -1，顺时针旋转

	    Position new_lt((int)(lt.x*cos(theta)+lt.y*sin(theta)), (int)(lt.y*cos(theta)-lt.x*sin(theta))), 
	             new_lb((int)(lb.x*cos(theta)+lb.y*sin(theta)), (int)(lb.y*cos(theta)-lb.x*sin(theta))),
	             new_rt((int)(rt.x*cos(theta)+rt.y*sin(theta)), (int)(rt.y*cos(theta)-rt.x*sin(theta))), 
	             new_rb((int)(rb.x*cos(theta)+rb.y*sin(theta)), (int)(rb.y*cos(theta)-rb.x*sin(theta)));
	    int new_width = max(abs(new_rt.x-new_lb.x), abs(new_lt.x-new_rb.x));
	    int new_height = max(abs(new_lt.y-new_rb.y), abs(new_lb.y-new_rt.y));

	    int delta_w = new_width / 2;
	    int delta_h = new_height / 2;

	    CImg<unsigned char> new_img(new_width, new_height, 1, 3, 0);
	   
	    CImgDisplay new_disp(new_width, new_height, "new image");

	    // for (int r = 0; r < new_height; r++) {
	    // 	for (int c = 0; c < new_width; c++) {
	    // 		Position origin(0,0);
	    // 		if (get_origin_pos(c-delta_w, r-delta_h, width, height, theta, origin)) {
	    // 			new_img(c, r, 0, 0) = origin_img(origin.x, origin.y, 0, 0);  // R
	    // 			new_img(c, r, 0, 1) = origin_img(origin.x, origin.y, 0, 1);  // G
	    // 			new_img(c, r, 0, 2) = origin_img(origin.x, origin.y, 0, 2);  // B
	    // 		}
	    // 	}
	    // }

        double ori_x, ori_y, u, v;
	    for (int r = 0; r < new_height; r++) {
	    	for (int c = 0; c < new_width; c++) {
	    		if (get_origin_pos(c-delta_w, r-delta_h, width, height, theta, ori_x, ori_y)) {
	    			u = ori_x - (int)ori_x;
	    			v = ori_y - (int)ori_y;
	    			for (int channel = 0; channel < 3; channel++) {
	    				new_img(c, r, 0, channel) = 
	    				    (int)((1-u)*(1-v)*origin_img(valueWidth(ori_x, width), valueHeight(ori_y, height), 0, channel)
	    					+(1-u)*v*origin_img(valueWidth(ori_x, width), valueHeight(ori_y+1, height), 0, channel)
	    					+u*(1-v)*origin_img(valueWidth(ori_x+1, width), valueHeight(ori_y, height), 0, channel)
	    					+u*v*origin_img(valueWidth(ori_x+1, width), valueHeight(ori_y+1, height), 0, channel));
	    			}
	    		}
	    	}
	    }

	    new_img.display(new_disp);

	    strcpy(filename, "new_image");
	    filename[strlen("new_image")] = rotate_times + '0';
	    strcpy(filename+strlen(filename), ".bmp");

	    new_img.save_bmp(filename);

	    while (!new_disp.is_closed() && !old_disp.is_closed()) {
	    	new_disp.wait();
	    	if (new_disp.button() && new_disp.mouse_y() >= 0) {
	    		break;
	    	}
	    }
	    ++rotate_times;
    }

	return 0;
}





