#include "CImg.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
using namespace cimg_library;
using namespace std;

// 绘制三角形
void draw_triangle(CImg<unsigned char>& img, int x1, int y1, int x2, int y2, int x3, int y3) {
	if (img != NULL) {
		int width = img.width();
		int height = img.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				double vx1 = x2 - x1, vy1 = y2 - y1, vx2 = x3 - x1, vy2 = y3 - y1;
				double vx0 = i - x1, vy0 = j - y1;
				double u = (vx2*vy0 - vx0*vy2) / (double)(vx2*vy1 - vx1*vy2),
				       v = (vx1*vy0 - vx0*vy1) / (double)(vx1*vy2 - vx2*vy1);
				if (u >= 0 && v >= 0 && u+v <= 1) {
					for (int channel = 0; channel < 3; channel++) {
						img(i, j, 0, channel) = 0;
					}
				}
			}
		}
	}
}

// 绘制矩形
void draw_rectangle(CImg<unsigned char>& img, int left, int top, int right, int bottom) {
	if (img != NULL) {
		int width = img.width();
		int height = img.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				if (i >= left && i <= right && j >= top && j <= bottom) {
					for (int channel = 0; channel < 3; channel++) {
						img(i, j, 0, channel) = 0;
					}
				}
			}
		}
	}
}

// 绘制圆形
void draw_circle(CImg<unsigned char>& img, int center_x, int center_y, double radius) {
	if (img != NULL) {
		int width = img.width();
		int height = img.height();
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				if ((i - center_x)*(i - center_x)+(j-center_y)*(j-center_y) <= radius*radius) {
					for (int channel = 0; channel < 3; channel++) {
						img(i, j, 0, channel) = 0;
					}
				}
			}
		}
	}
}

int main() {
	string file = "绘制矩形、三角形和圆.bmp";

	CImg<unsigned char> origin_img(file.c_str());

    int width = origin_img.width();
    int height = origin_img.height();

    int center_x = width / 2, center_y = height / 2;

    CImg<unsigned char> circle_img(origin_img);
    draw_circle(circle_img, center_x, center_y, 400);
    circle_img.save_bmp("circle.bmp");

    int long_side = 600, short_side = 500;
    int left = center_x - long_side/2, top = center_y - short_side/2,
        right = center_x + long_side/2, bottom = center_y + short_side/2;
    CImg<unsigned char> rectangle_img(origin_img);
    draw_rectangle(rectangle_img, left, top, right, bottom);
    rectangle_img.save_bmp("rectangle.bmp");

    CImg<unsigned char> triangle_img(origin_img);
    draw_triangle(triangle_img, 900, 200, 100, 500, 400, 900);
    triangle_img.save_bmp("triangle.bmp");

    // while (!circle_disp.is_closed() || !rectangle_disp.is_closed() || !triangle_disp.is_closed()) {
    // 	circle_disp.wait();
    // 	rectangle_disp.wait();
    // 	triangle_disp.wait();
    // }

}