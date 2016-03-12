#include "RotateOP.h"
#include "DrawOP.h"
#include "ScaleOP.h"
#include <cstring>
#include <memory.h>
#include <string>

using namespace std;

#define PI 3.1415926

void test_rotate();
void test_draw();
void test_scale();

int main() {

	test_rotate();
	test_draw();
	test_scale();

	return 0;
}



void test_rotate() {
	string file = "旋转、缩放图像1.bmp";

	CImg<unsigned char> srcImg(file.c_str());

	RotateOP rotateOP;

    char* fileName = new char[40];

    // 采用双线性/最邻近插值测试图片1的旋转效果
    for (int rotateTimes = 1; rotateTimes <= 6; rotateTimes++) {
    	CImg<unsigned char> newImg = rotateOP.rotate_nearest(srcImg, PI*rotateTimes/6);
    	memset(fileName, '\0', sizeof(char)*40);
        strcpy(fileName, "rotate_image_nearest1");
	    fileName[strlen(fileName)] = rotateTimes + '0';
	    strcpy(fileName+strlen(fileName), ".bmp");
	    newImg.save_bmp(fileName);

	    newImg = rotateOP.rotate_biliinear(srcImg, PI*rotateTimes/6);
    	memset(fileName, '\0', sizeof(char)*40);
        strcpy(fileName, "rotate_image_biliinear1");
	    fileName[strlen(fileName)] = rotateTimes + '0';
	    strcpy(fileName+strlen(fileName), ".bmp");
	    newImg.save_bmp(fileName);
    }

    file = "旋转、缩放图像2.bmp";
    srcImg.assign(file.c_str());

    // 采用双线性/最邻近插值测试图片2的旋转效果
    for (int rotateTimes = 1; rotateTimes <= 6; rotateTimes++) {

    	CImg<unsigned char> newImg = rotateOP.rotate_nearest(srcImg, PI*rotateTimes/6);
    	memset(fileName, '\0', sizeof(char)*40);
        strcpy(fileName, "rotate_image_nearest2");
	    fileName[strlen(fileName)] = rotateTimes + '0';
	    strcpy(fileName+strlen(fileName), ".bmp");
	    newImg.save_bmp(fileName);

    	newImg = rotateOP.rotate_biliinear(srcImg, PI*rotateTimes/6);
    	memset(fileName, '\0', sizeof(char)*40);
        strcpy(fileName, "rotate_image_biliinear2");
	    fileName[strlen(fileName)] = rotateTimes + '0';
	    strcpy(fileName+strlen(fileName), ".bmp");
	    newImg.save_bmp(fileName);
    }

    delete [] fileName;
}


void test_draw() {
	string file = "绘制矩形、三角形和圆.bmp";

	CImg<unsigned char> srcImg(file.c_str());

	DrawOP drawOP;

    int width = srcImg.width(), height = srcImg.height();
    int centerX = width / 2, centerY = height / 2;

    CImg<unsigned char> circleImg = drawOP.draw_circle(srcImg, centerX, centerY, 400);
    circleImg.save_bmp("circle.bmp");

    int long_side = 600, short_side = 500;
    int left = centerX - long_side/2, top = centerY - short_side/2,
        right = centerX + long_side/2, bottom = centerY + short_side/2;
    CImg<unsigned char> rectangleImg = drawOP.draw_rectangle(srcImg, left, top, right, bottom);;
    rectangleImg.save_bmp("rectangle.bmp");

    CImg<unsigned char> triangleImg = drawOP.draw_triangle(srcImg, 900, 200, 100, 500, 400, 900);
    triangleImg.save_bmp("triangle.bmp");
}


void test_scale() {
	string file = "旋转、缩放图像1.bmp";

	ScaleOP scaleOP;

	CImg<unsigned char> srcImg(file.c_str());

	CImg<unsigned char> outImg = scaleOP.nearest_scale(srcImg, 600, 500);
	outImg.save_bmp("scale_large_nearest1.bmp");
	outImg = scaleOP.bilinear_scale(srcImg, 600, 500);
	outImg.save_bmp("scale_large_bilinear1.bmp");

	outImg = scaleOP.nearest_scale(srcImg, 100, 200);
	outImg.save_bmp("scale_small_nearest1.bmp");
	outImg = scaleOP.bilinear_scale(srcImg, 100, 200);
	outImg.save_bmp("scale_small_bilinear1.bmp");

	file = "旋转、缩放图像2.bmp";
	srcImg.assign(file.c_str());

	outImg = scaleOP.nearest_scale(srcImg, 600, 500);
	outImg.save_bmp("scale_large_nearest2.bmp");
	outImg = scaleOP.bilinear_scale(srcImg, 600, 500);
	outImg.save_bmp("scale_large_bilinear2.bmp");

	outImg = scaleOP.nearest_scale(srcImg, 10, 10);
	outImg.save_bmp("scale_small_nearest2.bmp");
	outImg = scaleOP.bilinear_scale(srcImg, 10, 10);
	outImg.save_bmp("scale_small_bilinear2.bmp");

}







