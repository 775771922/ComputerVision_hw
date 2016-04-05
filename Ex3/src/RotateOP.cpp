#include "RotateOP.h"
#include <cmath>
#include <cstring>
#include <string>

using namespace std;

/**
* 采用双线性插值填充新图
*/
void RotateOP::biliinear_interpolation(CImg<float>& outImg, 
    	const CImg<float>& srcImg, double theta) {
	int newWidth = outImg.width(), newHeight = outImg.height();
	int srcWidth = srcImg.width(), srcHeight = srcImg.height();
	int halfW = newWidth / 2;
    int halfH = newHeight / 2;
    double srcX, srcY, u, v;
    for (int r = 0; r < newHeight; r++) {
    	for (int c = 0; c < newWidth; c++) {
    		if (get_origin_pos(c-halfW, r-halfH, srcWidth, srcHeight, theta, srcX, srcY)) {
    			u = srcX - (int)srcX;
    			v = srcY - (int)srcY;
    			for (int channel = 0; channel < 3; channel++) {
    				outImg(c, r, 0, channel) = 
    				    (int)((1-u)*(1-v)*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
    					+(1-u)*v*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel)
    					+u*(1-v)*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
    					+u*v*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel));
    			}
    		}
    	}
    }
}

/**
* 采用最邻近插值填充新图
*/
void RotateOP::nearest_interpolation(CImg<float>& outImg, 
	    const CImg<float>& srcImg, double theta) {
	int newWidth = outImg.width(), newHeight = outImg.height();
	int srcWidth = srcImg.width(), srcHeight = srcImg.height();
	int halfW = newWidth / 2;
    int halfH = newHeight / 2;
    int srcX, srcY, u, v;
    for (int r = 0; r < newHeight; r++) {
    	for (int c = 0; c < newWidth; c++) {
    		if (get_origin_pos(c-halfW, r-halfH, srcWidth, srcHeight, theta, srcX, srcY)) {
    			for (int channel = 0; channel < 3; channel++) {
    				outImg(c, r, 0, channel) = srcImg(srcX, srcY, 0, channel);
    			}
    		}
    	}
    }
}

// 获得旋转后图片的像素对应于原图的像素位置，用于最邻近插值
bool RotateOP::get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
	                int& srcX, int& srcY) {
    // 找到(x, y)在原图中对应的位置(srcX, srcY)
	srcX = x*cos(theta) - y*sin(theta);
	srcY = x*sin(theta) + y*cos(theta);
	if (srcX >= (0-srcWidth/2) && srcX <= srcWidth/2 && srcY >= (0-srcHeight/2) && srcY <= srcHeight/2) {
		srcX += srcWidth/2;
		srcY += srcHeight/2;
		return true;
	} else {
		return false;
	}
}

// 获得旋转后图片的像素对应于原图的像素位置，用于双线性插值
bool RotateOP::get_origin_pos(int x, int y, int srcWidth, int srcHeight, double theta, 
	                double& srcX, double& srcY) {
    // 找到(x, y)在原图中对应的位置(srcX, srcY)
	srcX = (double)x * cos(theta) - (double)y * sin(theta);
	srcY = (double)x * sin(theta) + (double)y * cos(theta);
	if (srcX >= (0-srcWidth/2-1) && srcX <= srcWidth/2+1 && srcY >= (0-srcHeight/2-1) && srcY <= srcHeight/2+1) {
		srcX += srcWidth/2;
		srcY += srcHeight/2;
		return true;
	} else {
		return false;
	}
}

// 检查像素位置，防止超过图片宽度
int RotateOP::valueWidth(double srcX, int width) {
	if (srcX < 0) srcX = 0;
	if (srcX >= width) srcX--;
	return srcX;
}

// 检查像素位置，防止超过图片高度
int RotateOP::valueHeight(double srcY, int height) {
	if (srcY < 0) srcY = 0;
	if (srcY >= height) srcY--;
	return srcY;
}


// 旋转图片，旋转角为theta，并采用双线性插值减少锯齿
CImg<float> RotateOP::rotate_biliinear(const CImg<float>& srcImg, double theta) {
	int width = srcImg.width();
    int height = srcImg.height();
    // 原图的四个顶点坐标，这里以图片中心为坐标原点
    Position lt(0-width/2, 0+height/2), lb(0-width/2, 0-height/2), 
         rt(0+width/2, 0+height/2), rb(0+width/2, 0-height/2);
    // 获得旋转后的图片的四个顶点坐标
    Position new_lt((int)(lt.x*cos(theta)+lt.y*sin(theta)), (int)(lt.y*cos(theta)-lt.x*sin(theta))), 
             new_lb((int)(lb.x*cos(theta)+lb.y*sin(theta)), (int)(lb.y*cos(theta)-lb.x*sin(theta))),
             new_rt((int)(rt.x*cos(theta)+rt.y*sin(theta)), (int)(rt.y*cos(theta)-rt.x*sin(theta))), 
             new_rb((int)(rb.x*cos(theta)+rb.y*sin(theta)), (int)(rb.y*cos(theta)-rb.x*sin(theta)));
    int newWidth = max(abs(new_rt.x-new_lb.x), abs(new_lt.x-new_rb.x));
    int newHeight = max(abs(new_lt.y-new_rb.y), abs(new_lb.y-new_rt.y));

    CImg<float> newImg(newWidth, newHeight, 1, 3, 0);
    // 开始填充新图片的灰度值
    biliinear_interpolation(newImg, srcImg, theta);

    return newImg;	
}





