#include "ScaleOP.h"

// 检查像素位置，防止超过图片宽度
int ScaleOP::valueWidth(double srcX, int width) {
	if (srcX < 0) srcX = 0;
	if (srcX >= width) srcX--;
	return srcX;
}

// 检查像素位置，防止超过图片高度
int ScaleOP::valueHeight(double srcY, int height) {
	if (srcY < 0) srcY = 0;
	if (srcY >= height) srcY--;
	return srcY;
}

// 最邻近插值缩放图片
// params:
// width height: 缩放后图片的宽高
CImg<unsigned char> ScaleOP::nearest_scale(const CImg<unsigned char>& srcImg, 
		int width, int height) {
	int srcWidth = srcImg.width();
	int srcHeight = srcImg.height();
	CImg<unsigned char> outImg(width, height, 1, 3, 0);
	// 缩放因子
	double width_scale = srcWidth / (double)width;
	double height_scale = srcHeight / (double)height;
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			outImg(c, r, 0, 0) = srcImg((int)(c*width_scale), (int)(r*height_scale), 0, 0);
			outImg(c, r, 0, 1) = srcImg((int)(c*width_scale), (int)(r*height_scale), 0, 1);
			outImg(c, r, 0, 2) = srcImg((int)(c*width_scale), (int)(r*height_scale), 0, 2);
		}
	}
	return outImg;
}

// 双线性插值缩放图片
// params:
// width height: 缩放后图片的宽高
CImg<unsigned char> ScaleOP::bilinear_scale(const CImg<unsigned char>& srcImg,
	    int width, int height) {
	int srcWidth = srcImg.width();
	int srcHeight = srcImg.height();
	CImg<unsigned char> outImg(width, height, 1, 3, 0);
	// 缩放因子
	double width_scale = srcWidth / (double)width;
	double height_scale = srcHeight / (double)height;
	// 新图对应原图的坐标
	double srcX, srcY, u, v;
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			srcX = c * width_scale;
			srcY = r * height_scale;
			u = srcX - (int)srcX;
			v = srcY - (int)srcY;
			for (int channel = 0; channel < 3; channel++) {
				outImg(c, r, 0, channel) = (int)((1-u)*(1-v)*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
					                  +(1-u)*v*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel)
					                  +u*(1-v)*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
					                  +u*v*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel));
		    }
		}
	}
	return outImg;
}