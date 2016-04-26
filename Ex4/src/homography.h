#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <vector>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

class Homography {

public:

	static inline double calc_X(double srcX, double srcY, double h[]) {
		//return h[0]*srcX+h[1]*srcY+h[2]*srcX*srcY+h[3];
		return (h[0]*srcX+h[1]*srcY+h[2])/(h[6]*srcX+h[7]*srcY+1);
	}

	static inline double calc_Y(double srcX, double srcY, double h[]) {
		//return h[4]*srcX+h[5]*srcY+h[6]*srcX*srcY+h[7];
		return (h[3]*srcX+h[4]*srcY+h[5])/(h[6]*srcX+h[7]*srcY+1);
	}

	static inline double calc_X(double srcX, double srcY, double h[], int offsetX) {
		//return h[0]*srcX+h[1]*srcY+h[2]*srcX*srcY+h[3];
		return (h[0]*srcX+h[1]*srcY+h[2])/(h[6]*srcX+h[7]*srcY+1) + offsetX;
	}

	static inline double calc_Y(double srcX, double srcY, double h[], int offsetY) {
		//return h[4]*srcX+h[5]*srcY+h[6]*srcX*srcY+h[7];
		return (h[3]*srcX+h[4]*srcY+h[5])/(h[6]*srcX+h[7]*srcY+1) + offsetY;
	}

	static inline void calc_homography(const vector<Point2f> &src,
		const vector<Point2f> &dest, double h[]) {
		//Mat m = Mat(2*4, 3, CV_64FC1);
		Mat matrix = getPerspectiveTransform(src, dest);
		int nRows = matrix.rows;
		int nCols = matrix.cols;
		for (int i = 0; i < nRows; i++) {
     		for (int j = 0; j < nCols; j++) {
  	        	h[i*nCols+j] = matrix.at<double>(i, j);
		    }
		}

	}
};

#endif
