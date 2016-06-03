#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <vector>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

class Homography {

public:

	static inline double calc_X(double srcX, double srcY, double h[]) {
		
		return (h[0]*srcX+h[1]*srcY+h[2])/(h[6]*srcX+h[7]*srcY+1);
	}

	static inline double calc_Y(double srcX, double srcY, double h[]) {
		
		return (h[3]*srcX+h[4]*srcY+h[5])/(h[6]*srcX+h[7]*srcY+1);
	}

	static inline double calc_X(double srcX, double srcY, double h[], int offsetX) {
		
		return (h[0]*srcX+h[1]*srcY+h[2])/(h[6]*srcX+h[7]*srcY+1) + offsetX;
	}

	static inline double calc_Y(double srcX, double srcY, double h[], int offsetY) {
		
		return (h[3]*srcX+h[4]*srcY+h[5])/(h[6]*srcX+h[7]*srcY+1) + offsetY;
	}

	static inline void calc_homography(const vector<Point2f> &src,
		const vector<Point2f> &dest, double h[]) {
		
		Mat matrix = getPerspectiveTransform(src, dest);
		int nRows = matrix.rows;
		int nCols = matrix.cols;
		for (int i = 0; i < nRows; i++) {
     		for (int j = 0; j < nCols; j++) {
  	        	h[i*nCols+j] = matrix.at<double>(i, j);
		    }
		}

	}

	static inline void least_squares(double *A, double *b, double h[], int N) {
		Mat matrixA = Mat(2*N, 8, CV_64FC1, A);
		Mat vectorB = Mat(2*N, 1, CV_64FC1, b);
		Mat tranA = matrixA.t();
		Mat res = (tranA*matrixA).inv()*tranA*vectorB;
		int nRows = res.rows;
		for (int i = 0; i < nRows; i++) {
  	        h[i] = res.at<double>(i, 0);
		}
		h[8] = 1;
	}
};

#endif
