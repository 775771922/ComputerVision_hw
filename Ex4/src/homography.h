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

    /*
	static inline void calc_homography(const vector<Point2f> &src,
			const vector<Point2f> &dest, double h[]) {
		int u0 = src[0].x, v0 = src[0].y, u1 = src[1].x, v1 = src[1].y, 
		    u2 = src[2].x, v2 = src[2].y, u3 = src[3].x, v3 = src[3].y,

	    	x0 = dest[0].x, y0 = dest[0].y, x1 = dest[1].x, y1 = dest[1].y, 
	    	x2 = dest[2].x, y2 = dest[2].y, x3 = dest[3].x, y3 = dest[4].y;
		double c1, c2, c3, c4, c5, c6, c7, c8;
		c1 = -(u0 * v0 * v1 * x2 - u0 * v0 * v2 * x1 - u0 * v0 * v1 * x3
				+ u0 * v0 * v3 * x1 - u1 * v0 * v1 * x2 + u1 * v1 * v2 * x0
				+ u0 * v0 * v2 * x3 - u0 * v0 * v3 * x2 + u1 * v0 * v1 * x3
				- u1 * v1 * v3 * x0 + u2 * v0 * v2 * x1 - u2 * v1 * v2 * x0
				- u1 * v1 * v2 * x3 + u1 * v1 * v3 * x2 - u2 * v0 * v2 * x3
				+ u2 * v2 * v3 * x0 - u3 * v0 * v3 * x1 + u3 * v1 * v3 * x0
				+ u2 * v1 * v2 * x3 - u2 * v2 * v3 * x1 + u3 * v0 * v3 * x2
				- u3 * v2 * v3 * x0 - u3 * v1 * v3 * x2 + u3 * v2 * v3 * x1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c2 =

		(u0 * u1 * v0 * x2 - u0 * u2 * v0 * x1 - u0 * u1 * v0 * x3
				- u0 * u1 * v1 * x2 + u0 * u3 * v0 * x1 + u1 * u2 * v1 * x0
				+ u0 * u1 * v1 * x3 + u0 * u2 * v0 * x3 + u0 * u2 * v2 * x1
				- u0 * u3 * v0 * x2 - u1 * u2 * v2 * x0 - u1 * u3 * v1 * x0
				- u0 * u2 * v2 * x3 - u0 * u3 * v3 * x1 - u1 * u2 * v1 * x3
				+ u1 * u3 * v1 * x2 + u1 * u3 * v3 * x0 + u2 * u3 * v2 * x0
				+ u0 * u3 * v3 * x2 + u1 * u2 * v2 * x3 - u2 * u3 * v2 * x1
				- u2 * u3 * v3 * x0 - u1 * u3 * v3 * x2 + u2 * u3 * v3 * x1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c3 =

		(u0 * v1 * x2 - u0 * v2 * x1 - u1 * v0 * x2 + u1 * v2 * x0
				+ u2 * v0 * x1 - u2 * v1 * x0 - u0 * v1 * x3 + u0 * v3 * x1
				+ u1 * v0 * x3 - u1 * v3 * x0 - u3 * v0 * x1 + u3 * v1 * x0
				+ u0 * v2 * x3 - u0 * v3 * x2 - u2 * v0 * x3 + u2 * v3 * x0
				+ u3 * v0 * x2 - u3 * v2 * x0 - u1 * v2 * x3 + u1 * v3 * x2
				+ u2 * v1 * x3 - u2 * v3 * x1 - u3 * v1 * x2 + u3 * v2 * x1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c4 =

		(u0 * u1 * v0 * v2 * x3 - u0 * u1 * v0 * v3 * x2
				- u0 * u2 * v0 * v1 * x3 + u0 * u2 * v0 * v3 * x1
				+ u0 * u3 * v0 * v1 * x2 - u0 * u3 * v0 * v2 * x1
				- u0 * u1 * v1 * v2 * x3 + u0 * u1 * v1 * v3 * x2
				+ u1 * u2 * v0 * v1 * x3 - u1 * u2 * v1 * v3 * x0
				- u1 * u3 * v0 * v1 * x2 + u1 * u3 * v1 * v2 * x0
				+ u0 * u2 * v1 * v2 * x3 - u0 * u2 * v2 * v3 * x1
				- u1 * u2 * v0 * v2 * x3 + u1 * u2 * v2 * v3 * x0
				+ u2 * u3 * v0 * v2 * x1 - u2 * u3 * v1 * v2 * x0
				- u0 * u3 * v1 * v3 * x2 + u0 * u3 * v2 * v3 * x1
				+ u1 * u3 * v0 * v3 * x2 - u1 * u3 * v2 * v3 * x0
				- u2 * u3 * v0 * v3 * x1 + u2 * u3 * v1 * v3 * x0)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c5 =

		-(u0 * v0 * v1 * y2 - u0 * v0 * v2 * y1 - u0 * v0 * v1 * y3
				+ u0 * v0 * v3 * y1 - u1 * v0 * v1 * y2 + u1 * v1 * v2 * y0
				+ u0 * v0 * v2 * y3 - u0 * v0 * v3 * y2 + u1 * v0 * v1 * y3
				- u1 * v1 * v3 * y0 + u2 * v0 * v2 * y1 - u2 * v1 * v2 * y0
				- u1 * v1 * v2 * y3 + u1 * v1 * v3 * y2 - u2 * v0 * v2 * y3
				+ u2 * v2 * v3 * y0 - u3 * v0 * v3 * y1 + u3 * v1 * v3 * y0
				+ u2 * v1 * v2 * y3 - u2 * v2 * v3 * y1 + u3 * v0 * v3 * y2
				- u3 * v2 * v3 * y0 - u3 * v1 * v3 * y2 + u3 * v2 * v3 * y1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c6 =

		(u0 * u1 * v0 * y2 - u0 * u2 * v0 * y1 - u0 * u1 * v0 * y3
				- u0 * u1 * v1 * y2 + u0 * u3 * v0 * y1 + u1 * u2 * v1 * y0
				+ u0 * u1 * v1 * y3 + u0 * u2 * v0 * y3 + u0 * u2 * v2 * y1
				- u0 * u3 * v0 * y2 - u1 * u2 * v2 * y0 - u1 * u3 * v1 * y0
				- u0 * u2 * v2 * y3 - u0 * u3 * v3 * y1 - u1 * u2 * v1 * y3
				+ u1 * u3 * v1 * y2 + u1 * u3 * v3 * y0 + u2 * u3 * v2 * y0
				+ u0 * u3 * v3 * y2 + u1 * u2 * v2 * y3 - u2 * u3 * v2 * y1
				- u2 * u3 * v3 * y0 - u1 * u3 * v3 * y2 + u2 * u3 * v3 * y1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c7 =

		(u0 * v1 * y2 - u0 * v2 * y1 - u1 * v0 * y2 + u1 * v2 * y0
				+ u2 * v0 * y1 - u2 * v1 * y0 - u0 * v1 * y3 + u0 * v3 * y1
				+ u1 * v0 * y3 - u1 * v3 * y0 - u3 * v0 * y1 + u3 * v1 * y0
				+ u0 * v2 * y3 - u0 * v3 * y2 - u2 * v0 * y3 + u2 * v3 * y0
				+ u3 * v0 * y2 - u3 * v2 * y0 - u1 * v2 * y3 + u1 * v3 * y2
				+ u2 * v1 * y3 - u2 * v3 * y1 - u3 * v1 * y2 + u3 * v2 * y1)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		c8 =

		(u0 * u1 * v0 * v2 * y3 - u0 * u1 * v0 * v3 * y2
				- u0 * u2 * v0 * v1 * y3 + u0 * u2 * v0 * v3 * y1
				+ u0 * u3 * v0 * v1 * y2 - u0 * u3 * v0 * v2 * y1
				- u0 * u1 * v1 * v2 * y3 + u0 * u1 * v1 * v3 * y2
				+ u1 * u2 * v0 * v1 * y3 - u1 * u2 * v1 * v3 * y0
				- u1 * u3 * v0 * v1 * y2 + u1 * u3 * v1 * v2 * y0
				+ u0 * u2 * v1 * v2 * y3 - u0 * u2 * v2 * v3 * y1
				- u1 * u2 * v0 * v2 * y3 + u1 * u2 * v2 * v3 * y0
				+ u2 * u3 * v0 * v2 * y1 - u2 * u3 * v1 * v2 * y0
				- u0 * u3 * v1 * v3 * y2 + u0 * u3 * v2 * v3 * y1
				+ u1 * u3 * v0 * v3 * y2 - u1 * u3 * v2 * v3 * y0
				- u2 * u3 * v0 * v3 * y1 + u2 * u3 * v1 * v3 * y0)
				/ (u0 * u1 * v0 * v2 - u0 * u2 * v0 * v1 - u0 * u1 * v0 * v3
						- u0 * u1 * v1 * v2 + u0 * u3 * v0 * v1
						+ u1 * u2 * v0 * v1 + u0 * u1 * v1 * v3
						+ u0 * u2 * v0 * v3 + u0 * u2 * v1 * v2
						- u0 * u3 * v0 * v2 - u1 * u2 * v0 * v2
						- u1 * u3 * v0 * v1 - u0 * u2 * v2 * v3
						- u0 * u3 * v1 * v3 - u1 * u2 * v1 * v3
						+ u1 * u3 * v0 * v3 + u1 * u3 * v1 * v2
						+ u2 * u3 * v0 * v2 + u0 * u3 * v2 * v3
						+ u1 * u2 * v2 * v3 - u2 * u3 * v0 * v3
						- u2 * u3 * v1 * v2 - u1 * u3 * v2 * v3
						+ u2 * u3 * v1 * v3);

		h[0] = c1, h[1] = c2, h[2] = c3, h[3] = c4, h[4] = c5,
		h[5] = c6, h[6] = c7, h[7] = c8, h[8] = 1;

	}
	*/
};

#endif
