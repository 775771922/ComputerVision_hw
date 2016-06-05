#ifndef NUMBER_RECOGNITION_H
#define NUMBER_RECOGNITION_H

#include "CImg.h"
#include "paper_corection.h"
#include <vector>
#include <iostream>
using namespace std;
using namespace cimg_library;

#define NUMBER_REG_DEBUG

#define MIN_PIXEL_ON_Y 10
#define MIN_PIXEL_ON_X 8
#define MIN_LENGTH_ON_Y 20
#define MIN_LENGTH_ON_X 10

struct Line {
	int start, end;
	Line(int s, int e) {
		start = s;
		end = e;
	}
};

template<class T>
class NumberReg {
public:
	vector<int> read_number_in(const CImg<T> &srcImg);
	NumberReg(PaperCorection p);
//private:
	PaperCorection paperCorection;
	CImg<T> correct_paper(const CImg<T> &srcImg);
    vector<CImg<T> > detect_number_area(CImg<T> &srcImg);
    vector<Line> project_to_y(const CImg<T> &srcImg);
    vector<Line> project_to_x(const CImg<T> &srcImg);
    vector<Line> get_lines_from_hist(vector<int> &hist, int minPixel, int minLength);
};

template<class T>
NumberReg<T>::NumberReg(PaperCorection p): paperCorection(p) {}

template<class T>
vector<int> NumberReg<T>::read_number_in(const CImg<T> &srcImg) {
	vector<CImg<T> > numberAreas = detect_number_area(correct_paper(srcImg));

}

template<class T>
CImg<T> NumberReg<T>::correct_paper(const CImg<T> &srcImg) {
	return paperCorection.paper_corection(srcImg);
}

template<class T>
vector<CImg<T> > NumberReg<T>::detect_number_area(CImg<T> &srcImg) {

}

template<class T>
vector<Line> NumberReg<T>::project_to_x(const CImg<T> &srcImg) {
	vector<int> histX(srcImg.width(), 0);
	cimg_forX(srcImg, x) {
		int sum = 0;
		cimg_forY(srcImg, y) {
			if (srcImg(x, y, 0, 0) == 0) {
				sum++;
			}
		} 
		histX[x] = sum;
	}

	#ifdef NUMBER_REG_DEBUG
	for (int i = 0; i < histX.size(); i++) {
		cout << i << ": " << histX[i] << endl;
	}
	cout << endl;
	#endif

	return get_lines_from_hist(histX, MIN_PIXEL_ON_X, MIN_LENGTH_ON_X);

}

template<class T>
vector<Line> NumberReg<T>::project_to_y(const CImg<T> &srcImg) {
	vector<int> histY(srcImg.height(), 0);
	cimg_forY(srcImg, y) {
		int sum = 0;
		cimg_forX(srcImg, x) {
			if (srcImg(x, y, 0, 0) == 0) {
				sum++;
			}
		}
		histY[y] = sum;
	}

	return get_lines_from_hist(histY, MIN_PIXEL_ON_Y, MIN_LENGTH_ON_Y);

}

template<class T>
vector<Line> NumberReg<T>::get_lines_from_hist(vector<int> &hist, int minPixel, int minLength) {
    int start = -1;
    vector<int> minPixelIndex;
    vector<Line> lines;
    for (int i = 0; i < hist.size(); i++) {
    	if (i == hist.size()-1) {
    		minPixelIndex.push_back(i);     // 最后一个直接放进去，留待之后处理
    	}
    	if ((hist[i]-minPixel)*(hist[i+1]-minPixel) <= 0) {
    		minPixelIndex.push_back(i);
    	}
    }

    for (int i = 0; i < minPixelIndex.size(); i++) {
    	if (minPixelIndex[i]+1 >= hist.size()) {    // 最后一个
    		if (start != -1 && minPixelIndex[i] - start >= minLength) {
    			lines.push_back(Line(start, minPixelIndex[i]));
    		}
    	} else {
    		if (hist[minPixelIndex[i]+1] > minPixel && start == -1) {   // 开始出现波峰
    			start = minPixelIndex[i];
    		} else if (hist[minPixelIndex[i]+1] <= minPixel && start != -1) {  // 开始到达波谷
    			if (minPixelIndex[i] - start >= minLength) {
    				lines.push_back(Line(start, minPixelIndex[i]));
    			}
    			start = -1;
    		}
    	}
    }

    #ifdef NUMBER_REG_DEBUG
    cout << "index" << endl;
    for (int i = 0; i < minPixelIndex.size(); i++) {
    	cout << minPixelIndex[i] << "======>" << hist[minPixelIndex[i]] << endl;
    }
    cout << endl << endl << "line" << endl;
    for (int i = 0; i < lines.size(); i++) {
    	cout << "(" << lines[i].start << ", " << lines[i].end << ")" << endl;
    }
    #endif


    return lines;
}



#endif