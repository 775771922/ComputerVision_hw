#ifndef NUMBER_RECOGNITION_H
#define NUMBER_RECOGNITION_H

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "CImg.h"
#include "paper_corection.h"
#include "image_segmentation.h"
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
using namespace cimg_library;

#define NUMBER_REG_DEBUG

#define TEST_IMAGE_SIZE 28

int c = 0;

struct Line {
	int start, end;
	Line(int s, int e) {
		start = s;
		end = e;
	}
};

template<class T>
void draw(CImg<T> &srcImg, Line X, Line Y) {
	const unsigned char blue[] = {0, 0, 255};
	srcImg.draw_line(X.start, Y.start, X.end, Y.start, blue);
	srcImg.draw_line(X.end, Y.start, X.end, Y.end, blue);
	srcImg.draw_line(X.end, Y.end, X.start, Y.end, blue);
	srcImg.draw_line(X.start, Y.end, X.start, Y.start, blue);
}

template<class T>
class NumberReg {
public:
	vector<int> read_number_in(const CImg<T> &srcImg);
	NumberReg(PaperCorection p);
//private:
	int MIN_PIXEL_ON_Y;
	int MIN_PIXEL_ON_X;
	int MIN_LENGTH_ON_Y;
	int MIN_LENGTH_ON_X;

	PaperCorection paperCorection;
	CImg<T> correct_paper(const CImg<T> &srcImg);
    vector<CImg<T> > detect_number_area(const CImg<T> &srcImg);
    bool check_number_image(CImg<T> &target);
    vector<int> project_to_y(const CImg<T> &srcImg);
    vector<int> project_to_x(const CImg<T> &srcImg);
    vector<Line> get_lines_from_hist(vector<int> &hist, int minPixel, int minLength);
    vector<CImg<T> > get_imgs_from_lines(const CImg<T> &srcImg, vector<Line> &lines, bool isX);
    CImg<T> normalize_img(CImg<T> &srcImg);
    void test_project_to_x(const CImg<T> &srcImg);
    void test_svm(string dirName);
    void test_adaboost(string dirName);
};

template<class T>
NumberReg<T>::NumberReg(PaperCorection p): paperCorection(p) {
	ifstream fin("config.txt");
	string type;
	int number;
	while (fin >> type >> number) {
		if (type == "MIN_PIXEL_ON_Y") {
			MIN_PIXEL_ON_Y = number;
			cout << "MIN_PIXEL_ON_Y===>" << MIN_PIXEL_ON_Y << endl;
		} else if (type == "MIN_PIXEL_ON_X") {
			MIN_PIXEL_ON_X = number;
			cout << "MIN_PIXEL_ON_X===>" << MIN_PIXEL_ON_X << endl;
		} else if (type == "MIN_LENGTH_ON_Y") {
			MIN_LENGTH_ON_Y = number;
			cout << "MIN_LENGTH_ON_Y===>" << MIN_LENGTH_ON_Y << endl;
		} else if (type == "MIN_LENGTH_ON_X") {
			MIN_LENGTH_ON_X = number;
			cout << "MIN_LENGTH_ON_X===>" << MIN_LENGTH_ON_X << endl;
		} else {
			continue;
		}
	}
	fin.close();
}

// template<class T>
// vector<int> NumberReg<T>::read_number_in(const CImg<T> &srcImg) {
// 	CImg<T> correctionPaper = correct_paper(srcImg);
// 	vector<CImg<T> > numberAreas = detect_number_area(correctionPaper);

// 	for (int i = 0; i < numberAreas.size(); i++) {
// 		normalize_img(numberAreas[i]);
// 	}
// }

template<class T>
vector<int> NumberReg<T>::read_number_in(const CImg<T> &srcImg) {
	int idx = 0;
	//CImg<T> correctionPaper = correct_paper(srcImg);
	vector<int> predictions;
	vector<CImg<T> > numberAreas = detect_number_area(srcImg);
	vector<CImg<T> > normalizedImgs;
	for (int i = 0; i < numberAreas.size(); i++) {
		normalizedImgs.push_back(normalize_img(numberAreas[i]));
	}

    HOGDescriptor hog(cvSize(28,28),cvSize(14,14),cvSize(1,1),cvSize(7,7),9);
    Size winStride = Size(1, 1);
    CvSVM svm;
    svm.load("svm-linear-kernel-100");
    for (int i = 0; i < normalizedImgs.size(); i++) {
    	normalizedImgs[i].convertTo(normalizedImgs[i], CV_8UC1);

    	vector<float> descriptors;
    	hog.compute(normalizedImgs[i], descriptors, winStride);

        Mat tmp;
        tmp.create(1, descriptors.size(), CV_32FC1);
        for (int j = 0; j < descriptors.size(); j++) {
        	tmp.at<float>(0, j) = descriptors[j];
        }

    	int prediction = svm.predict(tmp);
    	predictions.push_back(prediction);
    	cout << "prediction=====>" << prediction << endl;
    	normalizedImgs[i].display();
    }

    return predictions;
}

template<class T>
CImg<T> NumberReg<T>::correct_paper(const CImg<T> &srcImg) {
	return paperCorection.paper_corection(srcImg);
}

template<class T>
CImg<T> NumberReg<T>::normalize_img(CImg<T> &srcImg) {
    ImageSeg<T> imageSeg;
    cimg_forXY(srcImg, x, y) {
    	srcImg(x, y, 0, 0) = 255 - srcImg(x, y, 0, 0);
    }

    #ifdef NUMBER_REG_DEBUG
    //srcImg.display("normalize_img");
    #endif

	CImg<T> temp, ret(TEST_IMAGE_SIZE, TEST_IMAGE_SIZE, 1, 1, 0);
	double scale;
	if (srcImg.width() > srcImg.height()) {
		scale = (double)srcImg.width() / TEST_IMAGE_SIZE;
		temp = srcImg.get_resize(TEST_IMAGE_SIZE, (int)(srcImg.height() / scale), 1, 1, 3);
		int deltaY = (TEST_IMAGE_SIZE - temp.height()) / 2;
		for (int row = 0; row < TEST_IMAGE_SIZE; row++) {
			for (int col = deltaY, h = 0; h < temp.height(); col++, h++) {
				ret(row, col, 0, 0) = temp(row, h, 0, 0);
			}
		}
		#ifdef NUMBER_REG_DEBUG
		//ret.display("ret");
		ret.save(("normal3/" + to_string(c++) + ".png").c_str());
		#endif
	} else {
		scale = (double)srcImg.height() / TEST_IMAGE_SIZE;
		temp = srcImg.get_resize((int)(srcImg.width() / scale), TEST_IMAGE_SIZE, 1, 1, 3);
		int deltaX = (TEST_IMAGE_SIZE - temp.width()) / 2;
		for (int col = 0; col < TEST_IMAGE_SIZE; col++) {
			for (int row = deltaX, w = 0; w < temp.width(); row++, w++) {
				ret(row, col, 0, 0) = temp(w, col, 0, 0);
			}
		}
		#ifdef NUMBER_REG_DEBUG
		//ret.display("ret");
		ret.save(("normal3/" + to_string(c++) + ".png").c_str());
		#endif
	}

	return ret;
}

template<class T>
vector<CImg<T> > NumberReg<T>::detect_number_area(const CImg<T> &srcImg) {

	vector<int> histY = project_to_y(srcImg);
	vector<Line> yLines = get_lines_from_hist(histY, MIN_PIXEL_ON_Y, MIN_LENGTH_ON_Y);

	vector<CImg<T> > yImgs = get_imgs_from_lines(srcImg, yLines, false);

    int c = 0;
    vector<CImg<T> > ret;
    const unsigned char blue[] = {0, 0, 255};
	for (int i = 0; i < yImgs.size(); i++) {
		vector<int> histX = project_to_x(yImgs[i]);
		vector<Line> xLines = get_lines_from_hist(histX, MIN_PIXEL_ON_X, MIN_LENGTH_ON_X);
		for (int j = 0; j < xLines.size(); j++) {

			CImg<T> cropImg = yImgs[i].get_crop(xLines[j].start, 0, xLines[j].end, yImgs[i].height()-1);
			ret.push_back(cropImg);

			#ifdef NUMBER_REG_DEBUG

			cropImg.save(("cropImg" + to_string(c) + ".png").c_str());

			yImgs[i].draw_line(xLines[j].start, 0, xLines[j].start, yImgs[i].height()-1, blue);
			yImgs[i].draw_line(xLines[j].end, 0, xLines[j].end, yImgs[i].height()-1, blue);
			yImgs[i].display((to_string(c++)+".png").c_str());
			yImgs[i].save((to_string(c)+".png").c_str());

			//cropImg.display(("cropImg" + to_string(c)).c_str());
			#endif
		}
	}

	return ret;
}

template<class T>
vector<int> NumberReg<T>::project_to_x(const CImg<T> &srcImg) {
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
	// for (int i = 0; i < histX.size(); i++) {
	// 	cout << i << ": " << histX[i] << endl;
	// }
	// cout << endl;
	#endif

	return histX;

}

template<class T>
vector<int> NumberReg<T>::project_to_y(const CImg<T> &srcImg) {
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
	return histY;

}

template<class T>
vector<Line> NumberReg<T>::get_lines_from_hist(vector<int> &hist, int minPixel, int minLength) {
    int start = -1;
    vector<int> minPixelIndex;
    vector<Line> lines;
    for (int i = 0; i < hist.size(); i++) {
    	if (i == hist.size()-1) {
    		minPixelIndex.push_back(i);     // 最后一个直接放进去，留待之后处理
    		break;
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

template<class T>
vector<CImg<T> > NumberReg<T>::get_imgs_from_lines(const CImg<T> &srcImg, vector<Line> &lines, bool isX) {
	vector<CImg<T> > ret;
	if (isX) {
		for (int i = 0; i < lines.size(); i++) {
			ret.push_back(srcImg.get_crop(lines[i].start, 0, lines[i].end, srcImg.height()));
			//ret[i].display(("ret" + to_string(i)).c_str());
		}
	} else {
		for (int i = 0; i < lines.size(); i++) {
			ret.push_back(srcImg.get_crop(0, lines[i].start, srcImg.width(), lines[i].end));
			//ret[i].display(("ret" + to_string(i)).c_str());
			ret[i].save(("ret" + to_string(i) + ".png").c_str());
		}
	}
	return ret;
}

template<class T>
void NumberReg<T>::test_project_to_x(const CImg<T> &srcImg) {
	CImg<T> t(srcImg);
	vector<Line> xLines = project_to_x(srcImg);
	for (int i = 0; i < xLines.size(); i++) {
		const unsigned char blue[] = {0, 0, 255};
		t.draw_line(xLines[i].start, 0, xLines[i].start, t.height(), blue);
		t.draw_line(xLines[i].end, 0, xLines[i].end, t.height(), blue);
		string name = "test" + to_string(i) + ".png";
		t.display(name.c_str());
		t.save(name.c_str());
	}
}


vector<string> get_files_from_dir(string dirName) {
	DIR *dir;
	struct dirent *dirp;
	struct stat filestat;
	vector<string> files;

	dir = opendir(dirName.c_str());

	if (dir == NULL) {
		cout << "Directory " << dir << " not found!" << endl;
		return files;
	}

	while ((dirp = readdir(dir))) {
		string file_path = dirName;
		if (dirName[dirName.length()-1] != '/') {
			file_path += "/";
		}
		file_path += dirp->d_name;

		if (stat(file_path.c_str(), &filestat)) {
			continue;
		}

        string postfix[5] = {".bmp", ".jpg", ".jpeg", ".JPG", ".png"};
		for (int i = 0; i < 5; i++) {
			if (file_path.find(postfix[i]) != string::npos) {
				files.push_back(file_path);
			}
		}
	}

	closedir(dir);
	return files;
}

template<class T>
void NumberReg<T>::test_svm(string dirName) {
	ofstream fout("prediction.txt");
	cout << "dir====>" << dirName << endl;
	std::vector<string> v = get_files_from_dir(dirName);
	cout << "size: " << v.size() << endl;

    HOGDescriptor hog(cvSize(28,28),cvSize(14,14),cvSize(1,1),cvSize(7,7),9);
    Size winStride = Size(1, 1);
    CvSVM svm;
    svm.load("svm-linear-kernel-300");
    for (int i = 0; i < v.size(); i++) {
    	Mat input = imread(v[i].c_str(), 1);
    	CImg<T> img(v[i].c_str());
    	input.convertTo(input, CV_8UC1);
    	vector<float> descriptors;
    	hog.compute(input, descriptors, winStride);

        Mat tmp;
        tmp.create(1, descriptors.size(), CV_32FC1);
        for (int j = 0; j < descriptors.size(); j++) {
        	tmp.at<float>(0, j) = descriptors[j];
        }

    	int prediction = svm.predict(tmp);
    	cout << "prediction=====>" << prediction << endl;
    	fout << v[i].c_str() << "===>" << prediction << endl;
    	img.display();
    }
    fout.close();

}

template<class T>
void NumberReg<T>::test_adaboost(string dirName) {
	ofstream fout("prediction.txt");
	cout << "dir====>" << dirName << endl;
	std::vector<string> v = get_files_from_dir(dirName);
	cout << "size: " << v.size() << endl;

    HOGDescriptor hog(cvSize(28,28),cvSize(14,14),cvSize(1,1),cvSize(7,7),9);
    Size winStride = Size(1, 1);
    CvBoost boost;
    for (int i = 0; i < v.size(); i++) {
    	Mat input = imread(v[i].c_str(), 1);
    	CImg<T> img(v[i].c_str());
    	input.convertTo(input, CV_8UC1);
    	vector<float> descriptors;
    	hog.compute(input, descriptors, winStride);

    	Mat tmp;
    	tmp.create(1, descriptors.size(), CV_32FC1);
    	for (int j = 0; j < descriptors.size(); j++) {
    		tmp.at<float>(0, j) = descriptors[j];
    	}

    	double prediction = 0;
    	int predictionNum = 0;

    	for (int j = 0; j < 10; j++) {
    		string name = "train-boost-classifiers-10-class-" + to_string(j);
    		boost.load(name.c_str());
    		double tempPrediction = boost.predict(tmp);
    		cout << "predict " << j << ": " << tempPrediction << endl;
    		if (tempPrediction > prediction) {
    			prediction = tempPrediction;
    			predictionNum = j;
    		}
    	}

    	cout << "prediction=====>" << predictionNum << endl;
    	fout << v[i].c_str() << "===>" << predictionNum << endl;
    	img.display();
    }
    fout.close();

}


#endif