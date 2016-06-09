#include "opencv2/opencv.hpp"
//#include "opencv2/opencl_kernels_objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <cassert>
#include <ctime>

using namespace cv;
using namespace cv::gpu;
using namespace std;


const int ITERATIVE_COUNT = 300;


int reverseDigit(int i) {
	unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((int)c1 << 24) + ((int)c2 << 16) + ((int)c3 << 8) + c4;
}

//读取用于测试的手写字体图片;
vector<Mat> readTestImages() {
	int idx = 0;
	Mat img;
	ifstream file;
	vector<Mat> testData;
	cout<<"Test..." << endl;
	file.open("t10k-images.idx3-ubyte", ios::binary);
	if(!file.is_open()) {
		cout << "File Not Found!" << endl;
		exit(0);
	} else {
		int magic_number = 0;
		int number_of_images = 0;
		int n_rows = 0;
		int n_cols = 0;
		
		file.read((char*)&magic_number, sizeof(magic_number)); 
		magic_number = reverseDigit(magic_number);

		file.read((char*)&number_of_images, sizeof(number_of_images));
		number_of_images = reverseDigit(number_of_images);

		file.read((char*)&n_rows, sizeof(n_rows));
		n_rows = reverseDigit(n_rows);

		file.read((char*)&n_cols, sizeof(n_cols));
		n_cols = reverseDigit(n_cols);

		cout << "No. of images:" << number_of_images << endl;

		for(long int i = 0;i < number_of_images; ++i) {	
			img.create(n_rows, n_cols, CV_8UC1);
			for(int r = 0;r < n_rows; ++r) {
				for(int c = 0;c < n_cols; ++c) {
					uchar temp = 0;
					file.read((char*)&temp, sizeof(temp));
					img.at<uchar>(r, c) = temp;
				}
			}

			//string imgName ="Test/"+ to_string(idx) + ".jpg";
			
			testData.push_back(img);
			//imwrite(imgName, img);
			img.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading t10k-images.idx3-ubyte complete!" << endl;
	return testData;
}

//读取用于训练的手写字体图片;
vector<Mat> readTrainImages() {
	int idx = 0;
	Mat img;
	ifstream file;
	vector<Mat> trainingData;
	cout << "Training..." << endl;
	file.open("train-images.idx3-ubyte", ios::binary);
	if(!file.is_open()) {
		cout << "File Not Found!" << endl;
		exit(0);
	} else {
		int magic_number = 0;
		int number_of_images = 0;
		int n_rows = 0;
		int n_cols = 0;
		
		file.read((char*)&magic_number, sizeof(magic_number)); 
		magic_number = reverseDigit(magic_number);

		file.read((char*)&number_of_images, sizeof(number_of_images));
		number_of_images = reverseDigit(number_of_images);

		file.read((char*)&n_rows, sizeof(n_rows));
		n_rows = reverseDigit(n_rows);

		file.read((char*)&n_cols, sizeof(n_cols));
		n_cols = reverseDigit(n_cols);

		cout << "No. of images:" << number_of_images << endl;

		for(long int i = 0;i < number_of_images; ++i) {	
			img.create(n_rows, n_cols, CV_8UC1);
			for(int r = 0;r < n_rows; ++r) {
				for(int c = 0;c < n_cols; ++c) {
					uchar temp = 0;
					file.read((char*)&temp, sizeof(temp));
					img.at<uchar>(r, c) = temp;
				}
			}

			// string name = "Train/" + to_string(idx) + ".jpg";
			// imwrite(name, img);

			trainingData.push_back(img);
			img.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading train-images.idx3-ubyte complete!" << endl;
	return trainingData;
}

//读取用于测试的手写字体标签, 主要用来与predict的结果做对比;
vector<int> readTestLabels() {
	int idx = 0;
	Mat img;
	ifstream file;
	vector<int> testLabel;
	cout<<"Test..." << endl;
	file.open("t10k-labels.idx1-ubyte", ios::binary);
	
	if(!file.is_open()) {
		cout << "File Not Found!" << endl;
		exit(0);
	} else {
		int magic_number=0;
		int number_of_labels=0;
		
		file.read((char*)&magic_number,sizeof(magic_number)); 
		magic_number= reverseDigit(magic_number);

		file.read((char*)&number_of_labels,sizeof(number_of_labels));
		number_of_labels= reverseDigit(number_of_labels);
		
		cout << "No. of labels:" << number_of_labels << endl;

		for(long int i = 0; i < number_of_labels; ++i) {	
			unsigned char temp=0;
			file.read((char*)&temp,sizeof(temp));
			testLabel.push_back((int)temp);
		}
	}
	file.close();
	cout << "Reading t10k-labels.idx1-ubyte complete!" << endl;
	return testLabel;
}

//读取用于训练的手写字体标签, 主要用来标志predict的结果;
vector<int> readTrainLabels() {
	int idx = 0;
	Mat img;
	ifstream file;
	vector<int> trainingLabel;
	cout<<"Training..." << endl;
	file.open("train-labels.idx1-ubyte", ios::binary);
	
	if(!file.is_open()) {
		cout << "File Not Found!" << endl;
		exit(0);
	} else {
		int magic_number=0;
		int number_of_labels=0;
		
		file.read((char*)&magic_number,sizeof(magic_number)); 
		magic_number= reverseDigit(magic_number);

		file.read((char*)&number_of_labels,sizeof(number_of_labels));
		number_of_labels= reverseDigit(number_of_labels);
		
		cout << "No. of labels:" << number_of_labels << endl;

		for(long int i = 0; i < number_of_labels; ++i) {	
			unsigned char temp=0;
			file.read((char*)&temp, sizeof(temp));
			trainingLabel.push_back((int)temp);
		}
	}
	file.close();
	cout << "Reading train-labels.idx1-ubyte complete!" << endl;
	return trainingLabel;
}

vector<pair<Mat, int> > readTestSet() {
	vector<pair<Mat, int> > output;
	vector<Mat> testData = readTestImages();
	vector<int> testLabel = readTestLabels();
	assert(testLabel.size() == testData.size());
	HOGDescriptor hog(cvSize(28,28),cvSize(14,14),cvSize(1,1),cvSize(7,7),9);
	Size winStride = Size(1, 1);
	for (int i = 0; i < testLabel.size(); i++){
		Mat tmp = testData[i];
		vector<float> descriptors;
		hog.compute(tmp, descriptors, winStride);

		tmp.create(1, descriptors.size(), CV_32FC1);
		for (int j = 0; j < descriptors.size(); j++) {
			tmp.at<float>(0, j) = descriptors[j];
		}

		output.push_back(pair<Mat, int>(tmp, testLabel[i]));
	}
	cout << "finish reading testing data\n";
	return output;
}

pair<Mat, Mat> readTrainingSet() {
	pair<Mat, Mat> output;
	vector<Mat> trainData = readTrainImages();
	vector<int> trainLabels = readTrainLabels();
	Mat output_data, output_label;
	assert(trainData.size() == trainLabels.size());
	HOGDescriptor hog(cvSize(28,28),cvSize(14,14),cvSize(1,1),cvSize(7,7),9);
	for (int i = 0; i < trainData.size(); i++) {
		Mat tmp = trainData[i];
		vector<float> descriptors;

		Size winStride = Size(1, 1);

		hog.compute(tmp, descriptors, winStride);

        tmp.create(1, descriptors.size(), CV_32FC1);
        for (int j = 0; j < descriptors.size(); j++) {
        	tmp.at<float>(0, j) = descriptors[j];
        }

		output_data.push_back(tmp);
		output_label.push_back(trainLabels[i]);
	}
	output.first = output_data;
	output.second = output_label;

	cout << "finish reading training data\n";

	return output;
}

void train() {
	pair<Mat, Mat> trainData;
	trainData = readTrainingSet();
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	//params.kernel_type = CvSVM::SIGMOID;
	//params.kernel_type = CvSVM::RBF;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, ITERATIVE_COUNT, 1e-6);
	CvSVM SVM;
	cout << "data size: " << trainData.first.size() << " ";
	cout << "label size: " << trainData.second.size() << endl;
	SVM.train(trainData.first, trainData.second, Mat(), Mat(), params);
	string name = "svm-linear-kernel-" + to_string(ITERATIVE_COUNT);
	SVM.save(name.c_str());
}

vector<pair<int, int>> predict() {
	vector<pair<int, int>> results;
	int prediction;
	vector<pair<Mat, int> > testingData = readTestSet();
	CvSVM SVM;
	string name = "svm-linear-kernel-" + to_string(ITERATIVE_COUNT);
	//string name = "svm-train";
	SVM.load(name.c_str()); 
	ofstream fout("svm-res.txt", ios::app);
	fout << name << endl;
	for (int i = 0; i < testingData.size(); i++){
		prediction = SVM.predict(testingData[i].first);
		results.push_back(pair<int, int>(testingData[i].second, prediction));
	}
	fout.close();
	return results;
}

void calculateAccuracy() {
	vector<pair<int, int> > result = predict();

	int correct = 0;

	for (int i = 0; i < result.size(); i++) {
		if (result[i].first == result[i].second) {
			correct++;
		}
	}

	ofstream fout("svm-res.txt", ios::app | ios::out);
	cout << correct << endl;
	fout << correct << endl;

	cout << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	fout << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	fout.close();
}

int main(int argc, char ** argv) {
	time_t start = clock();
	train();
	time_t end = clock();
	ofstream fout("svm-res.txt", ios::app | ios::out);
	time_t prediction_start = clock();
	calculateAccuracy();
	time_t prediction_end = clock();
	fout << "cost===>" << double(end-start)/CLOCKS_PER_SEC << endl;
	fout << "prediction====>" << double(prediction_end-prediction_start)/CLOCKS_PER_SEC << endl << endl;
	fout.close();
	return 0;
}