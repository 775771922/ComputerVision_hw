#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <cassert>
#include "TrainingSet.h"

using namespace cv;
using namespace std;

vector<Mat> trainingData;
vector<int> trainingLabel;
vector<Mat> testData;
vector<int> testLabel;

ofstream display;

int reverseDigit(int i) {
	unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((int)c1 << 24) + ((int)c2 << 16) + ((int)c3 << 8) + c4;
}

//读取用于测试的手写字体图片;
void readTestImages() {
	int idx = 0;
	Mat img;
	ifstream file;
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

			string imgName ="Test/"+ to_string(idx) + ".jpg";
			img = img.reshape(0, 1);
			testData.push_back(img);
			//imwrite(imgName, img);
			img.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading t10k-images.idx3-ubyte complete!" << endl;
}

//读取用于训练的手写字体图片;
void readTrainImages() {
	int idx = 0;
	Mat img;
	ifstream file;
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

			string imgName = "Train/"+ to_string(idx) + ".jpg";
			//Mat tmp = img;
			//tmp.convertTo(tmp, CV_32F);
			//testData.push_back(tmp);
			//imwrite(imgName, img);
			img = img.reshape(0, 1);
			trainingData.push_back(img);
			img.release();
			//tmp.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading train-images.idx3-ubyte complete!" << endl;
}

//读取用于测试的手写字体标签, 主要用来与predict的结果做对比;
void readTestLabels() {
	int idx = 0;
	Mat img;
	ifstream file;
	//ofstream file2;
	cout<<"Test..." << endl;
	file.open("t10k-labels.idx1-ubyte", ios::binary);
	//file2.open("testLabels.txt");
	
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
			//file2 << (int)temp << endl;
			testLabel.push_back((int)temp);
		}
	}
	file.close();
	//file2.close();
	cout << "Reading t10k-labels.idx1-ubyte complete!" << endl;
}

//读取用于训练的手写字体标签, 主要用来标志predict的结果;
void readTrainLabels() {
	int idx = 0;
	Mat img;
	ifstream file;
	//ofstream file2;
	cout<<"Training..." << endl;
	file.open("train-labels.idx1-ubyte", ios::binary);
	//file2.open("trainLabels.txt");
	
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
			//file2 << (int)temp << endl;
			trainingLabel.push_back((int)temp);
		}
	}
	file.close();
	//file2.close();
	cout << "Reading train-labels.idx1-ubyte complete!" << endl;
}

vector<TrainingSet> readTestSet() {
	vector<TrainingSet> output;
	readTestImages();
	readTestLabels();
	for (int i = 0; i < testLabel.size(); i++){
		Mat tmp = testData[i];
		tmp.convertTo(tmp, CV_32F);
		output.push_back(TrainingSet(tmp, testLabel[i]));
	}
	return output;
}

pair<Mat, Mat> readTrainingSet() {
	pair<Mat, Mat> output;
	readTrainImages();
	readTrainLabels();
	Mat output_data, output_label;
	assert(trainingData.size() == trainingLabel.size());
	for (int i = 0; i < trainingData.size(); i++){
		Mat tmp = trainingData[i];
		tmp.convertTo(tmp, CV_32F);
		output_data.push_back(tmp);
		output_label.push_back(trainingLabel[i]);
	}
	//output_data.convertTo(output_data, CV_32F);
	output.first = output_data;
	output.second = output_label;

	return output;
}

void train() {
	pair<Mat, Mat> trainData;
	trainData = readTrainingSet();
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	CvSVM SVM;
	cout << "data size: " << trainData.first.size() << " ";
	cout << "label size: " << trainData.second.size() << endl;
	SVM.train(trainData.first, trainData.second, Mat(), Mat(), params);
	SVM.save("helloworld");
}

vector<pair<int, int>> predict() {
	vector<TrainingSet> testingData;
	vector<pair<int, int>> results;
	int prediction;
	testingData = readTestSet();
	CvSVM SVM;
	SVM.load("helloworld");
	display.open("Result.txt", ios::out);
	for (int i = 0; i < testingData.size(); i++){
		prediction = SVM.predict(testingData[i].data);
		results.push_back(pair<int, int>(testingData[i].label, prediction));
		cout << testingData[i].label << " " << prediction << endl;
		display << testingData[i].label << " " << prediction << endl;
	}
	display.close();
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

	display.open("Result.txt", ios::app | ios::out);
	cout << correct << endl;
	display << correct << endl;

	cout << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	display << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	display.close();
}

int main(int argc, char ** argv) {

	//readTrainImages();
	//cout << "Reading train-images.idx3-ubyte complete!" << endl;

	//readTestImages();
	//cout << "Reading t10k-images.idx3-ubyte complete!" << endl;

	//readTrainLabels();
	//cout << "Reading train-labels.idx1-ubyte complete!" << endl;

	//readTestLabels();
	//cout << "Reading t10k-labels.idx1-ubyte complete!" << endl;

	train();
	calculateAccuracy();


	trainingData.clear();
	trainingLabel.clear();
	testData.clear();
	testLabel.clear();
	vector<Mat> trainingData;
	vector<int>().swap(trainingLabel);
	vector<Mat>().swap(testData);
	vector<int>().swap(testLabel);
	return 0;
}