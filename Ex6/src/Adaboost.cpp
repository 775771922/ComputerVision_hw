#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <ctime>

using namespace cv;
using namespace std;

int weak_classifiers = 5;
double weight_trim_rate = 0.95;
int max_depth_of_trees = 1;

int reverseDigit(int i) {
	unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((int)c1 << 24) + ((int)c2 << 16) + ((int)c3 << 8) + c4;
}

//读取用于训练的手写字体图片;
Mat readTrainImages() {
	Mat trainingImages;
	int idx = 0;
	Mat img;
	ifstream file;
	cout << "Reading Train Images..." << endl;
	file.open("train-images.idx3-ubyte", ios::binary);
	if(!file.is_open()) {
		cout << "File train-images.idx3-ubyte Not Found!" << endl;
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

		cout << "Number of images: " << number_of_images << endl;

		for(long int i = 0;i < number_of_images; ++i) {	
			img.create(n_rows, n_cols, CV_8UC1);
			for(int r = 0;r < n_rows; ++r) {
				for(int c = 0;c < n_cols; ++c) {
					uchar temp = 0;
					file.read((char*)&temp, sizeof(temp));
					img.at<uchar>(r, c) = temp;
				}
			}
			img.convertTo(img, CV_32F);
			img = img.reshape(0, 1);
			trainingImages.push_back(img);
			img.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading train-images.idx3-ubyte complete!" << endl << endl;
	return trainingImages;
}

//读取用于测试的手写字体图片;
vector<Mat> readTestImages() {
	vector<Mat> testingImages;
	int idx = 0;
	Mat img;
	ifstream file;
	cout<<"Reading Test Images..." << endl;
	file.open("t10k-images.idx3-ubyte", ios::binary);
	if(!file.is_open()) {
		cout << "File t10k-images.idx3-ubyte Not Found!" << endl;
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

		cout << "Number of images: " << number_of_images << endl;

		for(long int i = 0;i < number_of_images; ++i) {	
			img.create(n_rows, n_cols, CV_8UC1);
			for(int r = 0;r < n_rows; ++r) {
				for(int c = 0;c < n_cols; ++c) {
					uchar temp = 0;
					file.read((char*)&temp, sizeof(temp));
					img.at<uchar>(r, c) = temp;
				}
			}
			img.convertTo(img, CV_32F);
			img = img.reshape(0, 1);
			testingImages.push_back(img);
			img.release();
			idx++;
		}
	}
	file.close();
	cout << "Reading t10k-images.idx3-ubyte complete!" << endl << endl;
	return testingImages;
}

//读取用于训练的手写字体标签, 主要用来标志predict的结果;
vector<int> readTrainingLabels() {
	vector<int> trainingLabels;
	int idx = 0;
	Mat img;
	ifstream file;
	cout<<"Reading Train Labels..." << endl;
	file.open("train-labels.idx1-ubyte", ios::binary);
	
	
	if(!file.is_open()) {
		cout << "File train-labels.idx1-ubyte Not Found!" << endl;
		exit(0);
	} else {
		int magic_number=0;
		int number_of_labels=0;
		
		file.read((char*)&magic_number,sizeof(magic_number)); 
		magic_number= reverseDigit(magic_number);

		file.read((char*)&number_of_labels,sizeof(number_of_labels));
		number_of_labels= reverseDigit(number_of_labels);
		
		cout << "Number of labels: " << number_of_labels << endl;

		for(long int i = 0; i < number_of_labels; ++i) {	
			unsigned char temp=0;
			file.read((char*)&temp, sizeof(temp));
			trainingLabels.push_back((int)temp);
		}
	}
	file.close();
	cout << "Reading train-labels.idx1-ubyte complete!" << endl << endl;
	return trainingLabels;
}

//读取用于测试的手写字体标签, 主要用来与predict的结果做对比;
vector<int> readTestLabels() {
	vector<int> testingLabels;
	int idx = 0;
	Mat img;
	ifstream file;
	ofstream file2;
	cout<<"Reading Test Labels..." << endl;
	file.open("t10k-labels.idx1-ubyte", ios::binary);
	
	if(!file.is_open()) {
		cout << "File t10k-labels.idx1-ubyte Not Found!" << endl;
		exit(0);
	} else {
		int magic_number=0;
		int number_of_labels=0;
		
		file.read((char*)&magic_number,sizeof(magic_number)); 
		magic_number= reverseDigit(magic_number);

		file.read((char*)&number_of_labels,sizeof(number_of_labels));
		number_of_labels= reverseDigit(number_of_labels);
		
		cout << "Number of labels: " << number_of_labels << endl;

		for(long int i = 0; i < number_of_labels; ++i) {	
			unsigned char temp=0;
			file.read((char*)&temp,sizeof(temp));
			testingLabels.push_back((int)temp);
		}
	}
	file.close();
	cout << "Reading t10k-labels.idx1-ubyte complete!" << endl << endl;
	return testingLabels;
}


void train(Mat &trainingImages, vector<int> &trainingLabels) {

    Mat two_class_labels[10];

	CvBoostParams params;
	params.boost_type = CvBoost::REAL;
	params.split_criteria = CvBoost::DEFAULT;
	params.weak_count = weak_classifiers;
	params.weight_trim_rate = weight_trim_rate;
	params.max_depth = max_depth_of_trees;
	params.use_surrogates = false;

	CvBoost boost;
	cout << "Images Size: " << trainingImages.size() << " ";
	cout << "Labels Size: " << trainingLabels.size() << endl;
	cout << "Training..." << endl;

	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < trainingLabels.size(); j++) {
			if(trainingLabels[j] == i) {
				two_class_labels[i].push_back(i);
			} else {
				two_class_labels[i].push_back(-1);
			}
		}
		cout << "Training class " << i << endl;
		boost.train(trainingImages, CV_ROW_SAMPLE, two_class_labels[i], Mat(), Mat(), Mat(), Mat(), params, false);
		string tmp = "train-boost-classifiers-" + to_string(weak_classifiers) + "-class-" + to_string(i);
		const char* name = tmp.c_str();
		boost.save(name);
	}
	cout << endl;
}

vector<pair<int, int>> predict(vector<Mat> &testingImages, vector<int> &testingLabels) {
	vector<pair<int, int>> results;
	int prediction;
	CvBoost boost;

	for (int i = 0; i < testingLabels.size(); i++) {
		int j;
		for (j = 0; j < 10; j++) {
			string tmp = "train-boost-classifiers-" + to_string(weak_classifiers) + "-class-" + to_string(j);
			const char* name = tmp.c_str();
			boost.load(name);
			prediction = boost.predict(testingImages[i]);
			if (prediction == testingLabels[i]) {
				results.push_back(pair<int, int>(testingLabels[i], prediction));
				break;
			}
		}
		if (j == 10) {   // 预测失败
			results.push_back(pair<int, int>(testingLabels[i], -1));
		}
	}
	return results;
}

void calculateAccuracy(vector<Mat> &testingImages, vector<int> &testingLabels) {
	ofstream fout("adaboost.txt", ios::app | ios::out);
	vector<pair<int, int> > result = predict(testingImages, testingLabels);
	int correct = 0;
	for (int i = 0; i < result.size(); i++) {
		if (result[i].first == result[i].second) {
			correct++;
		}
	}

	cout << correct << endl;

	cout << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	fout << "weak_classifiers===>" << weak_classifiers << endl;
	fout << correct << endl;
	fout << "Accuracy:" << (double)correct * 100.0 / (double)result.size() << endl;
	fout.close();
}

int main(int argc, char *argv[]) {

	Mat trainingImages = readTrainImages();
	vector<Mat> testingImages = readTestImages();
	vector<int> testingLabels = readTestLabels();
	vector<int> trainingLabels = readTrainingLabels();

	for (weak_classifiers = 80; weak_classifiers <= 80; weak_classifiers += 10) {
	    //weak_classifiers = 70;
	    time_t start = clock();
		train(trainingImages, trainingLabels);
		time_t end = clock();

		calculateAccuracy(testingImages, testingLabels);
		time_t prediction_end = clock();
		ofstream fout("adaboost.txt", ios::app | ios::out);
		cout << "cost====>" << double(end-start)/CLOCKS_PER_SEC << endl;
		cout << "predict time===>" << double(prediction_end-end)/CLOCKS_PER_SEC << endl;
		fout << "cost====>" << double(end-start)/CLOCKS_PER_SEC << endl;
		fout << "predict time===>" << double(prediction_end-end)/CLOCKS_PER_SEC << endl;
		fout.close();
	}

	trainingImages.release();
	testingImages.clear();
	testingLabels.clear();
	trainingLabels.clear();

	return 0;
}