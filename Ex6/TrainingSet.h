#include <opencv2\opencv.hpp>
using namespace cv;

class TrainingSet {
	public:
		Mat data;
		int label;
		TrainingSet(Mat data, int label) {
			this->data = data;
			this->label = label;
		}
		
};