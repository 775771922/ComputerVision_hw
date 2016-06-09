#g++ -std=c++11 `pkg-config --cflags --libs opencv` Adaboost.cpp -o boost
g++ -std=c++11 `pkg-config --cflags --libs opencv` svm_hog.cpp -o svm_hog