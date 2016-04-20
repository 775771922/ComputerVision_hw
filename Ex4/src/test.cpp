#include "image_stitch.h"
#include <iostream>
#include <string>
#include <cstring>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;

vector<string> get_files_from_dir(string dir);

int main(int argc, char** argv) {
	vector<string> files;

	if (argc <= 1) {
		return 0;
	} else if (argc == 2) {
		cout << "argv[1]===>" << argv[1] << endl;
    	string file(argv[1]);
		size_t dot_index = file.find(".");

		if (dot_index == string::npos) {
			files = get_files_from_dir(file);
		} else {
			cout << "only one image\n";
			return 0;
		}
    } else {
    	for (int i = 1; i < argc; i++) {
    		files.push_back(string(argv[i]));
    	}
    }



	int noctaves = 5, nlevels = 3, o_min = 0;
	ImageStitch imageStitch(noctaves, nlevels, o_min);

	vector<CImg<float> > imgs;
	for (int i = 0; i < files.size(); i++) {
		//CImg<float> img(files[i]);
		cout << "filename====>" << files[i] << endl;
		imgs.push_back(CImg<float>(files[i].c_str()));
		//imgs[i].display();
	}
	// CImg<float> img1(argv[1]);
	// CImg<float> img2(argv[2]);
    // CImg<float> res = imageStitch.image_stitch(img1, img2);
    CImg<float> res = imageStitch.image_stitch(imgs);
	res.display();
	res.save_jpeg("res.jpg");

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

        string postfix[4] = {".bmp", ".jpg", ".jpeg", ".png"};
		for (int i = 0; i < 4; i++) {
			if (file_path.find(postfix[i]) != string::npos) {
				files.push_back(file_path);
			}
		}
	}

	closedir(dir);
	return files;
}