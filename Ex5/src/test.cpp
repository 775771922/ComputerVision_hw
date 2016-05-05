#include "image_segmentation.h"

#include <iostream>
#include <string>
#include <cstdio>
#include <cstring>
#include <time.h>
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
    	string file(argv[1]);
		size_t dot_index = file.find(".");

		if (dot_index == string::npos) {
			files = get_files_from_dir(file);
		} else {
			files.push_back(string(argv[1]));
		}
    } else {
    	for (int i = 1; i < argc; i++) {
    		files.push_back(string(argv[i]));
    	}
    }

	ImageSeg<unsigned char> imageSeg;
	vector<CImg<unsigned char> > imgs;
	for (int i = 0; i < files.size(); i++) {
		cout << "file===>" << i << " " << files[i] << endl;
		imgs.push_back(CImg<unsigned char>(files[i].c_str()));
		
	}

    vector<CImg<unsigned char> > ret;
	ret = imageSeg.segment_images(imgs);
	for (int i = 0; i < ret.size(); i++) {
		ret[i].display();
		string path = files[i].substr(files[i].find_last_of("/\\")+1, files[i].length());
		path = "image/" + path;
		cout << "path==>" << path << endl;

		ret[i].save_jpeg(path.c_str());
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
		for (int i = 0; i < 4; i++) {
			if (file_path.find(postfix[i]) != string::npos) {
				files.push_back(file_path);
			}
		}
	}

	closedir(dir);
	return files;
}