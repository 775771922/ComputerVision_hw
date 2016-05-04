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

int main() {
	
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