#include "edge_dect.h"
#include "global.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>
#include <fstream>

EdgeDetect::EdgeDetect(double r, int t, int p) {
	this->rate = r;
	this->errorTheta = t;
	this->errorP = p;
}


CImg<float> EdgeDetect::detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
	const CImg<float> &cannyImg) {
    vector<Position> v;
    for (int i = 0; i < houghSpace.width(); i++) {
        for (int j = 0; j < houghSpace.height(); j++) {
            if (houghSpace(i, j, 0, 0) <= 0) {
                continue;
            }
            v.push_back(Position(i, j, houghSpace(i, j, 0, 0)));
        }
    }
    sort(v.begin(), v.end(), cmp);

    // 对投票数制定一个阈值，小于这个值的舍弃
    int threshold = v[0].sum * rate;
    for (int i = 0; i < v.size(); i++) {
        if (v[i].sum < threshold) {
            v.erase(v.begin()+i, v.end());
            break;
        }
    }

    #ifdef EDGE_DECT_DEBUG
    int width = cannyImg.width();
    int height = cannyImg.height();
    CImg<float> o(width, height, 1, 1, 0);
    ofstream fout("set.txt");
    for (int i = 0; i < v.size(); i++) {
        Position p = v[i];
        fout << "(" << p.x << ", " << p.y << ")" << " " << p.sum << "\n";
        draw_line(o, p.x, p.y);
    }
    o.save_jpeg("o.jpg");
    #endif

    // 对hough空间中剩余的点按照误差值进行聚类，聚类后的点放在cluster里面
    multimap<int, Position> cluster;
    multimap<int, Position>::iterator it;
    for (int i = 0; i < v.size(); ++i) {
        if (cluster.size() == 0) {
            cluster.insert(make_pair(v[i].sum, v[i]));
            continue;
        }
        bool flag = false;
        for (it = cluster.begin(); it != cluster.end(); ++it) {
            Position p = it->second;
            // 在errorTheta和errorP误差内的点进行聚类
            // 注意考虑角度的周期性
            if ((abs(p.x-v[i].x) <= errorTheta || abs(abs(p.x-v[i].x) - 360) <= errorTheta) 
            	&& abs(p.y-v[i].y) <= errorP) {
            	#ifdef EDGE_DECT_DEBUG
                fout << "merge===>*it :(" << v[i].x << ", " << v[i].y << ")\n";
                fout << "     position:(" << p.x << ", " << p.y << ")\n";
                #endif
                cluster.erase(it);
                if (abs(abs(p.x-v[i].x) - 360) <= errorTheta) {
                    Position newPos((p.x+v[i].x+360)/2, (p.y+v[i].y)/2, (p.sum+v[i].sum)/2);    
                    cluster.insert(make_pair(newPos.sum, newPos));
                } else {
                    Position newPos((p.x+v[i].x)/2, (p.y+v[i].y)/2, (p.sum+v[i].sum)/2);
                    cluster.insert(make_pair(newPos.sum, newPos));
                }
                flag = true;
                break;
            }
        }
        // 没找到能够聚类的点，则将点放入cluster中
        if (!flag) {
            cluster.insert(make_pair(v[i].sum, v[i]));
        }
    }

    #ifdef EDGE_DECT_DEBUG
    print_map(cluster);
    #endif

    // 根据聚类的结果，选出投票数最高的四个点绘制结果图
    CImg<float> result(srcImg);
    multimap<int, Position>::reverse_iterator reverseIt = cluster.rbegin();
    for (int count = 1; reverseIt != cluster.rend() && count <= 4; reverseIt++, count++) {
    	Position p = reverseIt->second;
        // 对检测到的直线标记红色
    	draw_result(result, p.x, p.y, 0);
    }

    return result;
}

/**
* 绘制最终的检测结果
* params: 
* img: 需要绘制的图片
* theta, p: A4纸边缘直线的参数
* channel: 绘制的色彩通道，主要是为了使标记的结果更明显
*/
void EdgeDetect::draw_result(CImg<float> &img, int theta, int p, int channel) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double temp_p = (double)i * cos(PI*(double)theta/180) + (double)j * sin(PI*(double)theta/180);
            if (p >= temp_p-3 && p <= temp_p+3) {
                img(i, j, 0, channel) = 0xff;
            }
        }
    }
} 

// 调试用
void EdgeDetect::draw_line(CImg<float> &img, int theta, int p) {
    int width = img.width();
    int height = img.height();
    int spectrum = img.spectrum();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double temp_p = (double)i * cos(PI*(double)theta/180) + (double)j * sin(PI*(double)theta/180);
            if (p >= temp_p-3 && p <= temp_p+3) {
                for (int channel = 0; channel < spectrum; channel++) {
                    img(i, j, 0, channel) = 0xff;
                }
            }
        }
    }
} 

// 调试用
void EdgeDetect::print_map(multimap<int, Position> &cluster) {
    multimap<int, Position>::iterator it;
    for (it = cluster.begin(); it != cluster.end(); ++it) {
        Position p = it->second;
        cout << it->first << "   (" << p.x << ", " << p.y << ")  " << endl;
    }
    cout << endl;
}

