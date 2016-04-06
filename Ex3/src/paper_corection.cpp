#include "paper_corection.h"
#include "global.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>
#include <cassert>
#include <fstream>
#include <cmath>

using namespace std;

PaperCorection::PaperCorection(double r, int t, int p) {
	this->rate = r;
	this->errorTheta = t;
	this->errorP = p;
}


vector<Position> PaperCorection::detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
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

    #ifdef PAPER_CORECTION_DEBUG
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

    #ifdef PAPER_CORECTION_DEBUG
    print_map(cluster);
    #endif

    #ifdef PAPER_CORECTION_DEBUG
    // 根据聚类的结果，选出投票数最高的四个点绘制结果图
    CImg<float> result(srcImg);
    multimap<int, Position>::reverse_iterator reverseIt = cluster.rbegin();
    for (int count = 1; reverseIt != cluster.rend() && count <= 7; reverseIt++, count++) {
    	Position p = reverseIt->second;
        // 对检测到的直线标记红色
    	draw_result(result, p.x, p.y, 0);
        result.display();
    }
    result.save_jpeg("result.jpg");
    #endif

    vector<Position> pos;
    for (reverseIt = cluster.rbegin(); reverseIt != cluster.rend(); reverseIt++) {
        pos.push_back(reverseIt->second);
    }

    #ifdef PAPER_CORECTION_DEBUG
    CImg<float> r(srcImg);
    vector<Position> newPos = detect_edge(pos);
    cout << "newPos====>" << newPos.size() << endl;
    for (int i = 0; i < newPos.size(); i++) {
        cout << "(" << newPos[i].x << "," << newPos[i].y << ")" << endl;
    }
    for (int i = 0; i < newPos.size(); i++) {
        draw_result(r, newPos[i].x, newPos[i].y, 0);
        r.display();
    }
    r.save_jpeg("r.jpg");
    #endif

    return detect_edge(pos);
}

vector<Position> PaperCorection::detect_edge(vector<Position> &pos) {
    assert(pos.size() >= 4);

    cout << "detect edge=====>" << pos.size() << endl;
    double thetaError = 10, verticalAngle = 90;
    vector<Position> res;
    // 默认投票数最高的点是正确的边缘
    Position p1 = pos[0], p2;
    res.push_back(p1);
    int p2Index = 0;
    // 找到与p1垂直的直线p2
    for (int i = 1; i < pos.size(); i++) {
        if ((abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-360) <= thetaError)) {
            continue;
        }
        if ((abs(p1.x-pos[i].x) >= 90-10 && abs(p1.x-pos[i].x) <= 90+10) ||
            (abs(abs(p1.x-pos[i].x)-180) >= 90-10 && abs(abs(p1.x-pos[i].x)-180) <= 90+10)) {
            p2 = pos[i];
            p2Index = i;
            break;
        }
    }
    for (int i = 1; i < pos.size(); i++) {
        if ((abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-360) <= thetaError)) {
            res.push_back(pos[i]);
            break;
        }
    }
    res.push_back(p2);
    for (int i = 1; i < pos.size(); i++) {
        if (i != p2Index && (abs(p2.x-pos[i].x) <= thetaError || abs(abs(p2.x-pos[i].x)-360) <= thetaError)) {
            res.push_back(pos[i]);
            break;
        }
    }
    return res;
}

vector<Position> PaperCorection::get_vertexs(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
    const CImg<float> &cannyImg) {
    vector<Position> pos = detect_edge(houghSpace, srcImg, cannyImg);
    assert(pos.size() == 4);
    vector<Position> ret;
    if (abs(sin(pos[0].x)) <= 0.001 || abs(sin(pos[1].x)) <= 0.001) {
        double x1 = pos[0].y / cos(pos[0].x), x2 = pos[1].y / cos(pos[1].x);
        double y1 = (pos[2].y-x1*cos(pos[2].x))/sin(pos[2].x), y2 = (pos[3].y-x1*cos(pos[3].x))/sin(pos[3].x),
               y3 = (pos[3].y-x2*cos(pos[3].x))/sin(pos[3].x), y4 = (pos[2].y-x2*cos(pos[2].x))/sin(pos[3].x);
        ret.push_back(Position(x1, y1, 0));
        ret.push_back(Position(x1, y2, 0));
        ret.push_back(Position(x2, y3, 0));
        ret.push_back(Position(x2, y4, 0));
    } else if (abs(cos(pos[0].x)) <= 0.001 || abs(cos(pos[1].x)) <= 0.001) {
        double y1 = pos[0].y / sin(pos[0].x), y2 = pos[1].y / sin(pos[1].x);
        double x1 = (pos[2].y-y1*sin(pos[2].x))/cos(pos[2].x), x2 = (pos[3].y-y2*sin(pos[3].x))/cos(pos[3].x),
               x3 = (pos[3].y-y2*sin(pos[3].x))/cos(pos[3].x), x4 = (pos[2].y-y1*sin(pos[2].x))/cos(pos[2].x);
        ret.push_back(Position(x1, y1, 0));
        ret.push_back(Position(x2, y1, 0));
        ret.push_back(Position(x3, y2, 0));
        ret.push_back(Position(x4, y2, 0));
    } else {
        cout << "else" << endl;
        double b1 = pos[0].y/sin(pos[0].x*PI/180), k1 = cos(pos[0].x*PI/180)/sin(pos[0].x*PI/180),
               b2 = pos[1].y/sin(pos[1].x*PI/180), k2 = cos(pos[1].x*PI/180)/sin(pos[1].x*PI/180),
               b3 = pos[2].y/sin(pos[2].x*PI/180), k3 = cos(pos[2].x*PI/180)/sin(pos[2].x*PI/180),
               b4 = pos[3].y/sin(pos[3].x*PI/180), k4 = cos(pos[3].x*PI/180)/sin(pos[3].x*PI/180);
        double x1 = (b3-b1)/(k3-k1), y1 = b1-k1*x1,
               x2 = (b4-b1)/(k4-k1), y2 = b1-k1*x2,
               x3 = (b4-b2)/(k4-k2), y3 = b2-k2*x3,
               x4 = (b3-b2)/(k3-k2), y4 = b2-k2*x4;
        ret.push_back(Position(x1, y1, 0));
        ret.push_back(Position(x2, y2, 0));
        ret.push_back(Position(x3, y3, 0));
        ret.push_back(Position(x4, y4, 0));
    }

    #ifdef PAPER_CORECTION_DEBUG
    CImg<float> temp(srcImg);
    for (int i = 0; i < ret.size(); i++) {
        cout << "(" << ret[i].x << "," << ret[i].y << ")\n";
        draw_point(temp, ret[i]);
        temp.display();
    }
    temp.save_jpeg("temp.jpg");
    #endif
    return ret;
}

/**
* 绘制最终的检测结果
* params: 
* img: 需要绘制的图片
* theta, p: A4纸边缘直线的参数
* channel: 绘制的色彩通道，主要是为了使标记的结果更明显
*/
void PaperCorection::draw_result(CImg<float> &img, int theta, int p, int channel) {
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
void PaperCorection::draw_line(CImg<float> &img, int theta, int p) {
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
void PaperCorection::print_map(multimap<int, Position> &cluster) {
    multimap<int, Position>::iterator it;
    for (it = cluster.begin(); it != cluster.end(); ++it) {
        Position p = it->second;
        cout << it->first << "   (" << p.x << ", " << p.y << ")  " << endl;
    }
    cout << endl;
}

void PaperCorection::draw_point(CImg<float> &img, const Position &p) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (sqrt((i-p.x)*(i-p.x)+(j-p.y)*(j-p.y)) < 20) {
                img(i, j, 0, 0) = 0xff;
            }
        }
    }
}

