#include "paper_corection.h"
#include "RotateOP.h"
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

    // #ifdef PAPER_CORECTION_DEBUG
    // int width = cannyImg.width();
    // int height = cannyImg.height();
    // CImg<float> o(width, height, 1, 1, 0);
    // ofstream fout("set.txt");
    // for (int i = 0; i < v.size(); i++) {
    //     Position p = v[i];
    //     fout << "(" << p.x << ", " << p.y << ")" << " " << p.sum << "\n";
    //     draw_line(o, p.x, p.y);
    // }
    // o.save_jpeg("o.jpg");
    // #endif

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

    // #ifdef PAPER_CORECTION_DEBUG
    // print_map(cluster);
    // #endif

    // #ifdef PAPER_CORECTION_DEBUG
    // // 根据聚类的结果，选出投票数最高的四个点绘制结果图
    // CImg<float> result(srcImg);
     multimap<int, Position>::reverse_iterator reverseIt = cluster.rbegin();
    // for (int count = 1; reverseIt != cluster.rend() && count <= 7; reverseIt++, count++) {
    // 	Position p = reverseIt->second;
    //     // 对检测到的直线标记红色
    // 	draw_result(result, p.x, p.y, 0);
    //     result.display();
    // }
    // result.save_jpeg("result.jpg");
    // #endif

    vector<Position> pos;
    for (reverseIt = cluster.rbegin(); reverseIt != cluster.rend(); reverseIt++) {
        pos.push_back(reverseIt->second);
    }

    // #ifdef PAPER_CORECTION_DEBUG
    // CImg<float> r(srcImg);
    // vector<Position> newPos = detect_edge(pos);
    // cout << "newPos====>" << newPos.size() << endl;
    // for (int i = 0; i < newPos.size(); i++) {
    //     cout << "(" << newPos[i].x << "," << newPos[i].y << ")" << endl;
    // }
    // for (int i = 0; i < newPos.size(); i++) {
    //     draw_result(r, newPos[i].x, newPos[i].y, 0);
    //     r.display();
    // }
    // r.save_jpeg("r.jpg");
    // #endif

    return detect_edge(pos);
}

vector<Position> PaperCorection::detect_edge(vector<Position> &pos) {
    assert(pos.size() >= 4);

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
    double x1 = -1, x2 = -1, x3 = -1, x4 = -1, y1 = -1, y2 = -1, y3 = -1, y4 = -1;
    double b1 = pos[0].y/sin(pos[0].x*PI/180), k1 = cos(pos[0].x*PI/180)/sin(pos[0].x*PI/180),
           b2 = pos[1].y/sin(pos[1].x*PI/180), k2 = cos(pos[1].x*PI/180)/sin(pos[1].x*PI/180),
           b3 = pos[2].y/sin(pos[2].x*PI/180), k3 = cos(pos[2].x*PI/180)/sin(pos[2].x*PI/180),
           b4 = pos[3].y/sin(pos[3].x*PI/180), k4 = cos(pos[3].x*PI/180)/sin(pos[3].x*PI/180);

    // pos[0]-pos[3]分别对应l1,l2,l3,l4, 函数detect_edge已经确保l1//l2, l3//l4
    if (abs(pos[0].x)<5 || abs(pos[0].x-180)<5) { // l1垂直
        x1 = x2 = pos[0].y / cos(pos[0].x*PI/180);
        y1 = (pos[2].y - x1*cos(pos[2].x*PI/180)) / sin(pos[2].x*PI/180);
        y2 = (pos[3].y - x1*cos(pos[3].x*PI/180)) / sin(pos[3].x*PI/180);
    }
    if (abs(pos[1].x)<5 || abs(pos[1].x-180)<5) { // l2垂直
        x3 = x4 = pos[1].y / cos(pos[1].x*PI/180);
        y3 = (pos[3].y - x3*cos(pos[3].x*PI/180)) / sin(pos[3].x*PI/180);
        y4 = (pos[2].y - x3*cos(pos[2].x*PI/180)) / sin(pos[2].x*PI/180);
    }
    if (abs(pos[2].x)<5 || abs(pos[2].x-180)<5) {// l3垂直
        x1 = x4 = pos[2].y / cos(pos[2].x*PI/180);
        y1 = (pos[0].y - x1*cos(pos[0].x*PI/180)) / sin(pos[0].x*PI/180);
        y4 = (pos[1].y - x1*cos(pos[1].x*PI/180)) / sin(pos[1].x*PI/180);
    }
    if (abs(pos[3].x)<5 || abs(pos[3].x-180)<5) {// l4垂直
        x2 = x3 = pos[3].y / cos(pos[3].x*PI/180);
        y2 = (pos[0].y - x2*cos(pos[0].x*PI/180)) / sin(pos[0].x*PI/180);
        y3 = (pos[1].y - x2*cos(pos[1].x*PI/180)) / sin(pos[1].x*PI/180);    
    }
    if (x1 == -1) {
        x1 = (b1-b3) / (k1-k3);
        y1 = b1 - k1*x1;
    }
    if (x2 == -1) {
        x2 = (b1-b4) / (k1-k4);
        y2 = b1 - k1*x2;
    }
    if (x3 == -1) {
        x3 = (b2-b4) / (k2-k4);
        y3 = b2 - k2*x3;
    }
    if (x4 == -1) {
        x4 = (b2-b3) / (k2-k3);
        y4 = b2 - k2*x4;
    }

    ret.push_back(Position(x1, y1));
    ret.push_back(Position(x2, y2));
    ret.push_back(Position(x3, y3));
    ret.push_back(Position(x4, y4));

    return ret;
}



vector<Position> PaperCorection::get_standard_vertexs(vector<Position> &v, bool &isVertical) {
    assert(v.size() == 4);
    vector<Position> ret(4, Position(-1,-1));
    Position p = v[0];
    int minIndex = 0;
    for (int i = 1; i < v.size(); i++) {
        if ((v[i].x*v[i].x+v[i].y*v[i].y) < p.x*p.x+p.y*p.y) {
            minIndex = i;
            p = v[i];
        }
    }
    
    int neighbor1 = minIndex+1 >= v.size() ? 0 : minIndex+1,
        neighbor2 = minIndex-1 < 0 ? v.size()-1 : minIndex-1;
    double l1 = sqrt((v[minIndex].x-v[neighbor1].x)*(v[minIndex].x-v[neighbor1].x)+
        (v[minIndex].y-v[neighbor1].y)*(v[minIndex].y-v[neighbor1].y));
    double l2 = sqrt((v[minIndex].x-v[neighbor2].x)*(v[minIndex].x-v[neighbor2].x)+
        (v[minIndex].y-v[neighbor2].y)*(v[minIndex].y-v[neighbor2].y));
    if (l1 >= l2) {
        l2 = l1/sqrt(2);
    } else {
        l1 = l2/sqrt(2);
    }

    int neighbor3 = (minIndex+2) % v.size();


    if ((v[neighbor3].x-v[neighbor1].x)*(v[neighbor3].x-v[neighbor2].x) < 0) { // neighbor1,2在3的两侧
        cout << "if" << endl;
        assert(v[neighbor1].x != v[neighbor2].x);
        if (v[neighbor1].x > v[neighbor2].x) {
            ret[neighbor1] = Position(v[minIndex].x+l1, v[minIndex].y);
            ret[neighbor2] = Position(v[minIndex].x, v[minIndex].y+l2);
            ret[neighbor3] = Position(v[minIndex].x+l1, v[minIndex].y+l2);
        } else {
            ret[neighbor1] = Position(v[minIndex].x, v[minIndex].y+l1);
            ret[neighbor2] = Position(v[minIndex].x+l2, v[minIndex].y);
            ret[neighbor3] = Position(v[minIndex].x+l2, v[minIndex].y+l1);
        }
    } else if ((v[neighbor3].x-v[neighbor1].x)*(v[neighbor3].x-v[neighbor2].x) > 0) { // neighbor1,2在3的同侧
        cout << "else if" << endl;
        assert(v[neighbor1].y != v[neighbor2].y);
        if (v[neighbor1].y < v[neighbor2].y) {
            ret[neighbor1] = Position(v[minIndex].x+l1, v[minIndex].y);
            ret[neighbor2] = Position(v[minIndex].x, v[minIndex].y+l2);
            ret[neighbor3] = Position(v[minIndex].x+l1, v[minIndex].y+l2);
        } else {
            ret[neighbor1] = Position(v[minIndex].x, v[minIndex].y+l1);
            ret[neighbor2] = Position(v[minIndex].x+l2, v[minIndex].y);
            ret[neighbor3] = Position(v[minIndex].x+l2, v[minIndex].y+l1);
        }
    } else { // neighbor1或2与3在同一垂线上
        cout << "else" << endl;
        if (v[neighbor1].x == v[neighbor3].x) {
            ret[neighbor1] = Position(v[minIndex].x+l1, v[minIndex].y);
            ret[neighbor2] = Position(v[minIndex].x, v[minIndex].y+l2);
            ret[neighbor3] = Position(v[minIndex].x+l1, v[minIndex].y+l2);            
        } else {
            ret[neighbor1] = Position(v[minIndex].x, v[minIndex].y+l1);
            ret[neighbor2] = Position(v[minIndex].x+l2, v[minIndex].y);
            ret[neighbor3] = Position(v[minIndex].x+l2, v[minIndex].y+l1);         
        }
    }
    ret[minIndex] = v[minIndex];

    isVertical = true;
    if (l1 >= l2) {
        if (ret[neighbor1].x > ret[minIndex].x) {
            isVertical = false;
        }
    } else {
        if (ret[neighbor2].x > ret[minIndex].x) {
            isVertical = false;
        }
    }

    return ret;
}

CImg<float> PaperCorection::image_wrap(vector<Position> &v, vector<Position> &s, const CImg<float> &srcImg, 
    int &offsetX, int &offsetY) {
    double a[4], b[4];
    double c[3], d[3];
    double x0 = v[0].x, x1 = v[1].x, x2 = v[2].x, x3 = v[3].x,
           y0 = v[0].y, y1 = v[1].y, y2 = v[2].y, y3 = v[3].y,
           u0 = s[0].x, u1 = s[1].x, u2 = s[2].x, u3 = s[3].x,
           v0 = s[0].y, v1 = s[1].y, v2 = s[2].y, v3 = s[3].y;

    // 防止出现除数为0的情况
    if (x0 == x2) {
        x2 = x0 + 0.01;
    }
    if (x0 == x3) {
        x3 = x3 + 0.01;
    }

    c[0] = y0-y1-(y0-y2)*(x0-x1)/(x0-x2);
    c[1] = x0*y0-x1*y1-(x0*y0-x2*y2)*(x0-x1)/(x0-x2);
    c[2] = u0-u1-(u0-u2)*(x0-x1)/(x0-x2);
    d[0] = y0-y1-(y0-y3)*(x0-x1)/(x0-x3);
    d[1] = x0*y0-x1*y1-(x0*y0-x3*y3)*(x0-x1)/(x0-x3);
    d[2] = u0-u1-(u0-u3)*(x0-x1)/(x0-x3);

    a[3] = (c[2]*d[0]-d[2]*c[0])/(c[1]*d[0]-d[1]*c[0]);
    a[2] = (c[2]-c[1]*a[3])/c[0];
    a[1] = (u0-u1-(x0*y0-x1*y1)*a[3]-(y0-y1)*a[2])/(x0-x1);
    a[0] = u0-x0*y0*a[3]-y0*a[2]-x0*a[1];

    double e[3], f[3];
    e[0] = y0-y1-(y0-y2)*(x0-x1)/(x0-x2); 
    e[1] = x0*y0-x1*y1-(x0*y0-x2*y2)*(x0-x1)/(x0-x2);
    e[2] = v0-v1-(v0-v2)*(x0-x1)/(x0-x2);
    f[0] = y0-y1-(y0-y3)*(x0-x1)/(x0-x3);
    f[1] = x0*y0-x1*y1-(x0*y0-x3*y3)*(x0-x1)/(x0-x3);
    f[2] = v0-v1-(v0-v3)*(x0-x1)/(x0-x3);

    b[3] = (e[2]*f[0]-e[0]*f[2])/(e[1]*f[0]-e[0]*f[1]);
    b[2] = (e[2]-e[1]*b[3])/e[0];
    b[1] = (v0-v1-(x0*y0-x1*y1)*b[3]-(y0-y1)*b[2])/(x0-x1);
    b[0] = v0-x0*y0*b[3]-y0*b[2]-x0*b[1];

    #ifdef PAPER_CORECTION_DEBUG
    cout << "a[i]" << endl;
    for (int i = 0; i < 4; i++) {
        cout << a[i] << " ";
    }
    cout << endl;
    cout << "b[i]" << endl;
    for (int i = 0; i < 4; i++) {
        cout << b[i] << " ";
    }
    cout << endl;
    #endif

    int width = srcImg.width(), height = srcImg.height(), spectrum = srcImg.spectrum();
    Position lt(a[0], b[0]), lb(a[0]+a[2]*(height-1), b[0]+b[2]*(height-1)),
             rt(a[0]+a[1]*(width-1), b[0]+b[1]*(width-1)), 
             rb(a[0]+(a[1]*width-1)+a[2]*(height-1)+a[3]*(width-1)*(height-1),
                b[0]+(b[1]*width-1)+b[2]*(height-1)+b[3]*(width-1)*(height-1));
    int minX = min(lt.x, min(lb.x, min(rt.x, rb.x))),
        maxX = max(lt.x, max(lb.x, max(rt.x, rb.x))),
        minY = min(lt.y, min(lb.y, min(rt.y, rb.y))),
        maxY = max(lt.y, max(lb.y, max(rt.y, rb.y)));

    offsetX = 0 - minX;
    offsetY = 0 - minY;

    int newWidth = maxX+offsetX, newHeight = maxY+offsetY;
    
    #ifdef PAPER_CORECTION_DEBUG
    ofstream fout("retXY.txt");
    fout << "vertex===>" << endl;
    for (int i = 0; i < v.size(); i++) {
        fout << "(" << v[i].x << "," << v[i].y << ")" << endl;
    }
    fout << "standard===>" << endl;
    for (int i = 0; i < s.size(); i++) {
        fout << "(" << s[i].x << "," << s[i].y << ")" << endl;
    }
    fout << "transform====>" << endl;
    for (int i = 0; i < v.size(); i++) {
        fout << "x====>" << a[0]+a[1]*v[i].x+a[2]*v[i].y+a[3]*v[i].x*v[i].y
             << "      y====>" << b[0]+b[1]*v[i].x+b[2]*v[i].y+b[3]*v[i].x*v[i].y << endl;
    }
    fout << "lt===>(" << lt.x << "," << lt.y << ")" << endl
         << "lb===>(" << lb.x << "," << lb.y << ")" << endl
         << "rt===>(" << rt.x << "," << rt.y << ")" << endl
         << "rb===>(" << rb.x << "," << rb.y << ")" << endl;
    fout << "offsetX===>" << offsetX << "   offsetY====>" << offsetY
         << "   newWidth===>" << newWidth << "   newHeight===>" << newHeight << endl;
    #endif

    CImg<float> temp(newWidth+1, newHeight+1, 1, spectrum, 0);

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int retX = (a[0] + a[1]*x + a[2]*y + a[3]*x*y)+offsetX;
            int retY = (b[0] + b[1]*x + b[2]*y + b[3]*x*y)+offsetY;
            if (!(retX >= 0 && retX <= newWidth && retY >= 0 && retY <= newHeight)) {
                continue;
            }
            assert(retX >= 0 && retX <= newWidth && retY >= 0 && retY <= newHeight);
            for (int k = 0; k < spectrum; k++) {
                temp(retX, retY, 0, k) = srcImg(x, y, 0, k);
            }
        }
    }
    return temp;
}

CImg<float> PaperCorection::adjust_orientation(const CImg<float> &srcImg, vector<Position> &v, bool isVertical) {
    if (!isVertical) {
        RotateOP ro;
        double rotateAngle = 90*PI/180;
        int width = srcImg.width(), height = srcImg.height();
        for (int i = 0; i < v.size(); i++) {
            v[i] = ro.rotate_pos(v[i], rotateAngle, width, height);
        }
        
        #ifdef PAPER_CORECTION_DEBUG
        for (int i = 0; i < v.size(); i++) {
            cout << "(" << v[i].x << "," << v[i].y << ")" << endl;
        }
        #endif

        return ro.rotate(srcImg, rotateAngle);
    }
    return srcImg;
}

CImg<float> PaperCorection::clip_img(const CImg<float> &srcImg, vector<Position> &s, 
    int offsetX, int offsetY) {

    #ifdef PAPER_CORECTION_DEBUG
    for (int i = 0; i < s.size(); i++) {
        cout << "(" << s[i].x << "," << s[i].y << ")" << endl; 
    }
    #endif

    Position p = s[0];
    int minIndex = 0;
    for (int i = 1; i < s.size(); i++) {
        if ((s[i].x*s[i].x+s[i].y*s[i].y) < p.x*p.x+p.y*p.y) {
            minIndex = i;
            p = s[i];
        }
    }
    int rightNeighbor, bottomNeighbor;
    int index1 = (minIndex+1) % s.size(), index2 = (minIndex+3) % s.size();
    if (s[index1].x > p.x) {
        rightNeighbor = index1;
        bottomNeighbor = index2;
    } else {
        rightNeighbor = index2;
        bottomNeighbor = index1;
    }
    int thirdNeighbor = (minIndex+2) % s.size();
    int newWidth = s[rightNeighbor].x - s[minIndex].x + 1,
        newHeight = s[bottomNeighbor].y - s[minIndex].y + 1,
        spectrum = srcImg.spectrum();
    CImg<float> ret(newWidth, newHeight, 1, spectrum);
    for (int i = 0, ii = s[minIndex].x+offsetX; i < newWidth && ii <= s[rightNeighbor].x+offsetX; i++, ii++) {
        for (int j = 0, jj = s[minIndex].y+offsetY; j < newHeight && jj <= s[bottomNeighbor].y+offsetY; j++, jj++) {
            for (int k = 0; k < spectrum; k++) {
                ret(i, j, 0, k) = srcImg(ii, jj, 0, k);
            }
        }
    }

    return ret;
}

CImg<float> PaperCorection::image_wrap(vector<Position> &origin, vector<Position> &standard,
    const CImg<float> &srcImg) {
    Position p = standard[0];
    int minIndex = 0;
    for (int i = 1; i < standard.size(); i++) {
        if ((standard[i].x*standard[i].x+standard[i].y*standard[i].y) < p.x*p.x+p.y*p.y) {
            minIndex = i;
            p = standard[i];
        }
    }
    int rt, lb;
    int index1 = (minIndex+1) % standard.size(), index2 = (minIndex+3) % standard.size();
    if (standard[index1].x > p.x) {
        rt = index1;
        lb = index2;
    } else {
        rt = index2;
        lb = index1;
    }

    int newWidth = standard[rt].x - standard[minIndex].x + 1,
        newHeight = standard[lb].y - standard[minIndex].y + 1,
        spectrum = srcImg.spectrum();

    #ifdef PAPER_CORECTION_DEBUG
    cout << "standard===>" << endl;
    for (int i = 0; i < standard.size(); i++) {
        cout << "(" << standard[i].x << "," << standard[i].y << ")" << endl;
    }
    cout << "newWidth===>" << newWidth << "  newHeight===>" << newHeight << endl;
    cout << "origin===>" << endl;
    for (int i = 0; i < origin.size(); i++) {
        cout << "(" << origin[i].x << "," << origin[i].y << ")" << endl;
    }

    #endif

    int rb = (minIndex+2) % standard.size();
    double x[4], y[4], H[8];
    x[0] = origin[minIndex].x, x[1] = origin[rt].x, x[2] = origin[lb].x, x[3] = origin[rb].x;
    y[0] = origin[minIndex].y, y[1] = origin[rt].y, y[2] = origin[lb].y, y[3] = origin[rb].y;
    double w = newWidth-1, h = newHeight-1, 
           a = w*y[3]-w*y[1], b = h*y[3]-h*y[2], c = y[1]+y[2]-y[0]-y[3],
           e = w*x[1]-w*x[3], f = h*x[2]-h*x[3], g = x[0]+x[3]-x[1]-x[2];
    if (e == 0) {
        assert(f != 0);
        assert(a != 0);
        H[7] = g / f;
        H[6] = (c-b*H[7]) / a;
    } else {
        if (a == 0) {
            assert(b != 0);
            H[7] = c / b;
            H[6] = (g-f*H[7]) / e;
        } else if (f == 0) {
            assert(b != 0);
            H[6] = g / e;
            H[7] = (c-a*H[6]) / b;
        } else {
            H[7] = (c*e-a*g)/(b*e-a*f);
            H[6] = (c-b*H[7]) / a;
        }
    }
    H[0] = (-x[0]+w*x[1]*H[6]+x[1]) / w;
    H[1] = (-x[0]+h*x[2]*H[7]+x[2]) / h;
    H[2] = x[0];
    H[3] = (y[0]-w*y[1]*H[6]-y[1])/(-w);
    H[4] = (y[0]-h*y[2]*H[7]-y[2])/(-h);
    H[5] = y[0];

    for (int i = 0; i < 8; i++) {
        cout << "H[" << i << "]==>" << H[i] << endl;
    }

    return biliinear_interpolation(srcImg, newWidth, newHeight, H);
}

CImg<float> PaperCorection::biliinear_interpolation(const CImg<float>& srcImg, int width, 
    int height, double h[8]) {
    int srcWidth = srcImg.width(), srcHeight = srcImg.height();
    int spectrum = srcImg.spectrum();
    double u, v, lamda, srcX, srcY;
    CImg<float> ret(width, height, 1, spectrum);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            lamda = i*h[6] + j*h[7] + 1;
            // srcX = (i*h[0]+j*h[1]+h[2]) * lamda;
            // srcY = (i*h[3]+j*h[4]+h[5]) * lamda;

            srcX = (i*h[0]+j*h[1]+h[2]);
            srcY = (i*h[3]+j*h[4]+h[5]);


            #ifdef PAPER_CORECTION_DEBUG
            if (i == 0 && j == 0) {
                cout << "lamda====>" << lamda << " (" << i << "," << j << ")===>(" << srcX << "," << srcY << ")" << endl;
            } else if (i == width-1 && j == 0) {
                cout << "lamda====>" << lamda << " (" << i << "," << j << ")===>(" << srcX << "," << srcY << ")" << endl;
            } else if (i == 0 && j == height-1) {
                cout << "lamda====>" << lamda << " (" << i << "," << j << ")===>(" << srcX << "," << srcY << ")" << endl;
            } else if (i == width-1 && j == height-1) {
                cout << "lamda====>" << lamda << " (" << i << "," << j << ")===>(" << srcX << "," << srcY << ")" << endl;
            }
            #endif
            u = srcX - (int)srcX;
            v = srcY - (int)srcY;
            if (srcX >= 0 && srcX < srcImg.width() && srcY >= 0 && srcY < srcImg.height()) {
                for (int channel = 0; channel < spectrum; channel++) {
                    ret(i, j, 0, channel) = 
                        (int)((1-u)*(1-v)*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
                        +(1-u)*v*srcImg(valueWidth(srcX, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel)
                        +u*(1-v)*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY, srcHeight), 0, channel)
                        +u*v*srcImg(valueWidth(srcX+1, srcWidth), valueHeight(srcY+1, srcHeight), 0, channel));

                }
            }
        }
    }
    return ret;
}

// 检查像素位置，防止超过图片宽度
int PaperCorection::valueWidth(double srcX, int width) {
    if (srcX < 0) srcX = 0;
    if (srcX >= width) srcX--;
    return srcX;
}

// 检查像素位置，防止超过图片高度
int PaperCorection::valueHeight(double srcY, int height) {
    if (srcY < 0) srcY = 0;
    if (srcY >= height) srcY--;
    return srcY;
}

CImg<float> PaperCorection::paper_corection(const CImg<float> &houghSpace, const CImg<float> &srcImg, 
    const CImg<float> &cannyImg) {
    vector<Position> vertexs = get_vertexs(houghSpace, srcImg, cannyImg);
    bool isVertical;
    vector<Position> standard = get_standard_vertexs(vertexs, isVertical); 
    // int offsetX, offsetY;
    // CImg<float> wrapImg = image_wrap(vertexs, standard, srcImg, offsetX, offsetY);
    // vector<Position> rotateVertex(standard);

    // for (int i = 0; i < rotateVertex.size(); i++) {
    //     rotateVertex[i].x += offsetX;
    //     rotateVertex[i].y += offsetY;
    // }

    // CImg<float> adjustImg = adjust_orientation(wrapImg, rotateVertex, isVertical);
    // CImg<float> ret = clip_img(adjustImg, rotateVertex, offsetX, offsetY);
    CImg<float> ret = image_wrap(vertexs, standard, srcImg);

    #ifdef PAPER_CORECTION_DEBUG

    CImg<float> t1(srcImg);
    for (int i = 0; i < vertexs.size(); i++) {
        cout << "(" << vertexs[i].x << "," << vertexs[i].y << ")\n";
        draw_point(t1, vertexs[i]);
        t1.display();
    }
    t1.save_jpeg("t1.jpg");
    
    CImg<float> t2(srcImg);
    for (int i = 0; i < standard.size(); i++) {
        draw_point(t2, standard[i]);
        t2.display();
    }
    t2.save_jpeg("t2.jpg");

    ret.display("ret");
    ret.save_jpeg("ret.jpg");

    // wrapImg.display("wrapImg");
    // wrapImg.save_jpeg("wrapImg.jpg");
    
    // adjustImg.display("adjustImg");
    // adjustImg.save_jpeg("adjustImg.jpg");

    // for (int i = 0; i < rotateVertex.size(); i++) {
    //     rotateVertex[i].x += offsetX;
    //     rotateVertex[i].y += offsetY;
    // }
    // for (int i = 0; i < rotateVertex.size(); i++) {
    //     draw_point(wrapImg, rotateVertex[i]);
    //     wrapImg.display();
    // }
    // wrapImg.save_jpeg("wrapImg_point.jpg");
    // for (int i = 0; i < rotateVertex.size(); i++) {
    //     draw_point(adjustImg, rotateVertex[i]);
    //     adjustImg.display();
    // }
    // adjustImg.save_jpeg("adjustImg_point.jpg");

    // ret.display("clip");
    // ret.save_jpeg("clip.jpg");
    
    // cout << "standard===>" << endl;
    // for (int i = 0; i < standard.size(); i++) {
    //     cout << "(" << standard[i].x << "," << standard[i].y << ")" << endl;
    // }
    // cout << "rotate===>" << endl;
    // for (int i = 0; i < rotateVertex.size(); i++) {
    //     cout << "(" << rotateVertex[i].x << "," << rotateVertex[i].y << ")" << endl;
    // }


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

