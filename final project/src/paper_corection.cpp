#include "paper_corection.h"
#include "global.h"
#include "homography.h"
#include "image_segmentation.h"
#include "canny.h"
#include "hough.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>
#include <cassert>
#include <fstream>
#include <cmath>

using namespace std;

PaperCorection::PaperCorection() {
    ifstream fin("config.txt");
    string type;
    double number;
    while (fin >> type >> number) {
        if (type == "PAPER_CORRECT_RATE") {
            this->rate = number;
        } else if (type == "PAPER_CORRECT_ERROR_THETA") {
            this->errorTheta = number;
        } else if (type == "PAPER_CORRECT_ERROR_P") {
            this->errorP = number;
        } else if (type == "PAPER_CORRECT_VERTICAL_ERROR") {
            this->thetaError = number;
        } else if (type == "PAPER_CORRECT_DISTANCE_ERROR") {
            this->disError = number;
        } else {
            continue;
        }
    }
    fin.close();
    // params for canny
    this->sigma = 1.5;
    this->winSize = 1;  
    this->firstDerWinSize = 1;
}

PaperCorection::PaperCorection(double r, int t, int p) {
    this->rate = r;
    this->errorTheta = t;
    this->errorP = p;
    // params for canny
    this->sigma = 1.5;
    this->winSize = 1;  
    this->firstDerWinSize = 1;
    // 垂直直线检测的误差
    this->thetaError = 5;
    this->disError = 100;
}

PaperCorection::PaperCorection(const PaperCorection& p) {
    this->rate = p.rate;
    this->errorTheta = p.errorTheta;
    this->errorP = p.errorP;
    this->thetaError = p.thetaError;
    this->disError = p.disError;
    this->sigma = p.sigma;
    this->winSize = p.winSize;
    this->firstDerWinSize = p.firstDerWinSize;
}

vector<Position> PaperCorection::test_detect_edge(vector<Position> &pos, const CImg<float> &srcImg) {
    assert(pos.size() >= 4);
    vector<Position> res;
    // 默认投票数最高的点是正确的边缘
    Position p1 = pos[0], p2;
    res.push_back(p1);
    // p2的下标
    int p2Index = 0;
    // 找到与p1垂直的直线p2
    for (int i = 1; i < pos.size(); i++) {
        if (abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-360) <= thetaError) {
            continue;
        }
        if ((abs(p1.x-pos[i].x) >= 90-thetaError && abs(p1.x-pos[i].x) <= 90+thetaError) ||
            (abs(abs(p1.x-pos[i].x)-180) >= 90-thetaError && abs(abs(p1.x-pos[i].x)-180) <= 90+thetaError)) {
            p2 = pos[i];
            p2Index = i;
            break;
        }
    }
    for (int i = 1; i < pos.size(); i++) {
        if ((abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-180) <= thetaError) && 
            (abs(p1.y-pos[i].y) > disError)) {
            res.push_back(pos[i]);
            break;
        }
    }
    res.push_back(p2);
    for (int i = 1; i < pos.size(); i++) {
        if (i != p2Index && (abs(p2.x-pos[i].x) <= thetaError || abs(abs(p2.x-pos[i].x)-180) <= thetaError) &&
            (abs(p2.y-pos[i].y) > disError)) {
            res.push_back(pos[i]);
            break;
        }
    }

    #ifdef PAPER_CORECTION_DEBUG
    ofstream fout("set.txt", ofstream::app);
    fout << endl;
    CImg<float> temp(srcImg);
    for (int i = 0; i < res.size(); i++) {
        cout << res[i].x << " " << res[i].y << " " << res[i].sum << endl;
        fout << res[i].x << " " << res[i].y << " " << res[i].sum << endl;
        draw_result(temp, res[i].x, res[i].y, 0);
        temp.display();
    }
    temp.save("standard_edge.png");
    fout.close();
    #endif

    return res;
}


vector<Position> PaperCorection::detect_edge(const CImg<float> &houghSpace, const CImg<float> &srcImg) {
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


    multimap<int, Position>::reverse_iterator reverseIt = cluster.rbegin();
    vector<Position> pos;
    for (reverseIt = cluster.rbegin(); reverseIt != cluster.rend(); reverseIt++) {
        pos.push_back(reverseIt->second);
    }

    #ifdef PAPER_CORECTION_DEBUG
    // cout << "pos.size()=====>" << pos.size() << endl;
    // ofstream fout("set.txt");
    // CImg<float> temp(srcImg);
    // for (int i = 0; i < pos.size(); i++) {
    //     cout << pos[i].x << " " << pos[i].y << " " << pos[i].sum << endl;
    //     fout << pos[i].x << " " << pos[i].y << " " << pos[i].sum << endl;
    //     draw_result(temp, pos[i].x, pos[i].y, 0);
    //     //temp.display();
    // }
    // temp.display("edge");
    // temp.save("edge.png");
    // fout.close();
    #endif

    //return test_detect_edge(pos, srcImg);


    return detect_edge(pos);
}

vector<Position> PaperCorection::detect_edge(vector<Position> &pos) {
    assert(pos.size() >= 4);

    vector<Position> res;
    // 默认投票数最高的点是正确的边缘
    Position p1 = pos[0], p2;
    res.push_back(p1);
    // p2的下标
    int p2Index = 0;
    // 找到与p1垂直的直线p2
    for (int i = 1; i < pos.size(); i++) {
        if ((abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-360) <= thetaError)) {
            continue;
        }
        if ((abs(p1.x-pos[i].x) >= 90-thetaError && abs(p1.x-pos[i].x) <= 90+thetaError) ||
            (abs(abs(p1.x-pos[i].x)-180) >= 90-thetaError && abs(abs(p1.x-pos[i].x)-180) <= 90+thetaError)) {
            p2 = pos[i];
            p2Index = i;
            break;
        }
    }
    for (int i = 1; i < pos.size(); i++) {
        if ((abs(p1.x-pos[i].x) <= thetaError || abs(abs(p1.x-pos[i].x)-180) <= thetaError)) {
            res.push_back(pos[i]);
            break;
        }
    }
    res.push_back(p2);
    for (int i = 1; i < pos.size(); i++) {
        if (i != p2Index && (abs(p2.x-pos[i].x) <= thetaError || abs(abs(p2.x-pos[i].x)-180) <= thetaError) &&
            (abs(p2.y-pos[i].y) > disError)) {
            res.push_back(pos[i]);
            break;
        }
    }

    return res;
}

vector<Position> PaperCorection::get_vertexs(vector<Position> &pos) {
    //vector<Position> pos = detect_edge(houghSpace, srcImg, cannyImg);
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


vector<Position> PaperCorection::get_standard_vertexs(vector<Position> &v, int srcWidth, int srcHeight) {
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

    for (int i = 0; i < ret.size(); i++) {
        if (ret[i].x < 0) {
            ret[i].x = 0;
        }
        if (ret[i].x >= srcWidth) {
            ret[i].x = srcWidth-1;
        }
        if (ret[i].y < 0) {
            ret[i].y = 0;
        }
        if (ret[i].y >= srcHeight) {
            ret[i].y = srcHeight-1;
        }
    }

    return ret;
}

CImg<float> PaperCorection::clip_img(const CImg<float> &srcImg, vector<Position> &s) {

    Position p = s[0];
    int lt = 0;
    for (int i = 1; i < s.size(); i++) {
        if ((s[i].x*s[i].x+s[i].y*s[i].y) < p.x*p.x+p.y*p.y) {
            lt = i;
            p = s[i];
        }
    }
    int rt, lb;
    int index1 = (lt+1) % s.size(), index2 = (lt+3) % s.size();
    if (s[index1].x > p.x) {
        rt = index1;
        lb = index2;
    } else {
        rt = index2;
        lb = index1;
    }
    int rb = (lt+2) % s.size();
    int newWidth = s[rt].x - s[lt].x + 1,
        newHeight = s[lb].y - s[lt].y + 1,
        spectrum = srcImg.spectrum();
    CImg<float> ret(newWidth, newHeight, 1, spectrum);
    for (int i = 0; i < newWidth; i++) {
        for (int j = 0; j < newHeight; j++) {
            for (int channel = 0; channel < spectrum; channel++) {
                ret(i, j, 0, channel) = srcImg(i+s[lt].x, j+s[lt].y, 0, channel);
            }
        }
    }

    return ret;
}

CImg<float> PaperCorection::biliinear_interpolation(const CImg<float>& srcImg, int width, 
    int height, double h[9]) {
    int srcWidth = srcImg.width(), srcHeight = srcImg.height();
    int spectrum = srcImg.spectrum();
    double u, v, lamda, srcX, srcY;
    CImg<float> ret(width, height, spectrum, spectrum);
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {

            srcX = Homography::calc_X(i, j, h);
            srcY = Homography::calc_Y(i, j, h);

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


CImg<float> PaperCorection::image_wrap(vector<Position> &vertexs, vector<Position> &s, const CImg<float> &srcImg) {
    assert(vertexs.size() == s.size());

    vector<Point2f> src, dest;
    for (int i = 0; i < vertexs.size(); i++) {
        src.push_back(Point2f(vertexs[i].x, vertexs[i].y));
        dest.push_back(Point2f(s[i].x, s[i].y));
    }
    double forwardH[9];
    double backwardH[9];
    Homography::calc_homography(src, dest, forwardH);
    Homography::calc_homography(dest, src, backwardH);

    int srcWidth = srcImg.width(), srcHeight = srcImg.height(), spectrum = srcImg.spectrum();
    return biliinear_interpolation(srcImg, srcWidth, srcHeight, backwardH);
}

CImg<float> PaperCorection::paper_corection(const CImg<float> &srcImg) {

    CImg<float> grayImg = get_gray_image(srcImg);

    ImageSeg<float> imageSeg;
    CImg<float> mask(5, 5, 1, 1, 255);

    CImg<float> segImg = imageSeg.segment_image(grayImg);
    segImg = segImg.get_erode(mask);

    xyz::Canny canny(sigma, winSize, firstDerWinSize);
    CImg<float> cannyImg = canny.detect_edge(segImg);

    int width = cannyImg.width();
    int height = cannyImg.height();
    int diagonal = sqrt(width*width + height*height);

    HoughTransform hough(360, diagonal);
    hough.draw_hough_space(cannyImg);

    vector<Position> pos = detect_edge(hough.get_hough_space(), srcImg);
    vector<Position> vertexs = get_vertexs(pos);
    vector<Position> standard = get_standard_vertexs(vertexs, srcImg.width(), srcImg.height()); 

    // 在原图上二值化之后再腐蚀
    CImg<float> erodeImg;
    mask.assign(3, 3, 1, 1, 255);
    grayImg = imageSeg.segment_image(grayImg);
    erodeImg = grayImg.get_erode(mask);
    CImg<float> wrapImg = image_wrap(vertexs, standard, erodeImg);
    CImg<float> clipImg = clip_img(wrapImg, standard);
    // 重要，之后需要根据二值化的图片统计像素数量
    clipImg = imageSeg.segment_image(clipImg);

    return clipImg;
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
            if (p >= temp_p-1 && p <= temp_p+1) {
                img(i, j, 0, channel) = 0xff;
            }
        }
    }
} 

CImg<float> PaperCorection::get_gray_image(const CImg<float> &srcImg) {
    if (srcImg.spectrum() == 1) {
        return srcImg;
    }
    int width = srcImg.width();
    int height = srcImg.height();
    int depth = srcImg.depth();
    CImg<float> grayImg(width, height, depth, 1);
    float r, g, b, gr;
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            r = srcImg(i, j, 0, 0);
            g = srcImg(i, j, 0, 1);
            b = srcImg(i, j, 0, 2);
            gr = 0.299*(r) + 0.587*(g) + 0.114*(b);
            grayImg(i, j, 0, 0) = gr;
        }
    }  
    return grayImg;
}

void PaperCorection::draw_point(CImg<float> &img, const Position &p) {
    int width = img.width();
    int height = img.height();
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (sqrt((i-p.x)*(i-p.x)+(j-p.y)*(j-p.y)) < 10) {
                img(i, j, 0, 0) = 0xff;
            }
        }
    }
}

