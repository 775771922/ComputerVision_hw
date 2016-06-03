#ifndef _GLOBAL_H_
#define _GLOBAL_H_

const double PI = 3.1415926;

struct Position {
    int x, y, sum;
    Position() {}
    Position(int x, int y) {
    	this->x = x;
    	this->y = y;
    	this->sum = 0;
    }
    Position(int x, int y, int sum) {
        this->x = x;
        this->y = y;
        this->sum = sum;
    }
    Position operator=(const Position &p) {
        this->x = p.x;
        this->y = p.y;
        this->sum = p.sum;
        return *this;
    }
};

bool cmp(const Position& a, const Position& b);

#endif