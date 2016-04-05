#ifndef _GLOBAL_H_
#define _GLOBAL_H_

const double PI = 3.1415926;

struct Position {
    int x, y, sum;
    Position() {}
    Position(int x, int y, int sum) {
        this->x = x;
        this->y = y;
        this->sum = sum;
    }
};

bool cmp(const Position& a, const Position& b);

#endif