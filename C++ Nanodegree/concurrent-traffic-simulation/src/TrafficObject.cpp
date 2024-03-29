#include "../include/TrafficObject.h"
#include <algorithm>
#include <chrono>
#include <iostream>

// init static variable
int TrafficObject::id_count_ = 0;

std::mutex TrafficObject::mutex_;

void TrafficObject::SetPosition(double x, double y)
{
    pos_x_ = x;
    post_y_ = y;
}

void TrafficObject::GetPosition(double &x, double &y)
{
    x = pos_x_;
    y = post_y_;
}

TrafficObject::TrafficObject()
{
    type_ = ObjectType::kNoObject;
    id_ = id_count_++;
}

TrafficObject::~TrafficObject()
{
    // set up thread barrier before this object is destroyed
    std::for_each(threads_.begin(), threads_.end(), [](std::thread &t) {
        t.join();
    });
}
