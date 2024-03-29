#include <iostream>
#include "../include/Vehicle.h"
#include "../include/Intersection.h"
#include "../include/Street.h"


Street::Street()
{
    type_ = ObjectType::kObjectStreet;
    length_ = 1000.0; // in m
}

void Street::SetInIntersection(std::shared_ptr<Intersection> in)
{
    inter_in_ = in;
    in->AddStreet(GetSharedThis()); // add this street to list of streets connected to the intersection
}

void Street::SetOutIntersection(std::shared_ptr<Intersection> out)
{
    inter_out_ = out;
    out->AddStreet(GetSharedThis()); // add this street to list of streets connected to the intersection
}
