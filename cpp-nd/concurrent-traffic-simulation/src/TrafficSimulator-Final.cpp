#include <iostream>
#include <thread>
#include <vector>
#include <memory>

#include "../include/Graphics.h"
#include "../include/Intersection.h"
#include "../include/Street.h"
#include "../include/Vehicle.h"

// Paris
void createTrafficObjects_Paris(
    std::vector<std::shared_ptr<Street>> &streets,
    std::vector<std::shared_ptr<Intersection>> &intersections,
    std::vector<std::shared_ptr<Vehicle>> &vehicles,
    std::string &filename,
    int nVehicles)
{
    // assign filename of corresponding city map
    filename = "../data/paris.jpg";

    // init traffic objects
    int nIntersections = 9;
    for (auto ni = 0; ni < nIntersections; ni++)
    {
        intersections.push_back(std::make_shared<Intersection>());
    }

    // position intersections in pixel coordinates (counter-clockwise)
    intersections.at(0)->SetPosition(385, 270);
    intersections.at(1)->SetPosition(1240, 80);
    intersections.at(2)->SetPosition(1625, 75);
    intersections.at(3)->SetPosition(2110, 75);
    intersections.at(4)->SetPosition(2840, 175);
    intersections.at(5)->SetPosition(3070, 680);
    intersections.at(6)->SetPosition(2800, 1400);
    intersections.at(7)->SetPosition(400, 1100);
    intersections.at(8)->SetPosition(1700, 900); // central plaza

    // create streets and connect traffic objects
    int nStreets = 8;
    for (auto ns = 0; ns < nStreets; ns++)
    {
        streets.push_back(std::make_shared<Street>());
        streets.at(ns)->SetInIntersection(intersections.at(ns));
        streets.at(ns)->SetOutIntersection(intersections.at(8));
    }

    // add vehicles to streets
    for (auto nv = 0; nv < nVehicles; nv++)
    {
        vehicles.push_back(std::make_shared<Vehicle>());
        vehicles.at(nv)->SetCurrentStreet(streets.at(nv));
        vehicles.at(nv)->SetCurrentDestination(intersections.at(8));
    }
}

// NYC
void createTrafficObjects_NYC(std::vector<std::shared_ptr<Street>> &streets, std::vector<std::shared_ptr<Intersection>> &intersections, std::vector<std::shared_ptr<Vehicle>> &vehicles, std::string &filename, int nVehicles)
{
    // assign filename of corresponding city map
    filename = "../data/nyc.jpg";

    // init traffic objects
    int nIntersections = 6;
    for (auto ni = 0; ni < nIntersections; ni++)
    {
        intersections.push_back(std::make_shared<Intersection>());
    }

    // position intersections in pixel coordinates
    intersections.at(0)->SetPosition(1430, 625);
    intersections.at(1)->SetPosition(2575, 1260);
    intersections.at(2)->SetPosition(2200, 1950);
    intersections.at(3)->SetPosition(1000, 1350);
    intersections.at(4)->SetPosition(400, 1000);
    intersections.at(5)->SetPosition(750, 250);

    // create streets and connect traffic objects
    int nStreets = 7;
    for (auto ns = 0; ns < nStreets; ns++)
    {
        streets.push_back(std::make_shared<Street>());
    }

    streets.at(0)->SetInIntersection(intersections.at(0));
    streets.at(0)->SetOutIntersection(intersections.at(1));

    streets.at(1)->SetInIntersection(intersections.at(1));
    streets.at(1)->SetOutIntersection(intersections.at(2));

    streets.at(2)->SetInIntersection(intersections.at(2));
    streets.at(2)->SetOutIntersection(intersections.at(3));

    streets.at(3)->SetInIntersection(intersections.at(3));
    streets.at(3)->SetOutIntersection(intersections.at(4));

    streets.at(4)->SetInIntersection(intersections.at(4));
    streets.at(4)->SetOutIntersection(intersections.at(5));

    streets.at(5)->SetInIntersection(intersections.at(5));
    streets.at(5)->SetOutIntersection(intersections.at(0));

    streets.at(6)->SetInIntersection(intersections.at(0));
    streets.at(6)->SetOutIntersection(intersections.at(3));

    // add vehicles to streets
    for (auto nv = 0; nv < nVehicles; nv++)
    {
        vehicles.push_back(std::make_shared<Vehicle>());
        vehicles.at(nv)->SetCurrentStreet(streets.at(nv));
        vehicles.at(nv)->SetCurrentDestination(intersections.at(nv));
    }
}

int main()
{
    /* PART 1 : Set up traffic objects */

    // create and connect intersections and streets
    std::vector<std::shared_ptr<Street>> streets;
    std::vector<std::shared_ptr<Intersection>> intersections;
    std::vector<std::shared_ptr<Vehicle>> vehicles;
    std::string backgroundImg;
    int nVehicles = 6;
    createTrafficObjects_Paris(streets, intersections, vehicles, backgroundImg, nVehicles);

    /* PART 2 : Simulate traffic objects */

    // Simulate intersection
    std::for_each(intersections.begin(), intersections.end(), [](std::shared_ptr<Intersection> &i) {
        i->Simulate();
    });

    // Simulate vehicles
    std::for_each(vehicles.begin(), vehicles.end(), [](std::shared_ptr<Vehicle> &v) {
        v->Simulate();
    });

    /* PART 3 : Launch visualization */

    // add all objects into common vector
    std::vector<std::shared_ptr<TrafficObject>> trafficObjects;
    std::for_each(intersections.begin(), intersections.end(), [&trafficObjects](std::shared_ptr<Intersection> &intersection) {
        std::shared_ptr<TrafficObject> trafficObject = std::dynamic_pointer_cast<TrafficObject>(intersection);
        trafficObjects.push_back(trafficObject);
    });

    std::for_each(vehicles.begin(), vehicles.end(), [&trafficObjects](std::shared_ptr<Vehicle> &vehicles) {
        std::shared_ptr<TrafficObject> trafficObject = std::dynamic_pointer_cast<TrafficObject>(vehicles);
        trafficObjects.push_back(trafficObject);
    });

    // draw all objects in vector
    auto graphics = new Graphics();
    graphics->SetBgFileName(backgroundImg);
    graphics->SetTrafficObjects(trafficObjects);
    graphics->Simulate();
}
