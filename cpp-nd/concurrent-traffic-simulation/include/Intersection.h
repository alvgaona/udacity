#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include <future>
#include <mutex>
#include <memory>
#include "TrafficObject.h"
#include "TrafficLight.h"

// forward declarations to avoid include cycle
class Street;
class Vehicle;

// auxiliary class to queue and dequeue waiting vehicles in a thread-safe manner
class WaitingVehicles
{
public:
    // getters / setters
    int get_size();

    // typical behaviour methods
    void push_back(std::shared_ptr<Vehicle> vehicle,
                   std::promise<void> &&promise);
    void permit_entry_to_first_in_queue();

private:
    std::vector<std::shared_ptr<Vehicle>> vehicles_;          // list of all vehicles waiting to enter this intersection
    std::vector<std::promise<void>> promises_; // list of associated promises
    std::mutex mutex_;
};

class Intersection : public TrafficObject
{
public:
    // constructor / desctructor
    Intersection();

    // getters / setters
    void SetIsBlocked(bool is_blocked);

    // typical behaviour methods
    void AddVehiclesToQueue(const std::shared_ptr<Vehicle> &vehicle);
    void AddStreet(std::shared_ptr<Street> street);
    std::vector<std::shared_ptr<Street>> QueryStreets(const std::shared_ptr<Street> &incoming);
    void Simulate();
    void VehicleHasLeft(const std::shared_ptr<Vehicle> &vehicle);
    bool TrafficLightIsGreen();

private:

    // typical behaviour methods
    void ProcessVehicleQueue();

    // private members
    std::vector<std::shared_ptr<Street>> streets_;   // list of all streets connected to this intersection
    WaitingVehicles waiting_vehicles_;               // list of all vehicles and their associated promises waiting to enter the intersection
    bool is_blocked_;                                // flag indicating whether the intersection is blocked by a vehicle
    TrafficLight traffic_light_;
};

#endif
