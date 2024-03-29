#ifndef VEHICLE_H
#define VEHICLE_H

#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Street;
class Intersection;

class Vehicle : public TrafficObject, public std::enable_shared_from_this<Vehicle>
{
public:
    // constructor / desctructor
    Vehicle();

    // getters / setters
    void SetCurrentStreet(std::shared_ptr<Street> street) { current_street_ = street; };
    void SetCurrentDestination(std::shared_ptr<Intersection> destination);

    // typical behaviour methods
    void Simulate() override;

    // miscellaneous
    std::shared_ptr<Vehicle> GetSharedThis() { return shared_from_this(); }

private:
    // typical behaviour methods
    void drive();

    std::shared_ptr<Street> current_street_;            // street on which the vehicle is currently on
    std::shared_ptr<Intersection> current_destination_; // destination to which the vehicle is currently driving
    double position_street_;                            // position on current street
    double speed_;                                      // ego speed in m/s
};

#endif
