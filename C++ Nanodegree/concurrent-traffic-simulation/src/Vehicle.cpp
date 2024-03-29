#include "../include/Vehicle.h"
#include "../include/Intersection.h"
#include "../include/Street.h"
#include <iostream>
#include <random>

Vehicle::Vehicle()
{
    current_street_ = nullptr;
    position_street_ = 0.0;
    type_ = ObjectType::kObjectVehicle;
    speed_ = 400; // m/s
}


void Vehicle::SetCurrentDestination(std::shared_ptr<Intersection> destination)
{
    // update destination
    current_destination_ = destination;

    // reset simulation parameters
    position_street_ = 0.0;
}

void Vehicle::Simulate()
{
    // launch drive function in a thread
    threads_.emplace_back(std::thread(&Vehicle::drive, this));
}

// virtual function which is executed in a thread
void Vehicle::drive()
{
    // print id of the current thread
    std::unique_lock<std::mutex> lck(mutex_);
    std::cout << "Vehicle #" << id_ << "::drive: thread id = " << std::this_thread::get_id() << std::endl;
    lck.unlock();

    // initalize variables
    bool hasEnteredIntersection = false;
    double cycleDuration = 1; // duration of a single simulation cycle in ms
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    // init stop watch
    lastUpdate = std::chrono::system_clock::now();
    while (true)
    {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
            // update position with a constant velocity motion model
            position_street_ += speed_ * timeSinceLastUpdate / 1000;

            // compute completion rate of current street
            double completion = position_street_ / current_street_->GetLength();

            // compute current pixel position on street based on driving direction
            std::shared_ptr<Intersection> i1, i2;
            i2 = current_destination_;
            i1 = i2->GetId() == current_street_->GetInIntersection()->GetId() ? current_street_->GetOutIntersection() : current_street_->GetInIntersection();

            double x1, y1, x2, y2, xv, yv, dx, dy, l;
            i1->GetPosition(x1, y1);
            i2->GetPosition(x2, y2);
            dx = x2 - x1;
            dy = y2 - y1;
            l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (x1 - x2));
            xv = x1 + completion * dx; // new position based on line equation in parameter form
            yv = y1 + completion * dy;
            this->SetPosition(xv, yv);

            // check wether halting position in front of destination has been reached
            if (completion >= 0.9 && !hasEnteredIntersection)
            {
                // request entry to the current intersection (using async)
                auto ftrEntryGranted = std::async(&Intersection::AddVehiclesToQueue, current_destination_, GetSharedThis());

                // wait until entry has been granted
                ftrEntryGranted.get();

                // slow down and set intersection flag
                speed_ /= 10.0;
                hasEnteredIntersection = true;
            }

            // check whether intersection has been crossed
            if (completion >= 1.0 && hasEnteredIntersection)
            {
                // choose next street and destination
                std::vector<std::shared_ptr<Street>> streetOptions = current_destination_->QueryStreets(current_street_);
                std::shared_ptr<Street> nextStreet;
                if (streetOptions.size() > 0)
                {
                    // pick one street at random and query intersection to enter this street
                    std::random_device rd;
                    std::mt19937 eng(rd());
                    std::uniform_int_distribution<> distr(0, streetOptions.size() - 1);
                    nextStreet = streetOptions.at(distr(eng));
                }
                else
                {
                    // this street is a dead-end, so drive back the same way
                    nextStreet = current_street_;
                }
                
                // pick the one intersection at which the vehicle is currently not
                std::shared_ptr<Intersection> nextIntersection = nextStreet->GetInIntersection()->GetId() == current_destination_->GetId() ? nextStreet->GetOutIntersection() : nextStreet->GetInIntersection();

                // send signal to intersection that vehicle has left the intersection
                current_destination_->VehicleHasLeft(GetSharedThis());

                // assign new street and destination
                this->SetCurrentDestination(nextIntersection);
                this->SetCurrentStreet(nextStreet);

                // reset speed and intersection flag
                speed_ *= 10.0;
                hasEnteredIntersection = false;
            }

            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();
        }
    } // eof simulation loop
}
