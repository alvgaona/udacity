#include "../include/Graphics.h"
#include "../include/Intersection.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void Graphics::Simulate()
{
    this->LoadBackgroundImage();
    while (true)
    {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // update graphics
        this->DrawTrafficObjects();
    }
}

void Graphics::LoadBackgroundImage()
{
    // create window
    window_name_ = "Concurrency Traffic Simulation";
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    // load image and create copy to be used for semi-transparent overlay
    cv::Mat background = cv::imread(bg_filename_);
    images_.push_back(background);         // first element is the original background
    images_.push_back(background.clone()); // second element will be the transparent overlay
    images_.push_back(background.clone()); // third element will be the result image for display
}

void Graphics::DrawTrafficObjects()
{
    // reset images
    images_.at(1) = images_.at(0).clone();
    images_.at(2) = images_.at(0).clone();

    // create overlay from all traffic objects
    for (auto it : traffic_objects_)
    {
        double posx, posy;
        it->GetPosition(posx, posy);

        if (it->GetType() == ObjectType::kObjectIntersection)
        {
            // cast object type from TrafficObject to Intersection
            std::shared_ptr<Intersection> intersection = std::dynamic_pointer_cast<Intersection>(it);

            // set color according to traffic light and draw the intersection as a circle
            cv::Scalar trafficLightColor = intersection->TrafficLightIsGreen() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::circle(images_.at(1), cv::Point2d(posx, posy), 25, trafficLightColor, -1);
        }
        else if (it->GetType() == ObjectType::kObjectVehicle)
        {
            cv::RNG rng(it->GetId());
            int b = rng.uniform(0, 255);
            int g = rng.uniform(0, 255);
            int r = sqrt(255*255 - g*g - r*r); // ensure that length of color vector is always 255
            cv::Scalar vehicleColor = cv::Scalar(b,g,r);
            cv::circle(images_.at(1), cv::Point2d(posx, posy), 50, vehicleColor, -1);
        }
    }

    float opacity = 0.85;
    cv::addWeighted(images_.at(1), opacity, images_.at(0), 1.0 - opacity, 0, images_.at(2));

    // display background and overlay image
    cv::imshow(window_name_, images_.at(2));
    cv::waitKey(33);
}
