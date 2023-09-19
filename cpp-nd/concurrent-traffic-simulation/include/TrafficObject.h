#ifndef TRAFFIC_OBJECT_H
#define TRAFFIC_OBJECT_H

#include <mutex>
#include <thread>
#include <vector>

enum class ObjectType {
  kNoObject = 0,
  kObjectVehicle,
  kObjectIntersection,
  kObjectStreet,
};

class TrafficObject {
public:
  // constructor / destructor
  TrafficObject();
  ~TrafficObject();

  // getter and setter
  int GetId() { return id_; }
  void SetPosition(double x, double y);
  void GetPosition(double &x, double &y);
  ObjectType GetType() { return type_; }

  // typical behaviour methods
  virtual void Simulate() {};

protected:
  ObjectType type_;                    // identifies the class type
  int id_;                             // every traffic object has its own unique id
  double pos_x_;                       // x vehicle position in pixels
  double post_y_;                      // y vehicle position in pixels
  std::vector<std::thread> threads_;   // holds all threads_ that have been launched within this object
  static std::mutex mutex_;            // mutex shared by all traffic objects for protecting cout

private:
  static int id_count_;                // global variable for counting object ids
};

#endif
