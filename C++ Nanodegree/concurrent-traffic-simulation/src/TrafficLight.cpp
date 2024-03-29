#include "../include/TrafficLight.h"
#include <future>
#include <iostream>
#include <random>

/* Implementation of class "MessageQueue" */

template <typename T> T MessageQueue<T>::receive() {
  std::unique_lock<std::mutex> u_lck(mutex_);
  condition_.wait(u_lck, [this] { return !queue_.empty(); });

  T message = std::move(queue_.back());
  queue_.pop_back();

  return message;
}

template <typename T> void MessageQueue<T>::send(T &&message) {
  std::lock_guard<std::mutex> lck_guard(mutex_);
  queue_.emplace_back(std::move(message));
  condition_.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight() {
  current_phase_ = TrafficLightPhase::kRed;
  msg_queue_ = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

TrafficLight::~TrafficLight() {}

void TrafficLight::WaitForGreen() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    auto curr_phase = msg_queue_->receive();
    if (curr_phase == TrafficLightPhase::kGreen) {
      return;
    }
  }
}

void TrafficLight::SetCurrentPhase(const TrafficLightPhase phase) {
  current_phase_ = phase;
}

TrafficLightPhase TrafficLight::GetCurrentPhase() { return current_phase_; }

void TrafficLight::Simulate() {
  threads_.emplace_back(std::thread(&TrafficLight::CycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::CycleThroughPhases() {
  std::random_device device;
  std::mt19937 mt_engine(device());
  std::uniform_int_distribution<> distribution(4, 6);

  std::unique_lock<std::mutex> lck(mutex_);
  std::cout << "Traffic Light #" << id_
            << "::Cycle Through Phases: thread id = "
            << std::this_thread::get_id() << std::endl;
  lck.unlock();

  int cycle_duration = distribution(mt_engine);

  auto last_update = std::chrono::system_clock::now();
  while (true) {
    long time_since_last_update =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - last_update)
            .count();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (time_since_last_update >= cycle_duration) {
      if (current_phase_ == TrafficLightPhase::kRed) {
        current_phase_ = TrafficLightPhase::kGreen;
      } else {
        current_phase_ = TrafficLightPhase::kRed;
      }

      auto msg = current_phase_;
      auto is_sent =
          std::async(
              std::launch::async, &MessageQueue<TrafficLightPhase>::send,
              msg_queue_,
              msg);
      is_sent.wait();

      last_update = std::chrono::system_clock::now();
      cycle_duration = distribution(mt_engine);
    }
  }
}
