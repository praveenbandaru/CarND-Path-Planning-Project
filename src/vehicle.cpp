#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->state = state;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.   
   */
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<vector<Vehicle>> final_trajectories;
  string best_state;

  double best_cost = 99999999;
  vector<Vehicle> best_trajectory;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    //Debug
    int i = 0;
    for (auto j: trajectory)
    {      
      std::cout << "traj lane "<< i << ": " << j.lane << std::endl;
      std::cout << "traj v "<< i++ << ": " << j.v << std::endl;
    }   

    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);

      if(cost<best_cost){
        best_cost = cost;
        best_trajectory = trajectory;
        best_state = *it;
      }

      // Debug
      std::cout << "Trajectory: " << *it << std::endl;
      std::cout << "Cost: " << cost << std::endl;
    }
  }

  std::cout << "Best Cost: " << best_cost << std::endl;
  std::cout << "Best Trajectory: " << best_state << std::endl;
              

  /* vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  std::cout << "Best Cost: " << *best_cost << std::endl;
  int best_idx = distance(begin(costs), best_cost);
  std::cout << "Trajectory Best Index: " << best_idx << std::endl; */

  /**
   * TODO: Change return value here:
   */
  //return final_trajectories[best_idx];
  return best_trajectory;
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    if (this->lane != 0) {
      states.push_back("PLCL");
    }
    if (this->lane != lanes_available - 1) {
      states.push_back("PLCR");
    }
  } else if (state.compare("PLCL") == 0) {
    if (this->lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (this->lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, 
                                             map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, 
                                      int lane) {
  // Gets next timestep kinematics (position, velocity) 
  //   for a given lane. Tries to choose the maximum velocity, 
  //   given other vehicle positions and accel/velocity constraints.
  double max_velocity_accel_limit;
  double new_position;
  double new_velocity;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      //new_velocity = vehicle_ahead.v;
      max_velocity_accel_limit =  this->v - this->max_acceleration/2.24;
      double max_velocity_in_front = (vehicle_ahead.s - this->s 
                                  - this->preferred_buffer) + vehicle_ahead.v;
      new_velocity = std::min(std::min(max_velocity_in_front, 
                                       max_velocity_accel_limit), 
                                       this->target_speed/2.24);
    } else {
      max_velocity_accel_limit =  this->v - this->max_acceleration/2.24;
      double max_velocity_in_front = (vehicle_ahead.s - this->s 
                                  - this->preferred_buffer) + vehicle_ahead.v;
      new_velocity = std::min(std::min(max_velocity_in_front, 
                                       max_velocity_accel_limit), 
                                       this->target_speed/2.24);
    }
  } else {
    max_velocity_accel_limit = this->max_acceleration/2.24 + this->v;
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed/2.24);
  }
    
  //new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity;
    
  return{new_position, new_velocity};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  double next_pos = position_at(1.0);
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->state), 
                                Vehicle(this->lane,next_pos,this->v,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->state)};
  vector<double> kinematics = get_kinematics(predictions, this->lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, "KL"));
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, 
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  double new_s;
  double new_v;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, 
                                        this->state)};
  vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];  
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    // Choose kinematics with highest velocity.
    if (next_lane_new_kinematics[1] > curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, 
                                                map<int, vector<Vehicle>> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies 
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, 
                               this->state));
  vector<double> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], state));
  return trajectory;
}

double Vehicle::position_at(double t) {
  return this->s + this->v*t;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, 
                                 int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = 30;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s < this->s 
        && (this->s - temp_vehicle.s) < max_s) {
      max_s = this->s -temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  //std::cout << "found vehicle behind " << found_vehicle << std::endl;
  /* if(found_vehicle)
  {
    std::cout << "found vehicle behind position " << rVehicle.s << std::endl;
    std::cout << "found vehicle behind speed " << rVehicle.v << std::endl;
  } */
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, 
                                int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int min_s = 30;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s > this->s 
        && (temp_vehicle.s - this->s) < min_s) {
      min_s = temp_vehicle.s - this->s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  //std::cout << "found vehicle ahead " << found_vehicle << std::endl;
  /* if(found_vehicle)
  {
    std::cout << "found vehicle ahead position " << rVehicle.s << std::endl;
    std::cout << "found vehicle ahead speed " << rVehicle.v << std::endl;
  } */
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(double t) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  double next_s = position_at(t);
  double next_v = this->v;
  predictions.push_back(Vehicle(this->lane, next_s, next_v));
  
  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
}
