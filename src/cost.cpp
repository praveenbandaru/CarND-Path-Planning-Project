#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
double REACH_GOAL = 0*pow(10, 1);
double INEFFICIENCY = pow(10, 2);
double COLLISION = pow(10, 1);
double BUFFER = pow(10, 3);
double COMFORT = pow(10,0);

// Here we have provided two possible suggestions for cost functions, but feel 
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

double goal_distance_cost(Vehicle &vehicle, 
                         vector<Vehicle> &trajectory, 
                         map<int, vector<Vehicle>> &predictions, 
                         map<string, double> &data) {
  // Cost increases based on distance of intended lane (for planning a lane 
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches 
  //   goal distance.
  double cost;
  double distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] 
         - data["final_lane"]) / distance));
  } else {
    cost = 1;
  }

  return cost;
}

double inefficiency_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  double proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed/2.24;
  }

  double proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed/2.24;
  }
    
  double cost = (2.0*vehicle.target_speed/2.24 - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed/2.24;

  return cost;
}

/* double efficiency_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data) {
  // Rewards high average speeds. 

  double proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed/2.24;
  }

  // Debug
  std::cout << "proposed_speed_intended: " << proposed_speed_intended << std::endl;

  double cost = 2.0 / (1 + exp(-(2*double(vehicle.target_speed/2.24 - proposed_speed_intended) / proposed_speed_intended))) - 1.0;

  return cost;
} */

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> &predictions, int lane) {
  // Get the speed limit for a lane.
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (vehicle.get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    return vehicle_ahead.v;
  }
  // Found no vehicle in the lane
  return -1;
}

double collision_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data) {
  // Binary cost function which penalizes collisions..
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  double nearest;

  if (vehicle.get_vehicle_ahead(predictions, data["final_lane"], vehicle_ahead)) {
    if (vehicle.get_vehicle_behind(predictions, data["final_lane"], vehicle_behind)) {
      if(vehicle_ahead.s - vehicle.s < vehicle.s - vehicle_behind.s) 
      {
        nearest = vehicle_ahead.s - vehicle.s;
      }
      else
      {
        nearest = vehicle.s - vehicle_behind.s;
      }
           
    }
    else
    {
      nearest = vehicle_ahead.s - vehicle.s;
    }
  }
  else if (vehicle.get_vehicle_behind(predictions, data["final_lane"], vehicle_behind)) {
    nearest = vehicle.s - vehicle_behind.s;
  }

  if (nearest < 2*1.5)
    {
      return 1.0;
    }
    else 
    {
      return 0.0;
    }

}

double buffer_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data) {
  // Penalizes getting close to other vehicles.
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  double nearest = 999999999.0;

  if (vehicle.get_vehicle_ahead(predictions, data["final_lane"], vehicle_ahead)) {
    if (vehicle.get_vehicle_behind(predictions, data["final_lane"], vehicle_behind)) {
      if(vehicle_ahead.s - vehicle.s < vehicle.s - vehicle_behind.s) 
      {
        nearest = vehicle_ahead.s - vehicle.s;
      }
      else
      {
        nearest = vehicle.s - vehicle_behind.s;
      }
           
    }
    else
    {
      nearest = vehicle_ahead.s - vehicle.s;
    }
  }
  else if (vehicle.get_vehicle_behind(predictions, data["final_lane"], vehicle_behind)) {
    nearest = vehicle.s - vehicle_behind.s;
  }
  
  return 2.0 / (1 + exp(-(2*1.5 / nearest))) - 1.0;

}

double comfort_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data){
  return (data["intended_lane"] != data["final_lane"]);
}

double calculate_cost(Vehicle &vehicle, 
                     map<int, vector<Vehicle>> &predictions, 
                     vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  double cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<double(Vehicle &, vector<Vehicle> &, 
                             map<int, vector<Vehicle>> &, 
                             map<string, double> &)
    >> cf_list = {goal_distance_cost, inefficiency_cost, collision_cost, buffer_cost, comfort_cost};
  vector<double> weight_list = {REACH_GOAL, INEFFICIENCY, COLLISION, BUFFER, COMFORT};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    double new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
    cost += new_cost;
  }
  return cost;
}

map<string, double> get_helper_data(Vehicle &vehicle, 
                                   vector<Vehicle> &trajectory, 
                                   map<int, vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, double> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  double intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  double distance_to_goal = vehicle.goal_s - trajectory_last.s;
  double final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
    
  return trajectory_data;
}