#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

double calculate_cost(Vehicle &vehicle, 
                     map<int, vector<Vehicle>> &predictions, 
                     vector<Vehicle> &trajectory);

double goal_distance_cost(Vehicle &vehicle,  
                         vector<Vehicle> &trajectory,  
                         map<int, vector<Vehicle>> &predictions, 
                         map<string, double> &data);

double inefficiency_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

/* double efficiency_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data); */

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> &predictions, int lane);

double collision_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double buffer_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double comfort_cost(Vehicle &vehicle, 
                        vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

                        
map<string, double> get_helper_data(Vehicle &vehicle, 
                                   vector<Vehicle> &trajectory, 
                                   map<int, vector<Vehicle>> &predictions);

#endif  // COST_H