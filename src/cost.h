#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

double calculate_cost(const Vehicle &vehicle, 
                     const map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory);

double goal_distance_cost(const Vehicle &vehicle,  
                         const vector<Vehicle> &trajectory,  
                         const map<int, vector<Vehicle>> &predictions, 
                         map<string, double> &data);

double inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double efficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double lane_speed(const Vehicle &vehicle, const map<int, vector<Vehicle>> &predictions, int lane);

double collision_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double buffer_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

double comfort_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, double> &data);

                        
map<string, double> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, vector<Vehicle>> &predictions);

#endif  // COST_H