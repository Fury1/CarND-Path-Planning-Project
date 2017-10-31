#include <iostream>
#include <cmath>
#include <map>
#include "datavariables.h"

// Constructor
DataVariables::DataVariables() {
    // General starting values for simulator
    desired_vehicle_speed = 0.0;
    lane = LANES[1];
    speed_limit = 49.0; // 1 mph buffer under 50

    // KEEP LANE state starting flags
    too_close = false;
    plc = false;

}

// Destructor
DataVariables::~DataVariables() {}

// Update object variables with current simulator values
void DataVariables::Init(const std::vector<std::vector<double>> &sensor_fusion, double &car_s,
                         double &car_d, double &end_path_s, double &car_speed, int &previous_path_length) {
    this->sensor_fusion = sensor_fusion; // update the sensor fusion data so other state methods may access it
    this->car_s = car_s; // update car_s
    this->car_d = car_d; // update car_d
    this->end_path_s = end_path_s; // update end_path_s
    this->car_speed = car_speed; // update the current vehicle speed
    this->previous_path_length = previous_path_length; // update the previous path length
}

// Keep lane and follow vehicles when needed
void DataVariables::KeepLane() {

    /*
    * Find the desired vehicle speed and stay in the current lane while looking for cars ahead of us.
    */

    double followed_vehicle_s;
    double followed_vehicle_speed;
    double s_delta;
    double speed_delta;


    // Search for vehicles that are ahead of us
    for (int i = 0; i < sensor_fusion.size(); i++) {

        // Figure out where the other cars are
        double d = sensor_fusion[i][6];

        // Check to see if the detected vehicle is ahead in our lane
        if (d < (2 + LANE_WIDTH * lane + 2) && d > (2 + LANE_WIDTH * lane - 2)) {

            // If the car is in our lane figure out how close it is
            double vx = sensor_fusion[i][3]; // detected vehicle speed x
            double vy = sensor_fusion[i][4]; // detected vehicle speed y
            double check_speed = sqrt(vx * vx + vy *vy); // calculate xy magnitude
            double check_car_s = sensor_fusion[i][5]; // freenet "s" value of the vehicle in question

            // Project the future state of the vehicle to make up for latency
            check_car_s += ((double)previous_path_length * 0.02 * check_speed); // type cast to double

            // If the vehicle is getting to close to our car, begin to follow it
            if (check_car_s > car_s && check_car_s - end_path_s < FOLLOW_BUFFER) { // FOLLOW_BUFFER how close in freenet s values
                // Switch to the following state
                too_close = true;
                followed_vehicle_s = check_car_s;
                s_delta = check_car_s - end_path_s;
                followed_vehicle_speed = check_speed /  0.44704; // convert M/s to MPH
                speed_delta = car_speed - followed_vehicle_speed;
                /* End the search for a vehicle in front of us and follow it
                since we can only follow one vehicle at a time.*/
                break;
            }
            else too_close = false;
        }
    }
    // If we are getting to close to a vehicle decided what to do in order of precedence
    if (too_close) {
        // Check for an emergency braking situation first
        if (s_delta < 12) {
            desired_vehicle_speed -= 0.4; // strong braking
        }
        // Adjust car following distance normally if there is no emergency braking needed
        else if (s_delta <= FOLLOW_BUFFER) {
            // Adjust speed proportionately to difference in vehicle speeds (delta multiple stabilizes following distance faster)
            desired_vehicle_speed -= speed_delta / (followed_vehicle_speed + car_speed / 2) + s_delta * 0.0001;
        }
        else if (s_delta > FOLLOW_BUFFER) {
            desired_vehicle_speed += std::abs(speed_delta / (followed_vehicle_speed + car_speed / 2)) + s_delta * 0.0001;
        }
        // If we are following a slow vehicle prepare for a lane change
        if (too_close && desired_vehicle_speed < speed_limit) {
            plc = true; // start looking for a lane change to get back up to the speed limit
        }
    }
    // If not following a car drive the speed limit
    else if (!too_close && desired_vehicle_speed < speed_limit) {
        desired_vehicle_speed += 0.224;
    }
    // BEGIN DEBUG KEEP LANE
    // std::cout << "KEEP LANE STATS" << std::endl
    // << "Too close: " << too_close << std::endl
    // << "PLC: " << plc << std::endl
    // << "Car S: " << car_s << std::endl
    // << "Follow Vehicle S: " << followed_vehicle_s << std::endl
    // << "S Delta: " << s_delta << std::endl
    // << "Car Speed: " << car_speed << std::endl
    // << "Followed Vehicle Speed: " << followed_vehicle_speed << std::endl
    // << "Speed Delta: " << speed_delta << std::endl;
    // std::cout << "\033[2J\033[1;1H"; // clear console
    // END DEBUG
}

// Prepare for a lane change
void DataVariables::PrepareLaneChange() {

     /*
     * If we are following a slow vehicle start looking for a new lane to change into.
     */

    std::vector<int> available_lanes; // vector of possible lanes

    // Find the other possible lane options
    for (int i = 0; i < LANES.size(); i++) {
        if (LANES[i] == lane) {
            continue;
        }
        else available_lanes.push_back(LANES[i]);
    }

    /* Create a dictionary like data structure of available lanes and the cars present in each lane.
    (available lane, vector of vehicles in that lane)*/
    std::map<int, std::vector<std::vector<double>>> tracked_vehicles; // vehicles that could cause a collision during a lane change
    std::map<int, std::vector<std::vector<double>>> nearby_vehicles; // vehicles around us but not in the way of a lane change

    // For the available lanes found, find any vehicle in front and behind our location in those lanes
    for (int i = 0; i < available_lanes.size(); i++) {

        // Record the current lane
        int current_lane = available_lanes[i];

        // Check the current lane against all detected vehicles to find which vehicles are in this particular lane
        for (int j = 0; j < sensor_fusion.size(); j++) {

            // Figure out where the other cars are
            double d = sensor_fusion[j][6];

            // If a detected vehicle is in the current lane and close enough to worry about track it
            if (d < (2 + LANE_WIDTH * current_lane + 2) && d > (2 + LANE_WIDTH * current_lane - 2)) {
                // If the car is in our lane figure out how close it is
                double vx = sensor_fusion[j][3]; // detected vehicle speed x
                double vy = sensor_fusion[j][4]; // detected vehicle speed y
                double check_speed = sqrt(vx * vx + vy *vy); // calculate xy magnitude
                double check_car_s = sensor_fusion[j][5]; // freenet "s" value of the vehicle in question
                double check_car_speed = check_speed /  0.44704; // convert M/s to MPH

                // Project the future state of the vehicle to make up for latency
                check_car_s += ((double)previous_path_length * 0.02 * check_speed); // type cast to double

                // Find the cars close to us that could get in the way of a lane change
                // if (behind us && front of us)
                if (check_car_s > car_s - 4 && check_car_s < car_s + 9) {
                    // Keep track of vehicles that could cause a collision
                    tracked_vehicles[current_lane].push_back(sensor_fusion[j]);
                }
                // Check for vehicles that are not in the way but near us ahead
                else if (check_car_s > car_s + 9 && check_car_s < car_s + 20) {
                    // Keep track of other nearby vehicles in the area
                    nearby_vehicles[current_lane].push_back(sensor_fusion[j]);
                }
            }
        }
    }

    // Find the best lane
    int best_lane_option = ChooseLane(available_lanes, tracked_vehicles, nearby_vehicles);

    // BEGIN DEBUG PREPARE FOR LANE CHANGE
    // std::cout << "PREPARE LANE CHANGE" << std::endl;
    // std::cout << "Car S: " << car_s << std::endl;
    // std::cout << "Car Speed: " << car_speed << std::endl;
    // std::cout << "Desired Lane: " << best_lane_option << std::endl;
    // std::cout << "Too Close: " << too_close << std::endl;

    // // Print out the tracked vehicles as LANE:VEHICLES
    // for (auto &lane : tracked_vehicles) {
    //     // Print out the lane we are looking at first
    //     std::cout << "Tracked cars in lane " << lane.first << ":" << std::endl;
    //     for (auto &vehicle : lane.second) { // vector of vehicles and their values (std::vector<std::vector<double>>)
    //         for (auto it = vehicle.begin(); it < vehicle.end(); it++) { // print out the details of each vehicle
    //             std::cout << *it << " ";
    //         }
    //     std::cout << std::endl << "---" << std::endl; // go to the next line for a new vehicle
    //     }
    // }
    // // Print out the nearby vehicles as LANE:VEHICLES
    // for (auto &lane : nearby_vehicles) {
    //     // Print out the lane we are looking at first
    //     std::cout << "Nearby cars in lane " << lane.first << ":" << std::endl;
    //     for (auto &vehicle : lane.second) { // vector of vehicles and their values (std::vector<std::vector<double>>)
    //         for (auto it = vehicle.begin(); it < vehicle.end(); it++) { // print out the details of each vehicle
    //             std::cout << *it << " ";
    //         }
    //     std::cout << std::endl << "---" << std::endl; // go to the next line for a new vehicle
    //     }
    // }
    // std::cout << "\033[2J\033[1;1H"; // clear console
    // END DEBUG

    // Change into the best lane found
    LaneChange(best_lane_option);
}

// Find what the best lane change will be (L or R)
double DataVariables::ChooseLane(std::vector<int> available_lanes, std::map<int, std::vector<std::vector<double>>> tracked_vehicles,
                                 std::map<int, std::vector<std::vector<double>>> nearby_vehicles) {
    /*
    * Figure out what the next best lane choice will be to pass the slow moving vehicle ahead of us.
    * If a better lane can't be found currently, wait until a better choice is available.
    */

    int best_lane_option = lane; // current lane
    double best_lane_cost = 1 - exp(-1 / car_speed); //current lane cost

    // Find the best lane
    for (int i = 0; i < available_lanes.size(); i++) {
        /* Index into the tracked vehicles. Check to see if the tracked_vehicles for the available lane
        is 0, IE: free of any cars that will get in the way of the lane change.
        Also make sure we are not changing more then one lane at a time.
        (We can safely index into tracked vehicles
        because it was created with available_lanes, so there is no risk of making a new entry
        inadvertently causing false vehicle positives)*/
        if (tracked_vehicles[available_lanes[i]].size() == 0 && abs(lane - available_lanes[i]) == 1) {

            /*
            * Now that there is a safe lane change availabe, see if that lane is moving any faster on average then us.
            * Or even better, if there are no nearby vehicles to slow us down at all.
            */

             // Set vehicle vector iterator
            auto vehicles = nearby_vehicles[available_lanes[i]]; // look at the other vehicles and speeds in the available lane
            std::vector<double> lane_speed; // speeds of vehicles in the lane to be averaged

            // Check all of the nearby vehicle speeds in the lane we could switch in to
            for (auto vehicle = vehicles.begin(); vehicle != vehicles.end(); vehicle++) {

                double vx = (*vehicle)[3]; // detected vehicle speed x
                double vy = (*vehicle)[4]; // detected vehicle speed y
                double check_speed = sqrt(vx * vx + vy * vy); // calculate xy magnitude
                double check_car_speed = check_speed /  0.44704; // convert M/s to MPH

                // Record the speed
                lane_speed.push_back(check_car_speed);
            }
            // Average speed
            double average = 0;

            // If there are NO nearby vehicles in the lane adjascent us, switch lanes because its the best option
            if (lane_speed.size() < 1) {
                best_lane_option = available_lanes[i];
            }
            // If there are nearby vehicles in the lane adjascent to us evaluate the lane's average speed
            else if (lane_speed.size() > 0) {
                for (int i = 0; i < lane_speed.size(); i++) {
                    average += lane_speed[i];
                }
                // Calculate average
               average = average / lane_speed.size();

                // Available lane cost (if average speed is much faster the lane is better despite the nearby cars)
               double cost = 1 - exp(-1 / average * 0.02) + 0.05; // penalize extra cars

                // Check if the speed is better then ours and its cost is lower despite the extra vehicles
                if (average > car_speed && cost < best_lane_cost) {
                    best_lane_option = available_lanes[i];
                }
                else continue; // traffic is moving slower in the adjacent lane even though we could change lanes, wait
            }
        else continue; // wait until there is a safe adjacent lane to switch into
        }
    }
    // Return the best lane identified
    return best_lane_option;
}

// Switch into the best lane
void DataVariables::LaneChange(int lane) {

     /*
     * Switch into the specified lane.
     */

    this->lane = lane; // change the lane
    plc = false; // set prepare lane change back to false so KeepLane() can set it again later if need be
}