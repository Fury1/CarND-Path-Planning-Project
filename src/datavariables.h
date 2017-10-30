/*
*
* This is a data class to hold variables and methods that need to be maintained while driving and fed in
* and out of the simulator.
*
*/

#ifndef DATAVARIABLES_H
#define DATAVARIABLES_H

#include <vector>


class DataVariables {
    public:
        // Path Planning Constants
        const int PATH_LENGTH = 50; // length of waypoint path
        const int LANE_CENTER = 2; // center of lane measurement
        const int LANE_WIDTH = 4; // width of lane in meters
        const std::vector<int> LANES = {0, 1, 2}; // lanes to choose from
        const int FOLLOW_BUFFER = 20; // vehicle following distance buffer in freenet "s"

        // Variables that need to be maintained during h.onMessage()
        double desired_vehicle_speed; // desired speed of vehicle
        int lane; // current lane (far left lane is starting lane counting from left to right (0,1,2))
        double speed_limit; // speed limit for vehicle
        std::vector<std::vector<double>> sensor_fusion; // container for current sensor fusion data
        double car_s; // car s value
        double car_d; // car d value
        double end_path_s; // end s value for last waypoint path
        double car_speed; // current vehicle speed in mph
        int previous_path_length; // length of previous waypoint path

        // KeepLane() state variables
        bool too_close; // flag for signaling we are getting to close to the vehicle in front of us in our lane
        bool plc; // flag for signaling we should look for a lane change

        // Constructor
        DataVariables();

        // Destructor
        virtual ~DataVariables();

        // Initializes data to current simulator values
        void Init(const std::vector<std::vector<double>> &sensor_fusion, double &car_s, double &car_d,
                  double &end_path_s, double &car_speed, int &previous_path_length);

        // Keep lane state and follow vehicles when needed
        void KeepLane();

        // Prepare for lane change state
        void PrepareLaneChange();

        // Change lanes transition
        void LaneChange(int lane);

        // Decided to change lanes right, left, or just wait
        double ChooseLane(std::vector<int> available_lanes, std::map<int, std::vector<std::vector<double>>> tracked_vehicles,
                          std::map<int, std::vector<std::vector<double>>> nearby_vehicles);
};
#endif