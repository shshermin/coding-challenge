#include <neura_motion_planning_challenge/TrajectoryParser.h>
#include <ros/ros.h>
#include <sstream>
#include <map>
#include <algorithm>
#include <cctype>

using namespace neura_motion_planning_challenge;

// Helper: Parse joint names from line
void TrajectoryParser::parseJointNames(const std::string& line, std::vector<std::string>& joint_names)
{
    size_t start = line.find("[");
    size_t end = line.find("]");
    if (start == std::string::npos || end == std::string::npos) return;
    
    std::string names_str = line.substr(start + 1, end - start - 1);
    std::istringstream ss(names_str);
    std::string name;
    
    while (std::getline(ss, name, ',')) {
        // Trim whitespace
        name.erase(0, name.find_first_not_of(" \t"));
        name.erase(name.find_last_not_of(" \t") + 1);
        joint_names.push_back(name);
    }
}

// Helper: Parse vector of doubles
std::vector<double> TrajectoryParser::parseVector(const std::string& str)
{
    std::vector<double> values;
    size_t start = str.find("[");
    size_t end = str.find("]");
    if (start == std::string::npos || end == std::string::npos) return values;
    
    std::string vals_str = str.substr(start + 1, end - start - 1);
    std::istringstream ss(vals_str);
    std::string val;
    
    while (std::getline(ss, val, ',')) {
        values.push_back(std::stod(val));
    }
    return values;
}

// Helper: Parse a waypoint
void TrajectoryParser::parseWaypoint(std::ifstream& file, std::string& current_line, trajectory_msgs::JointTrajectory& joint_traj)
{
    trajectory_msgs::JointTrajectoryPoint point;
    std::string line = current_line;
    
    // Continue reading lines for this waypoint
    while (std::getline(file, line)) {
        if (line.find("    - index:") != std::string::npos) {
            // Next waypoint, put line back
            current_line = line;
            break;
        }
        if (line.find("cartesian_poses:") != std::string::npos) {
            break;
        }
        
        if (line.find("time_from_start:") != std::string::npos) {
            size_t pos = line.find(":") + 1;
            double time = std::stod(line.substr(pos));
            point.time_from_start = ros::Duration(time);
        }
        else if (line.find("positions:") != std::string::npos) {
            point.positions = parseVector(line);
        }
        else if (line.find("velocities:") != std::string::npos) {
            point.velocities = parseVector(line);
        }
        else if (line.find("accelerations:") != std::string::npos) {
            point.accelerations = parseVector(line);
        }
    }
    
    joint_traj.points.push_back(point);
}

// Helper: Parse JSON trajectory format (converts to internal trajectory structure)
bool TrajectoryParser::parseJsonTrajectory(const std::string& filepath, trajectory_msgs::JointTrajectory& joint_traj)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open JSON file: %s", filepath.c_str());
        return false;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    
    // Simple JSON parser for the specific trajectory.json format
    // Expected structure: {"00": {"trajectory": {"000": [j1,j2,...,j7], ...}, "time_from_start": {"000": t0, ...}}}
    
    // Set joint names (hardcoded for this robot - 7 DOF arm)
    joint_traj.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    
    // Find first trajectory object "trajectory": {
    size_t traj_start = content.find("\"trajectory\":");
    if (traj_start == std::string::npos) {
        ROS_ERROR("Could not find 'trajectory' key in JSON");
        return false;
    }
    
    // Find the opening brace of trajectory object
    size_t traj_brace = content.find("{", traj_start);
    if (traj_brace == std::string::npos) {
        ROS_ERROR("Invalid JSON structure - no opening brace for trajectory");
        return false;
    }
    
    // Find time_from_start section
    size_t time_start = content.find("\"time_from_start\":", traj_start);
    if (time_start == std::string::npos) {
        ROS_ERROR("Could not find 'time_from_start' key in JSON");
        return false;
    }
    
    // Parse trajectory waypoints (between traj_brace and time_start)
    std::map<std::string, std::vector<double>> waypoints;
    size_t pos = traj_brace;
    
    while (pos < time_start) {
        // Find next quote (potential key)
        size_t quote1 = content.find("\"", pos + 1);
        if (quote1 == std::string::npos || quote1 >= time_start) break;
        
        size_t quote2 = content.find("\"", quote1 + 1);
        if (quote2 == std::string::npos) break;
        
        std::string key = content.substr(quote1 + 1, quote2 - quote1 - 1);
        
        // Check if this is a 3-digit waypoint key
        if (key.length() == 3 && std::isdigit(key[0]) && std::isdigit(key[1]) && std::isdigit(key[2])) {
            // Find the array [...]
            size_t arr_start = content.find("[", quote2);
            if (arr_start == std::string::npos || arr_start >= time_start) {
                pos = quote2;
                continue;
            }
            
            size_t arr_end = content.find("]", arr_start);
            if (arr_end == std::string::npos) {
                pos = quote2;
                continue;
            }
            
            // Extract array content
            std::string arr_content = content.substr(arr_start + 1, arr_end - arr_start - 1);
            
            // Parse numbers
            std::vector<double> values;
            std::istringstream iss(arr_content);
            std::string token;
            
            while (std::getline(iss, token, ',')) {
                // Trim whitespace
                token.erase(0, token.find_first_not_of(" \t\n\r"));
                token.erase(token.find_last_not_of(" \t\n\r") + 1);
                if (!token.empty()) {
                    try {
                        values.push_back(std::stod(token));
                    } catch (...) {
                        ROS_WARN("Failed to parse value: %s", token.c_str());
                    }
                }
            }
            
            // Store if we have exactly 7 joint values
            if (values.size() == 7) {
                waypoints[key] = values;
            }
            
            pos = arr_end;
        } else {
            pos = quote2;
        }
    }
    
    ROS_INFO("Parsed %lu trajectory waypoints from JSON", waypoints.size());
    
    // Parse time_from_start values
    size_t time_brace = content.find("{", time_start);
    if (time_brace == std::string::npos) {
        ROS_ERROR("Invalid JSON structure - no opening brace for time_from_start");
        return false;
    }
    
    // Find closing brace (search for matching brace)
    int brace_count = 1;
    size_t time_end = time_brace + 1;
    while (time_end < content.length() && brace_count > 0) {
        if (content[time_end] == '{') brace_count++;
        if (content[time_end] == '}') brace_count--;
        time_end++;
    }
    
    std::map<std::string, double> times;
    pos = time_brace;
    
    while (pos < time_end) {
        size_t quote1 = content.find("\"", pos + 1);
        if (quote1 == std::string::npos || quote1 >= time_end) break;
        
        size_t quote2 = content.find("\"", quote1 + 1);
        if (quote2 == std::string::npos) break;
        
        std::string key = content.substr(quote1 + 1, quote2 - quote1 - 1);
        
        if (key.length() == 3 && std::isdigit(key[0]) && std::isdigit(key[1]) && std::isdigit(key[2])) {
            size_t colon = content.find(":", quote2);
            if (colon != std::string::npos && colon < time_end) {
                // Find the value (next number after colon)
                size_t val_start = colon + 1;
                while (val_start < time_end && (content[val_start] == ' ' || content[val_start] == '\t' || content[val_start] == '\n')) {
                    val_start++;
                }
                
                size_t val_end = val_start;
                while (val_end < time_end && (std::isdigit(content[val_end]) || content[val_end] == '.' || content[val_end] == '-' || content[val_end] == 'e' || content[val_end] == 'E')) {
                    val_end++;
                }
                
                if (val_end > val_start) {
                    std::string val_str = content.substr(val_start, val_end - val_start);
                    try {
                        times[key] = std::stod(val_str);
                    } catch (...) {
                        ROS_WARN("Failed to parse time value: %s", val_str.c_str());
                    }
                }
                pos = val_end;
            } else {
                pos = quote2;
            }
        } else {
            pos = quote2;
        }
    }
    
    ROS_INFO("Parsed %lu time values from JSON", times.size());
    
    // Combine waypoints and times - sort by key to maintain order
    std::vector<std::string> keys;
    for (const auto& wp : waypoints) {
        if (times.find(wp.first) != times.end()) {
            keys.push_back(wp.first);
        }
    }
    std::sort(keys.begin(), keys.end());
    
    for (const auto& key : keys) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = waypoints[key];
        point.time_from_start = ros::Duration(times[key]);
        joint_traj.points.push_back(point);
    }
    
    ROS_INFO("Created %lu joint trajectory points", joint_traj.points.size());
    return !joint_traj.points.empty();
}
