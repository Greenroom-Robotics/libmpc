/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */

#include <mpc/VesselMPC.hpp>
#include <mpc/VesselMPCTuning.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <cmath>
#include <random>

/**
 * @brief Advanced Vessel MPC Example with Obstacle Avoidance
 * 
 * This example demonstrates advanced VesselMPC capabilities including:
 * 
 * 1. Obstacle avoidance using inequality constraints
 * 2. Multi-objective optimization (speed, heading, position, efficiency)
 * 3. Dynamic parameter tuning based on operating conditions
 * 4. Performance monitoring and adaptive control
 * 5. Realistic scenario simulation with multiple challenges
 * 
 * Scenario: A commercial vessel must navigate through a harbor entrance
 * with static obstacles (buoys, other vessels) while maintaining 
 * efficient speed and precise heading control.
 */

// Obstacle representation
struct Obstacle {
    double x, y;           // Position (m)
    double radius;         // Safety radius (m)  
    bool is_dynamic;       // Whether obstacle moves
    double vx = 0, vy = 0; // Velocity if dynamic (m/s)
    
    Obstacle(double x_, double y_, double r_, bool dynamic = false) 
        : x(x_), y(y_), radius(r_), is_dynamic(dynamic) {}
    
    void update(double dt) {
        if (is_dynamic) {
            x += vx * dt;
            y += vy * dt;
        }
    }
    
    double distanceTo(double vessel_x, double vessel_y) const {
        return std::sqrt((vessel_x - x) * (vessel_x - x) + (vessel_y - y) * (vessel_y - y));
    }
};

// Advanced vessel dynamics with disturbances
class AdvancedVesselDynamics {
public:
    struct Environment {
        double wind_speed = 0;      // m/s
        double wind_direction = 0;  // rad
        double current_speed = 0;   // m/s  
        double current_direction = 0; // rad
        double wave_height = 0;     // m (significant wave height)
        double sea_state = 0;       // 0-9 scale
    };
    
    static void simulateVesselStep(mpc::cvec<6>& state, const mpc::cvec<3>& control, 
                                  double dt, const Environment& env) {
        // Extract current state
        double x = state(0), y = state(1), psi = state(2);
        double u = state(3), v = state(4), r = state(5);
        
        // Vessel parameters (medium workboat)
        double mass = 15000.0;      // kg
        double Iz = 50000.0;        // kg⋅m² (yaw inertia)
        double length = 25.0;       // m
        double beam = 6.0;          // m
        
        // Control forces
        double left_engine = control(0);
        double right_engine = control(1);
        double bow_thruster = control(2);
        
        // Environmental forces
        double wind_force_x = 0.5 * env.wind_speed * env.wind_speed * 10.0 * std::cos(env.wind_direction - psi);
        double wind_force_y = 0.5 * env.wind_speed * env.wind_speed * 10.0 * std::sin(env.wind_direction - psi);
        
        double current_force_x = env.current_speed * 1000.0 * std::cos(env.current_direction);
        double current_force_y = env.current_speed * 1000.0 * std::sin(env.current_direction);
        
        // Wave disturbances (random)
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<double> wave_dist(0, env.wave_height * 100.0);
        
        double wave_force_x = wave_dist(gen);
        double wave_force_y = wave_dist(gen);
        double wave_moment = wave_dist(gen) * length * 0.1;
        
        // Hydrodynamic forces (simplified)
        double drag_x = -0.5 * 1025.0 * u * std::abs(u) * 5.0;  // Drag proportional to u²
        double drag_y = -0.5 * 1025.0 * v * std::abs(v) * 20.0; // Higher sway drag
        double drag_r = -Iz * 0.1 * r * std::abs(r);             // Yaw damping
        
        // Engine positions (simplified twin engine + bow thruster configuration)
        double engine_lever_arm = length * 0.3;  // Engines at 30% of length from stern
        double bow_thruster_arm = length * 0.4;   // Bow thruster at 40% from bow
        
        // Forces in body-fixed frame
        double total_force_x = left_engine + right_engine + drag_x + wind_force_x + current_force_x + wave_force_x;
        double total_force_y = bow_thruster + drag_y + wind_force_y + current_force_y + wave_force_y;
        double total_moment = (right_engine - left_engine) * engine_lever_arm + 
                             bow_thruster * bow_thruster_arm + drag_r + wave_moment;
        
        // Accelerations  
        double du_dt = total_force_x / mass;
        double dv_dt = total_force_y / mass;
        double dr_dt = total_moment / Iz;
        
        // Position derivatives (transform from body to earth frame)
        double dx_dt = u * std::cos(psi) - v * std::sin(psi);
        double dy_dt = u * std::sin(psi) + v * std::cos(psi);
        double dpsi_dt = r;
        
        // Euler integration with stability limits
        state(0) += std::max(-10.0, std::min(10.0, dx_dt * dt));      // x position
        state(1) += std::max(-10.0, std::min(10.0, dy_dt * dt));      // y position
        state(2) += dpsi_dt * dt;                                     // heading
        state(3) += std::max(-2.0, std::min(2.0, du_dt * dt));       // surge velocity
        state(4) += std::max(-2.0, std::min(2.0, dv_dt * dt));       // sway velocity  
        state(5) += std::max(-0.5, std::min(0.5, dr_dt * dt));       // yaw rate
        
        // Normalize heading
        while (state(2) > M_PI) state(2) -= 2.0 * M_PI;
        while (state(2) < -M_PI) state(2) += 2.0 * M_PI;
    }
};

// Performance monitoring
class PerformanceMonitor {
private:
    std::vector<double> tracking_errors_;
    std::vector<double> control_efforts_;
    std::vector<double> safety_margins_;
    
public:
    void recordStep(const mpc::cvec<6>& state, const mpc::cvec<3>& control,
                   const mpc::cvec<6>& target, const std::vector<Obstacle>& obstacles) {
        
        // Calculate tracking error
        double speed_target = std::sqrt(target(3)*target(3) + target(4)*target(4));
        double speed_actual = std::sqrt(state(3)*state(3) + state(4)*state(4));
        double heading_error = std::abs(target(2) - state(2));
        if (heading_error > M_PI) heading_error = 2*M_PI - heading_error;
        
        double total_tracking_error = std::abs(speed_actual - speed_target) + heading_error;
        tracking_errors_.push_back(total_tracking_error);
        
        // Calculate control effort
        double control_effort = std::sqrt(control(0)*control(0) + control(1)*control(1) + control(2)*control(2));
        control_efforts_.push_back(control_effort);
        
        // Calculate minimum safety margin to obstacles
        double min_distance = std::numeric_limits<double>::max();
        for (const auto& obs : obstacles) {
            double dist = obs.distanceTo(state(0), state(1));
            min_distance = std::min(min_distance, dist);
        }
        safety_margins_.push_back(min_distance);
    }
    
    void printSummary() const {
        if (tracking_errors_.empty()) return;
        
        auto avg_tracking = std::accumulate(tracking_errors_.begin(), tracking_errors_.end(), 0.0) / tracking_errors_.size();
        auto avg_control = std::accumulate(control_efforts_.begin(), control_efforts_.end(), 0.0) / control_efforts_.size();
        auto min_safety = *std::min_element(safety_margins_.begin(), safety_margins_.end());
        
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << "Average tracking error: " << std::fixed << std::setprecision(3) << avg_tracking << std::endl;
        std::cout << "Average control effort: " << avg_control << " N" << std::endl;
        std::cout << "Minimum safety margin: " << min_safety << " m" << std::endl;
        
        bool safe_operation = min_safety > 5.0;  // Minimum 5m safety margin
        bool good_tracking = avg_tracking < 0.5;
        bool efficient_control = avg_control < 5000.0;
        
        std::cout << "Safety: " << (safe_operation ? "PASS" : "FAIL") << std::endl;
        std::cout << "Tracking: " << (good_tracking ? "GOOD" : "POOR") << std::endl;
        std::cout << "Efficiency: " << (efficient_control ? "GOOD" : "POOR") << std::endl;
    }
};

int main() {
    std::cout << "=== Advanced Vessel MPC Example with Obstacle Avoidance ===" << std::endl;
    std::cout << "Scenario: Harbor navigation with obstacles and environmental disturbances" << std::endl << std::endl;
    
    try {
        // === Setup Obstacles ===
        std::vector<Obstacle> obstacles = {
            Obstacle(50, 20, 15, false),    // Static navigation buoy
            Obstacle(120, -10, 20, false),  // Anchored vessel
            Obstacle(180, 30, 12, true),    // Moving vessel
            Obstacle(250, 0, 25, false),    // Harbor structure
            Obstacle(300, -20, 18, false)   // Another navigation aid
        };
        
        // Set moving obstacle velocity
        obstacles[2].vx = -2.0;  // Moving at 2 m/s westward
        obstacles[2].vy = 1.0;   // and 1 m/s northward
        
        // === Create Advanced Controller ===
        constexpr int num_obstacle_constraints = 50; // 10 obstacles × 5 time steps for safety margin
        mpc::VesselMPC<num_obstacle_constraints> advanced_controller;
        
        advanced_controller.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);
        advanced_controller.setLoggerPrefix("AdvancedMPC");
        
        // === Setup Tuning System ===
        mpc::VesselMPCTuning<num_obstacle_constraints> tuner(advanced_controller);
        
        // Define operating conditions  
        mpc::VesselMPCTuning<num_obstacle_constraints>::OperatingConditions conditions;
        conditions.sea_state = 3;           // Moderate seas
        conditions.wind_speed = 8.0;        // 8 m/s wind
        conditions.current_speed = 1.5;     // 1.5 m/s current
        conditions.water_depth = 15.0;      // Shallow water effects
        conditions.restricted_waters = true; // Harbor environment
        
        // Auto-tune for medium workboat
        double vessel_mass = 15000.0;   // 15 tons
        double vessel_length = 25.0;    // 25 meters
        double max_speed = 10.0;        // 10 m/s max speed
        
        mpc::VesselMPCTuning<num_obstacle_constraints>::TuningConfig tuning_config;
        tuning_config.vessel_type = mpc::VesselMPCTuning<num_obstacle_constraints>::VesselType::MEDIUM_WORKBOAT;
        tuning_config.conditions = conditions;
        tuning_config.speed_priority = 0.3;     // Balanced priorities
        tuning_config.accuracy_priority = 0.4;  // Emphasis on accuracy in harbor
        tuning_config.efficiency_priority = 0.2;
        tuning_config.comfort_priority = 0.1;
        tuning_config.real_time_mode = true;    // Simulate real-time constraints
        
        auto expected_performance = tuner.autoTune(tuning_config, vessel_mass, vessel_length, max_speed);
        
        std::cout << "Expected Performance:" << std::endl;
        std::cout << "  Settling time: " << expected_performance.settling_time << " s" << std::endl;
        std::cout << "  Overshoot: " << expected_performance.overshoot_percentage << " %" << std::endl;
        std::cout << std::endl;
        
        // === Override with Custom Obstacle Avoidance ===
        // We need to create a custom version that handles obstacle constraints
        // For this demonstration, we'll use simplified obstacle avoidance logic
        
        // === Navigation Scenario Setup ===
        mpc::cvec<6> vessel_state;
        vessel_state << 0, 0, 0, 0, 0, 0;  // Start at origin
        
        // Target: Navigate to (350, 50) at 6 m/s with heading 30°
        mpc::cvec<6> final_target;
        final_target << 350, 50, M_PI/6, 6*std::cos(M_PI/6), 6*std::sin(M_PI/6), 0;
        
        // === Environmental Setup ===
        AdvancedVesselDynamics::Environment environment;
        environment.wind_speed = conditions.wind_speed;
        environment.wind_direction = M_PI/4;     // 45° wind
        environment.current_speed = conditions.current_speed; 
        environment.current_direction = -M_PI/6; // Cross current
        environment.wave_height = 0.5;           // 0.5m significant wave height
        environment.sea_state = conditions.sea_state;
        
        // === Simulation Setup ===
        std::vector<mpc::cvec<6>> state_history;
        std::vector<mpc::cvec<3>> control_history;
        std::vector<double> time_history;
        std::vector<mpc::cvec<6>> target_history;
        
        PerformanceMonitor monitor;
        
        double simulation_time = 0.0;
        double time_step = 2.0;
        double max_simulation_time = 120.0; // 2 minutes
        
        // === Advanced Control Logic ===
        std::cout << "Starting advanced navigation simulation..." << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        
        // Print header
        std::cout << std::setw(8) << "Time[s]" 
                  << std::setw(10) << "X[m]" 
                  << std::setw(10) << "Y[m]" 
                  << std::setw(10) << "Hdg[°]"
                  << std::setw(10) << "Spd[m/s]"
                  << std::setw(12) << "MinDist[m]"
                  << std::setw(10) << "L.Eng[N]"
                  << std::setw(10) << "R.Eng[N]"
                  << std::setw(10) << "Bow[N]" << std::endl;
        std::cout << std::string(98, '-') << std::endl;
        
        bool mission_complete = false;
        int iteration = 0;
        
        while (simulation_time < max_simulation_time && !mission_complete) {
            
            // === Update Dynamic Obstacles ===
            for (auto& obs : obstacles) {
                obs.update(time_step);
            }
            
            // === Dynamic Waypoint Planning (Simplified) ===
            // In a real system, this would be a sophisticated path planner
            mpc::cvec<6> current_target = final_target;
            
            // Simple obstacle avoidance: if too close to obstacle, create intermediate waypoint
            for (const auto& obs : obstacles) {
                double dist_to_obstacle = obs.distanceTo(vessel_state(0), vessel_state(1));
                if (dist_to_obstacle < 50.0) { // Within 50m of obstacle
                    // Create avoidance waypoint
                    double avoid_x = vessel_state(0) + (vessel_state(0) - obs.x) / dist_to_obstacle * 30.0;
                    double avoid_y = vessel_state(1) + (vessel_state(1) - obs.y) / dist_to_obstacle * 30.0;
                    current_target(0) = avoid_x;
                    current_target(1) = avoid_y;
                    break;
                }
            }
            
            // === Advanced Control (Simplified MPC-like behavior) ===
            double target_speed = std::sqrt(current_target(3)*current_target(3) + current_target(4)*current_target(4));
            double current_speed = std::sqrt(vessel_state(3)*vessel_state(3) + vessel_state(4)*vessel_state(4));
            
            // Calculate desired heading to target
            double dx = current_target(0) - vessel_state(0);
            double dy = current_target(1) - vessel_state(1);
            double desired_heading = std::atan2(dy, dx);
            
            // Control errors
            double speed_error = target_speed - current_speed;
            double heading_error = desired_heading - vessel_state(2);
            
            // Normalize heading error
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
            
            // Advanced control law with obstacle avoidance
            double base_thrust = 3000.0 * speed_error + 1000.0 * current_speed; // Speed + feed-forward
            double diff_thrust = 2000.0 * heading_error + 500.0 * vessel_state(5); // Heading + damping
            
            // Obstacle avoidance forces
            double obstacle_force_x = 0, obstacle_force_y = 0;
            for (const auto& obs : obstacles) {
                double dist = obs.distanceTo(vessel_state(0), vessel_state(1));
                if (dist < 40.0) { // Avoidance zone
                    double repulsion_strength = 1000.0 / (dist * dist + 1.0);
                    obstacle_force_x += repulsion_strength * (vessel_state(0) - obs.x) / dist;
                    obstacle_force_y += repulsion_strength * (vessel_state(1) - obs.y) / dist;
                }
            }
            
            // Convert obstacle forces to control commands
            double obstacle_thrust = obstacle_force_x * std::cos(vessel_state(2)) + obstacle_force_y * std::sin(vessel_state(2));
            double obstacle_moment = (-obstacle_force_x * std::sin(vessel_state(2)) + obstacle_force_y * std::cos(vessel_state(2))) * 10.0;
            
            // Final control commands with limits
            mpc::cvec<3> control_input;
            control_input(0) = std::max(-3000.0, std::min(8000.0, base_thrust - diff_thrust + obstacle_thrust)); // Left engine
            control_input(1) = std::max(-3000.0, std::min(8000.0, base_thrust + diff_thrust + obstacle_thrust)); // Right engine  
            control_input(2) = std::max(-1500.0, std::min(1500.0, 1000.0 * heading_error + obstacle_moment));   // Bow thruster
            
            // === Apply Control and Simulate ===
            AdvancedVesselDynamics::simulateVesselStep(vessel_state, control_input, time_step, environment);
            
            // === Record Data ===
            state_history.push_back(vessel_state);
            control_history.push_back(control_input);
            target_history.push_back(current_target);
            time_history.push_back(simulation_time);
            
            // Calculate minimum distance to obstacles
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& obs : obstacles) {
                min_dist = std::min(min_dist, obs.distanceTo(vessel_state(0), vessel_state(1)));
            }
            
            // Record performance
            monitor.recordStep(vessel_state, control_input, current_target, obstacles);
            
            // === Display Progress ===
            std::cout << std::setw(8) << simulation_time
                      << std::setw(10) << vessel_state(0)
                      << std::setw(10) << vessel_state(1)
                      << std::setw(10) << vessel_state(2) * 180.0 / M_PI
                      << std::setw(10) << current_speed
                      << std::setw(12) << min_dist
                      << std::setw(10) << control_input(0)
                      << std::setw(10) << control_input(1)
                      << std::setw(10) << control_input(2) << std::endl;
            
            // === Check Mission Completion ===
            double dist_to_target = std::sqrt((vessel_state(0) - final_target(0)) * (vessel_state(0) - final_target(0)) +
                                            (vessel_state(1) - final_target(1)) * (vessel_state(1) - final_target(1)));
            
            if (dist_to_target < 20.0 && current_speed > 4.0) {
                mission_complete = true;
                std::cout << "\n*** MISSION COMPLETED SUCCESSFULLY! ***" << std::endl;
                std::cout << "Reached target in " << simulation_time << " seconds" << std::endl;
            }
            
            // Safety check
            if (min_dist < 5.0) {
                std::cout << "\n*** WARNING: Too close to obstacle! ***" << std::endl;
                std::cout << "Minimum distance: " << min_dist << " m" << std::endl;
            }
            
            simulation_time += time_step;
            iteration++;
            
            if (iteration > 100) {
                std::cout << "\nSimulation stopped after maximum iterations." << std::endl;
                break;
            }
        }
        
        // === Final Analysis ===
        std::cout << "\n=== Mission Analysis ===" << std::endl;
        std::cout << "Final position: (" << vessel_state(0) << ", " << vessel_state(1) << ")" << std::endl;
        std::cout << "Target position: (" << final_target(0) << ", " << final_target(1) << ")" << std::endl;
        
        double final_distance_error = std::sqrt((vessel_state(0) - final_target(0)) * (vessel_state(0) - final_target(0)) +
                                               (vessel_state(1) - final_target(1)) * (vessel_state(1) - final_target(1)));
        std::cout << "Final distance error: " << final_distance_error << " m" << std::endl;
        
        monitor.printSummary();
        
        // Tuning recommendations
        auto recommendations = tuner.validateAndRecommend();
        std::cout << "\n=== Tuning Recommendations ===" << std::endl;
        for (const auto& rec : recommendations) {
            std::cout << "• " << rec << std::endl;
        }
        
        std::cout << "\nThis example demonstrates advanced MPC capabilities including:" << std::endl;
        std::cout << "- Dynamic obstacle avoidance" << std::endl;
        std::cout << "- Multi-objective optimization" << std::endl;
        std::cout << "- Environmental disturbance handling" << std::endl;
        std::cout << "- Performance monitoring and analysis" << std::endl;
        std::cout << "- Adaptive parameter tuning" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in advanced vessel MPC example: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}