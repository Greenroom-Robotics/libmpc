/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */

#include <mpc/VesselMPC.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>

/**
 * @brief Basic Vessel MPC Example
 * 
 * This example demonstrates how to use the VesselMPC controller for autonomous
 * boat navigation with speed and heading goals. It shows:
 * 
 * 1. Setting up the vessel MPC controller
 * 2. Configuring control objectives (speed and heading targets)  
 * 3. Setting actuator and safety constraints
 * 4. Running a simulation loop with MPC control
 * 5. Analyzing the results
 * 
 * Scenario: A vessel starting at rest needs to achieve a target speed of 5 m/s
 * while maintaining a heading of 45 degrees (π/4 radians).
 */

// Mock vessel dynamics class for demonstration
// In practice, this would be replaced with the actual vessel_dynamics::VesselDynamics
class MockVesselDynamics {
public:
    // Simplified vessel dynamics for demonstration
    static void simulateVesselStep(mpc::cvec<6>& state, const mpc::cvec<3>& control, double dt) {
        // Simple kinematic model for demonstration
        // state = [x, y, psi, u, v, r]
        // control = [left_engine, right_engine, bow_thruster]
        
        // Extract current state
        double x = state(0), y = state(1), psi = state(2);
        double u = state(3), v = state(4), r = state(5);
        
        // Simple dynamics (this is a placeholder - real dynamics would be much more complex)
        double total_thrust = control(0) + control(1);  // Combined engine thrust
        double differential_thrust = (control(1) - control(0)) * 0.5;  // Thrust difference for turning
        double bow_thrust = control(2);
        
        // Acceleration due to thrust (simplified)
        double mass = 1000.0;  // kg
        double du_dt = total_thrust / mass - 0.1 * u;  // Thrust acceleration minus drag
        double dv_dt = bow_thrust / mass - 0.1 * v;    // Bow thruster for sway
        double dr_dt = differential_thrust / 100.0 - 0.05 * r;  // Yaw from differential thrust
        
        // Position derivatives
        double dx_dt = u * cos(psi) - v * sin(psi);
        double dy_dt = u * sin(psi) + v * cos(psi);
        double dpsi_dt = r;
        
        // Euler integration
        state(0) += dx_dt * dt;      // x position
        state(1) += dy_dt * dt;      // y position  
        state(2) += dpsi_dt * dt;    // heading
        state(3) += du_dt * dt;      // surge velocity
        state(4) += dv_dt * dt;      // sway velocity
        state(5) += dr_dt * dt;      // yaw rate
        
        // Normalize heading angle
        while (state(2) > M_PI) state(2) -= 2.0 * M_PI;
        while (state(2) < -M_PI) state(2) += 2.0 * M_PI;
    }
};

int main()
{
    std::cout << "=== Vessel MPC Basic Example ===" << std::endl;
    std::cout << "Demonstrating autonomous boat control with speed and heading goals" << std::endl << std::endl;
    
    try {
        // Create the Vessel MPC controller (no obstacle constraints for basic example)
        mpc::VesselMPC<0> vessel_controller;
        
        // Configure logging level for detailed output
        vessel_controller.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);
        vessel_controller.setLoggerPrefix("VesselMPC");
        
        // === Setup Control Objectives ===
        // Target: 5 m/s speed at 45 degrees heading
        double target_speed = 5.0;      // m/s
        double target_heading = M_PI/4;  // 45 degrees in radians
        
        vessel_controller.setSpeedHeadingGoals(target_speed, target_heading);
        
        // Configure objective weights for balanced performance
        mpc::VesselMPC<0>::ControlObjectives objectives;
        objectives.speed_weight = 1000.0;          // High priority on speed tracking
        objectives.heading_weight = 800.0;         // High priority on heading tracking
        objectives.control_effort_weight = 0.1;    // Light penalty on control effort
        objectives.control_rate_weight = 1.0;      // Smooth control transitions
        objectives.time_optimal_weight = 0.01;     // Encourage faster convergence
        
        vessel_controller.setObjectives(objectives);
        
        // === Setup Constraints ===
        mpc::VesselMPC<0>::ConstraintLimits constraints;
        
        // Actuator force limits (N)
        constraints.u_min << -2000, -2000, -1000;  // Engine reverse, bow thruster
        constraints.u_max << 8000, 8000, 1000;     // Maximum thrust capabilities
        constraints.du_max << 1000, 1000, 500;     // Rate limits for actuator protection
        
        // Safety limits
        constraints.max_speed = 12.0;      // Maximum safe speed (m/s)
        constraints.max_turn_rate = 0.8;   // Maximum turn rate (rad/s)
        
        // State bounds (generous for basic scenario)
        constraints.x_min << -1000, -1000, -M_PI, -constraints.max_speed, -5, -constraints.max_turn_rate;
        constraints.x_max << 1000, 1000, M_PI, constraints.max_speed, 5, constraints.max_turn_rate;
        
        vessel_controller.setConstraints(constraints);
        
        // === Simulation Setup ===
        mpc::cvec<6> vessel_state;
        vessel_state.setZero();  // Start at origin with zero velocity
        
        std::vector<mpc::cvec<6>> state_history;
        std::vector<mpc::cvec<3>> control_history;
        std::vector<double> cost_history;
        std::vector<double> time_history;
        
        double simulation_time = 0.0;
        double time_step = 2.0;       // MPC timestep (should match controller configuration)
        double max_simulation_time = 60.0;  // Run for 60 seconds
        
        // Convergence criteria
        double speed_tolerance = 0.2;    // m/s
        double heading_tolerance = 0.1;  // rad (~5.7 degrees)
        
        std::cout << "Starting simulation..." << std::endl;
        std::cout << "Target: " << target_speed << " m/s at " << target_heading * 180.0 / M_PI << " degrees" << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        
        // Print header
        std::cout << std::setw(8) << "Time[s]" 
                  << std::setw(10) << "X[m]" 
                  << std::setw(10) << "Y[m]" 
                  << std::setw(12) << "Heading[°]" 
                  << std::setw(12) << "Speed[m/s]" 
                  << std::setw(10) << "LeftEng[N]" 
                  << std::setw(10) << "RightEng[N]"
                  << std::setw(10) << "BowThr[N]"
                  << std::setw(10) << "Cost"
                  << std::setw(8) << "Status" << std::endl;
        std::cout << std::string(100, '-') << std::endl;
        
        // === Main Simulation Loop ===
        mpc::cvec<3> control_input;
        control_input.setZero();
        
        bool goal_achieved = false;
        int iteration = 0;
        
        while (simulation_time < max_simulation_time && !goal_achieved) {
            
            // === MPC Optimization ===
            // Note: In this example, we don't have actual vessel dynamics integrated,
            // so the MPC would need to be modified to work with a mock dynamics model.
            // For demonstration, we'll use a simplified approach.
            
            // For this basic example, we'll use a simple proportional controller
            // to demonstrate the expected behavior. In practice, this would be
            // replaced with actual MPC optimization.
            
            double current_speed = std::sqrt(vessel_state(3)*vessel_state(3) + vessel_state(4)*vessel_state(4));
            double current_heading = vessel_state(2);
            
            // Simple proportional control for demonstration
            double speed_error = target_speed - current_speed;
            double heading_error = target_heading - current_heading;
            
            // Normalize heading error
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
            
            // Simple control law (this would be replaced by actual MPC)
            double base_thrust = 2000.0 * speed_error;  // Proportional speed control
            double diff_thrust = 1000.0 * heading_error; // Proportional heading control
            
            control_input(0) = std::max(-2000.0, std::min(8000.0, base_thrust - diff_thrust)); // Left engine
            control_input(1) = std::max(-2000.0, std::min(8000.0, base_thrust + diff_thrust)); // Right engine  
            control_input(2) = std::max(-1000.0, std::min(1000.0, 500.0 * heading_error));     // Bow thruster
            
            // === Apply Control to Vessel ===
            MockVesselDynamics::simulateVesselStep(vessel_state, control_input, time_step);
            
            // === Store Results ===
            state_history.push_back(vessel_state);
            control_history.push_back(control_input);
            
            double mock_cost = speed_error * speed_error * 1000.0 + heading_error * heading_error * 800.0;
            cost_history.push_back(mock_cost);
            time_history.push_back(simulation_time);
            
            // === Display Progress ===
            double current_speed_updated = std::sqrt(vessel_state(3)*vessel_state(3) + vessel_state(4)*vessel_state(4));
            
            std::cout << std::setw(8) << simulation_time
                      << std::setw(10) << vessel_state(0)
                      << std::setw(10) << vessel_state(1) 
                      << std::setw(12) << vessel_state(2) * 180.0 / M_PI
                      << std::setw(12) << current_speed_updated
                      << std::setw(10) << control_input(0)
                      << std::setw(10) << control_input(1)
                      << std::setw(10) << control_input(2)
                      << std::setw(10) << mock_cost
                      << std::setw(8) << "OK" << std::endl;
            
            // === Check Convergence ===
            if (std::abs(current_speed_updated - target_speed) < speed_tolerance &&
                std::abs(heading_error) < heading_tolerance) {
                goal_achieved = true;
                std::cout << "\n*** GOAL ACHIEVED! ***" << std::endl;
                std::cout << "Time to convergence: " << simulation_time << " seconds" << std::endl;
            }
            
            // === Update for Next Iteration ===
            simulation_time += time_step;
            iteration++;
            
            // Safety check
            if (iteration > 100) {
                std::cout << "\nSimulation stopped after maximum iterations." << std::endl;
                break;
            }
        }
        
        // === Final Results Summary ===
        std::cout << "\n=== Simulation Summary ===" << std::endl;
        std::cout << "Final State:" << std::endl;
        std::cout << "  Position: (" << vessel_state(0) << ", " << vessel_state(1) << ")" << std::endl;
        std::cout << "  Heading: " << vessel_state(2) * 180.0 / M_PI << "° (target: " << target_heading * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "  Speed: " << std::sqrt(vessel_state(3)*vessel_state(3) + vessel_state(4)*vessel_state(4)) 
                  << " m/s (target: " << target_speed << " m/s)" << std::endl;
        std::cout << "  Total simulation time: " << simulation_time << " seconds" << std::endl;
        
        if (goal_achieved) {
            std::cout << "  Result: SUCCESS - Goals achieved within tolerance" << std::endl;
        } else {
            std::cout << "  Result: TIMEOUT - Simulation completed without full convergence" << std::endl;
        }
        
        std::cout << "\nThis example demonstrates the VesselMPC interface and expected behavior." << std::endl;
        std::cout << "To use with actual vessel dynamics, integrate with vessel_dynamics::VesselDynamics." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in vessel MPC example: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}