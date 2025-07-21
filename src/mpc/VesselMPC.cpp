/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */

#include <mpc/VesselMPC.hpp>

// Include vessel dynamics header for integration
// Note: This assumes the vessel dynamics header is available in the include path
// In practice, this would need to be properly configured in the build system
// #include "vessel_dynamics/vessel_dynamics.hpp"

namespace mpc
{
    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::setVesselDynamics(std::shared_ptr<vessel_dynamics::VesselDynamics> dynamics)
    {
        vessel_dynamics_ = dynamics;
        
        if (dynamics != nullptr) {
            // Configure the MPC callbacks to use the vessel dynamics
            configureDynamicsCallbacks();
            configureObjectiveFunction();
            configureConstraints();
            
            dynamics_configured_ = true;
            
            Logger::instance().log(Logger::LogType::INFO) 
                << "Vessel dynamics successfully configured for MPC" << std::endl;
        } else {
            dynamics_configured_ = false;
            Logger::instance().log(Logger::LogType::ALERT) 
                << "Warning: Vessel dynamics set to null pointer" << std::endl;
        }
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::setSpeedHeadingGoals(double target_speed, double target_heading)
    {
        objectives_.target_speed = target_speed;
        objectives_.target_heading = normalizeAngle(target_heading);
        objectives_.use_speed_heading_mode = true;
        
        // Update the objective function with new targets
        if (dynamics_configured_) {
            configureObjectiveFunction();
        }
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Speed/heading goals set: speed=" << target_speed 
            << " m/s, heading=" << target_heading << " rad" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::setStateGoals(const cvec<6>& target_state)
    {
        objectives_.target_state = target_state;
        objectives_.target_speed = calculateSpeed(target_state(VEL_U), target_state(VEL_V));
        objectives_.target_heading = normalizeAngle(target_state(HEADING));
        objectives_.use_speed_heading_mode = false;
        
        // Update the objective function with new targets  
        if (dynamics_configured_) {
            configureObjectiveFunction();
        }
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Full state goals set: [" << target_state.transpose() << "]" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::setObjectives(const ControlObjectives& objectives)
    {
        objectives_ = objectives;
        
        // Update the objective function with new weights
        if (dynamics_configured_) {
            configureObjectiveFunction();
        }
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Objective weights updated" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::setConstraints(const ConstraintLimits& constraints)
    {
        constraints_ = constraints;
        
        // Update the constraint functions with new limits
        if (dynamics_configured_) {
            configureConstraints();
        }
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Constraint limits updated" << std::endl;
    }

    template <int Tnx_obs>
    Result<3> VesselMPC<Tnx_obs>::optimizeControl(const cvec<6>& current_state, const cvec<3>& warm_start)
    {
        if (!dynamics_configured_) {
            Logger::instance().log(Logger::LogType::ALERT) 
                << "Error: Vessel dynamics not configured. Call setVesselDynamics() first." << std::endl;
            
            Result<3> error_result;
            error_result.status = ResultStatus::ERROR;
            error_result.solver_status_msg = "Vessel dynamics not configured";
            return error_result;
        }

        // Use the parent NLMPC optimize method with warm start
        cvec<3> warm_start_input = warm_start.isZero() ? last_control_input_ : warm_start;
        auto result = this->optimize(current_state, warm_start_input);
        
        // Store the result for next iteration warm start
        if (result.status == ResultStatus::SUCCESS) {
            last_control_input_ = result.cmd;
        }
        
        Logger::instance().log(Logger::LogType::DETAIL) 
            << "MPC optimization completed with status: " << result.status
            << ", cost: " << result.cost << std::endl;
        
        return result;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::configureDefaultSolverParameters()
    {
        NLParameters params;
        
        // Configure for marine applications - prioritize robustness over speed
        params.maximum_iteration = 150;           // Allow more iterations for complex marine dynamics
        params.relative_ftol = 1e-8;              // Tight cost convergence for precision
        params.relative_xtol = 1e-8;              // Tight variable convergence
        params.absolute_ftol = 1e-8;              // Absolute cost tolerance
        params.absolute_xtol = 1e-8;              // Absolute variable tolerance  
        params.time_limit = 2.0;                  // 2 second time limit for real-time operation
        params.hard_constraints = true;          // Enforce safety constraints strictly
        params.enable_warm_start = true;         // Use warm start for performance
        
        this->setOptimizerParameters(params);
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Default solver parameters configured for marine applications" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::configureDynamicsCallbacks()
    {
        if (!vessel_dynamics_) {
            Logger::instance().log(Logger::LogType::ALERT) 
                << "Error: Cannot configure dynamics - vessel_dynamics_ is null" << std::endl;
            return;
        }

        // Setup the state space function using vessel dynamics
        auto dynamics_callback = [this](cvec<6>& dx, const cvec<6>& x, const cvec<3>& u, const unsigned int& step) {
            
            // Convert MPC state to vessel dynamics format
            // MPC uses: [x, y, psi, u, v, r]
            // VesselDynamics uses: geometry_msgs::msg::Twist for velocity
            
            // Create velocity message in FLU frame
            geometry_msgs::msg::Twist velocity_stw;
            velocity_stw.linear.x = x(VEL_U);   // Surge velocity
            velocity_stw.linear.y = x(VEL_V);   // Sway velocity  
            velocity_stw.linear.z = 0.0;        // Heave (not used)
            velocity_stw.angular.x = 0.0;       // Roll rate (not used)
            velocity_stw.angular.y = 0.0;       // Pitch rate (not used)  
            velocity_stw.angular.z = x(VEL_R);  // Yaw rate
            
            // Create actuator commands from MPC controls
            std::vector<maritime_control_msgs::msg::Actuator> actuator_commands;
            // Note: This is a simplified mapping - in practice you'd need to properly
            // map the MPC control forces to specific actuator commands based on the
            // vessel configuration
            
            // For now, using placeholder mapping:
            // u(0) -> left engine, u(1) -> right engine, u(2) -> bow thruster
            maritime_control_msgs::msg::Actuator left_engine, right_engine, bow_thruster;
            left_engine.value = u(LEFT_ENGINE);
            right_engine.value = u(RIGHT_ENGINE);  
            bow_thruster.value = u(BOW_THRUSTER);
            
            actuator_commands = {left_engine, right_engine, bow_thruster};
            
            // Get acceleration from vessel dynamics
            double current_time = step * 2.0;  // Assuming 2s timestep
            auto acceleration = vessel_dynamics_->step_acceleration(velocity_stw, actuator_commands, current_time);
            
            // Convert acceleration back to MPC state derivative format
            dx(POS_X) = x(VEL_U);  // Position derivative is velocity
            dx(POS_Y) = x(VEL_V);
            dx(HEADING) = x(VEL_R); // Heading derivative is yaw rate
            dx(VEL_U) = acceleration.linear.x;   // Surge acceleration
            dx(VEL_V) = acceleration.linear.y;   // Sway acceleration  
            dx(VEL_R) = acceleration.angular.z;  // Yaw acceleration
        };
        
        this->setStateSpaceFunction(dynamics_callback);
        
        // Setup output function (identity mapping for full state feedback)
        auto output_callback = [](cvec<6>& y, const cvec<6>& x, const cvec<3>& u, const unsigned int& step) {
            y = x;  // Direct state measurement
        };
        
        this->setOutputFunction(output_callback);
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Dynamics callbacks configured successfully" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::configureObjectiveFunction()
    {
        // Multi-critic objective function implementation
        auto objective_callback = [this](const mat<PREDICTION_HORIZON + 1, STATE_DIM>& x,
                                        const mat<PREDICTION_HORIZON + 1, OUTPUT_DIM>& y,
                                        const mat<PREDICTION_HORIZON + 1, CONTROL_DIM>& u,
                                        const double& slack) -> double {
            
            double total_cost = 0.0;
            
            // Iterate through the prediction horizon
            for (int k = 0; k < PREDICTION_HORIZON + 1; k++) {
                cvec<STATE_DIM> state_k = x.row(k).transpose();
                
                if (objectives_.use_speed_heading_mode) {
                    // Speed tracking critic
                    double current_speed = calculateSpeed(state_k(VEL_U), state_k(VEL_V));
                    double speed_error = current_speed - objectives_.target_speed;
                    total_cost += objectives_.speed_weight * speed_error * speed_error;
                    
                    // Heading tracking critic with angle wrapping
                    double heading_error = angleDifference(objectives_.target_heading, state_k(HEADING));
                    total_cost += objectives_.heading_weight * heading_error * heading_error;
                } else {
                    // Full state tracking
                    cvec<STATE_DIM> state_error = state_k - objectives_.target_state;
                    
                    // Position error
                    double pos_error = state_error.segment<2>(POS_X).squaredNorm();
                    total_cost += objectives_.position_weight * pos_error;
                    
                    // Heading error with proper wrapping
                    double heading_error = angleDifference(objectives_.target_state(HEADING), state_k(HEADING));
                    total_cost += objectives_.heading_weight * heading_error * heading_error;
                    
                    // Velocity error  
                    double vel_error = state_error.segment<3>(VEL_U).squaredNorm();
                    total_cost += objectives_.speed_weight * vel_error;
                }
                
                // Control effort penalty (for all steps except last)
                if (k < PREDICTION_HORIZON) {
                    cvec<CONTROL_DIM> control_k = u.row(k).transpose();
                    total_cost += objectives_.control_effort_weight * control_k.squaredNorm();
                    
                    // Control rate penalty (except first step)
                    if (k > 0) {
                        cvec<CONTROL_DIM> control_prev = u.row(k-1).transpose();
                        cvec<CONTROL_DIM> control_rate = control_k - control_prev;
                        total_cost += objectives_.control_rate_weight * control_rate.squaredNorm();
                    }
                }
            }
            
            // Time-optimal penalty (encourage faster goal achievement)
            // This penalizes being far from the goal, encouraging quicker convergence
            if (objectives_.use_speed_heading_mode) {
                double final_speed = calculateSpeed(x(PREDICTION_HORIZON, VEL_U), x(PREDICTION_HORIZON, VEL_V));
                double final_speed_error = std::abs(final_speed - objectives_.target_speed);
                double final_heading_error = std::abs(angleDifference(objectives_.target_heading, x(PREDICTION_HORIZON, HEADING)));
                total_cost += objectives_.time_optimal_weight * (final_speed_error + final_heading_error);
            }
            
            // Slack variable penalty for constraint violation  
            total_cost += 1000.0 * slack * slack;
            
            return total_cost;
        };
        
        this->setObjectiveFunction(objective_callback);
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Multi-critic objective function configured" << std::endl;
    }

    template <int Tnx_obs>
    void VesselMPC<Tnx_obs>::configureConstraints()
    {
        // Set input bounds using the constraint limits
        this->setInputBounds(constraints_.u_min, constraints_.u_max, HorizonSlice::all());
        
        // Set state bounds
        this->setStateBounds(constraints_.x_min, constraints_.x_max, HorizonSlice::all());
        
        // Setup inequality constraints for actuator rate limits and custom constraints
        if constexpr (Tnx_obs > 0) {
            auto inequality_callback = [this](cvec<Tnx_obs>& ineq,
                                             const mat<PREDICTION_HORIZON + 1, STATE_DIM>& x,
                                             const mat<PREDICTION_HORIZON + 1, OUTPUT_DIM>& y,
                                             const mat<PREDICTION_HORIZON + 1, CONTROL_DIM>& u,
                                             const double& slack) {
                
                // This is a placeholder for obstacle avoidance or other custom constraints
                // The actual implementation would depend on the specific constraints needed
                
                // Example: Speed limit constraints (if Tnx_obs >= PREDICTION_HORIZON)
                int constraint_idx = 0;
                for (int k = 0; k < PREDICTION_HORIZON + 1 && constraint_idx < Tnx_obs; k++) {
                    double speed = calculateSpeed(x(k, VEL_U), x(k, VEL_V));
                    ineq(constraint_idx++) = speed - constraints_.max_speed; // speed <= max_speed
                }
                
                // Example: Turn rate constraints  
                for (int k = 0; k < PREDICTION_HORIZON + 1 && constraint_idx < Tnx_obs; k++) {
                    ineq(constraint_idx++) = std::abs(x(k, VEL_R)) - constraints_.max_turn_rate; // |r| <= max_turn_rate
                }
            };
            
            this->setIneqConFunction(inequality_callback);
        }
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Constraints configured successfully" << std::endl;
    }

    template <int Tnx_obs>
    double VesselMPC<Tnx_obs>::normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    template <int Tnx_obs>
    double VesselMPC<Tnx_obs>::angleDifference(double target, double current)
    {
        double diff = target - current;
        return normalizeAngle(diff);
    }

    template <int Tnx_obs>
    double VesselMPC<Tnx_obs>::calculateSpeed(double u, double v)
    {
        return std::sqrt(u * u + v * v);
    }

    // Explicit template instantiations for common use cases
    template class VesselMPC<0>;    // No obstacle constraints
    template class VesselMPC<10>;   // Up to 10 obstacle constraints
    template class VesselMPC<20>;   // Up to 20 obstacle constraints
    template class VesselMPC<50>;   // Up to 50 obstacle constraints

} // namespace mpc