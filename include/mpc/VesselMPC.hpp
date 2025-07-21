/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */
#pragma once

#include <mpc/NLMPC.hpp>
#include <mpc/Types.hpp>
#include <mpc/Utils.hpp>
#include <memory>
#include <cmath>
#include <functional>

// Forward declarations for vessel dynamics integration
namespace vessel_dynamics {
    class VesselDynamics;
}

namespace mpc
{
    /**
     * @brief Vessel Model Predictive Control for autonomous boat navigation
     * 
     * This class provides nonlinear MPC capabilities specifically designed for 
     * maritime vessel control with multiple actuators (engines and thrusters).
     * It integrates with the vessel_dynamics library to provide realistic
     * vessel behavior simulation and control.
     * 
     * System Configuration:
     * - States (6): [x, y, ψ, u, v, r] - position, heading, velocities  
     * - Controls (3): [left_engine, right_engine, bow_thruster] forces
     * - Prediction Horizon: 10 steps
     * - Control Horizon: 10 steps  
     * - Default timestep: 2.0 seconds (20s total horizon)
     * 
     * @tparam Tnx_obs Number of obstacle inequality constraints (default: 0)
     */
    template <int Tnx_obs = 0>
    class VesselMPC : public NLMPC<6, 3, 6, 10, 10, Tnx_obs, 0>
    {
    public:
        // Vessel state dimensions
        static constexpr int STATE_DIM = 6;     // [x, y, psi, u, v, r]
        static constexpr int CONTROL_DIM = 3;   // [left_engine, right_engine, bow_thruster]
        static constexpr int OUTPUT_DIM = 6;    // Same as states for full observability
        static constexpr int PREDICTION_HORIZON = 10;
        static constexpr int CONTROL_HORIZON = 10;
        
        // State indices for clarity
        enum StateIndex {
            POS_X = 0,   // Position x (m)
            POS_Y = 1,   // Position y (m)
            HEADING = 2, // Heading angle ψ (rad)
            VEL_U = 3,   // Surge velocity u (m/s)
            VEL_V = 4,   // Sway velocity v (m/s) 
            VEL_R = 5    // Yaw rate r (rad/s)
        };
        
        // Control indices
        enum ControlIndex {
            LEFT_ENGINE = 0,    // Left engine force (N)
            RIGHT_ENGINE = 1,   // Right engine force (N)
            BOW_THRUSTER = 2    // Bow thruster force (N)
        };

        /**
         * @brief Vessel control objectives for multi-critic optimization
         */
        struct ControlObjectives {
            double target_speed = 0.0;        // Target speed (m/s)
            double target_heading = 0.0;      // Target heading (rad)
            cvec<6> target_state;              // Full target state vector
            bool use_speed_heading_mode = true; // Use speed/heading vs full state
            
            // Objective weights
            double speed_weight = 1000.0;      // Speed tracking weight
            double heading_weight = 500.0;     // Heading tracking weight  
            double position_weight = 100.0;    // Position tracking weight
            double control_effort_weight = 0.1; // Control effort penalty
            double control_rate_weight = 1.0;  // Control rate penalty
            double time_optimal_weight = 0.01;  // Time optimality weight
            
            ControlObjectives() {
                target_state.setZero();
            }
        };

        /**
         * @brief Vessel actuator and safety constraints
         */
        struct ConstraintLimits {
            // Actuator force limits (N)
            cvec<3> u_min;
            cvec<3> u_max;
            
            // Actuator rate limits (N/s)  
            cvec<3> du_max;
            
            // State bounds
            cvec<6> x_min;
            cvec<6> x_max;
            
            // Safety parameters
            double max_speed = 10.0;           // Maximum allowed speed (m/s)
            double max_turn_rate = 0.5;        // Maximum turn rate (rad/s)
            
            ConstraintLimits() {
                // Default actuator limits - to be set by user
                u_min << -5000, -5000, -2000;  // Negative thrust capability
                u_max << 10000, 10000, 2000;   // Max thrust capability
                du_max << 2000, 2000, 1000;    // Rate limits
                
                // Default state bounds (generous)
                x_min << -1e6, -1e6, -M_PI, -max_speed, -max_speed, -max_turn_rate;
                x_max << 1e6, 1e6, M_PI, max_speed, max_speed, max_turn_rate;
            }
        };

    public:
        /**
         * @brief Constructor - initializes the vessel MPC controller
         */
        VesselMPC() : NLMPC<6, 3, 6, 10, 10, Tnx_obs, 0>()
        {
            // Default timestep for marine applications (2 seconds)
            this->setDiscretizationSamplingTime(2.0);
            
            // Initialize default objectives and constraints
            objectives_ = ControlObjectives();
            constraints_ = ConstraintLimits();
            
            // Setup default solver parameters for marine applications
            configureDefaultSolverParameters();
            
            // Flag to indicate if dynamics have been set
            dynamics_configured_ = false;
            last_control_input_.setZero();
        }

        /**
         * @brief Set the vessel dynamics simulator
         * 
         * @param dynamics Shared pointer to vessel dynamics simulator
         */
        void setVesselDynamics(std::shared_ptr<vessel_dynamics::VesselDynamics> dynamics);

        /**
         * @brief Set control objectives for speed and heading tracking
         * 
         * @param target_speed Desired vessel speed (m/s)
         * @param target_heading Desired heading angle (rad)
         */
        void setSpeedHeadingGoals(double target_speed, double target_heading);

        /**
         * @brief Set full state tracking objectives
         * 
         * @param target_state Desired 6-DOF state vector
         */
        void setStateGoals(const cvec<6>& target_state);

        /**
         * @brief Configure objective weights for multi-critic optimization
         * 
         * @param objectives Structure containing all objective parameters
         */
        void setObjectives(const ControlObjectives& objectives);

        /**
         * @brief Set actuator and safety constraint limits
         * 
         * @param constraints Structure containing all constraint parameters
         */
        void setConstraints(const ConstraintLimits& constraints);

        /**
         * @brief Get current control objectives
         * 
         * @return const ControlObjectives& Current objectives configuration
         */
        const ControlObjectives& getObjectives() const { return objectives_; }

        /**
         * @brief Get current constraint configuration  
         * 
         * @return const ConstraintLimits& Current constraints configuration
         */
        const ConstraintLimits& getConstraints() const { return constraints_; }

        /**
         * @brief Optimize and get control command for current vessel state
         * 
         * @param current_state Current 6-DOF vessel state
         * @param warm_start Previous control input for warm start (optional)
         * @return Result<3> Optimization result with optimal control command
         */
        Result<3> optimizeControl(const cvec<6>& current_state, 
                                const cvec<3>& warm_start = cvec<3>::Zero());

        /**
         * @brief Check if the vessel dynamics have been properly configured
         * 
         * @return true if dynamics are set and ready
         */
        bool isDynamicsConfigured() const { return dynamics_configured_; }

    private:
        // Internal state
        std::shared_ptr<vessel_dynamics::VesselDynamics> vessel_dynamics_;
        ControlObjectives objectives_;
        ConstraintLimits constraints_;
        bool dynamics_configured_;
        cvec<3> last_control_input_;

        /**
         * @brief Configure default solver parameters optimized for marine applications
         */
        void configureDefaultSolverParameters();

        /**
         * @brief Setup the nonlinear dynamics function using vessel dynamics
         */
        void configureDynamicsCallbacks();

        /**
         * @brief Setup the multi-critic objective function
         */
        void configureObjectiveFunction();

        /**
         * @brief Setup actuator and safety constraints  
         */
        void configureConstraints();

        /**
         * @brief Normalize angle difference to [-π, π] range
         * 
         * @param angle Input angle (rad)
         * @return double Normalized angle
         */
        static double normalizeAngle(double angle);

        /**
         * @brief Calculate angular difference with proper wrapping
         * 
         * @param target Target angle (rad)
         * @param current Current angle (rad) 
         * @return double Angular error in [-π, π]
         */
        static double angleDifference(double target, double current);

        /**
         * @brief Extract speed from velocity components
         * 
         * @param u Surge velocity (m/s)
         * @param v Sway velocity (m/s)
         * @return double Total speed (m/s)
         */
        static double calculateSpeed(double u, double v);
    };

} // namespace mpc