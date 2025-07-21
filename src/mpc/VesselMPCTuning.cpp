/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */

#include <mpc/VesselMPCTuning.hpp>
#include <algorithm>
#include <sstream>
#include <cmath>

namespace mpc
{
    template <int Tnx_obs>
    void VesselMPCTuning<Tnx_obs>::applyVesselTypePreset(VesselType vessel_type, 
                                                         const OperatingConditions& conditions)
    {
        current_config_.vessel_type = vessel_type;
        current_config_.conditions = conditions;
        
        typename VesselMPC<Tnx_obs>::ControlObjectives objectives;
        typename VesselMPC<Tnx_obs>::ConstraintLimits constraints;
        
        // Base parameters by vessel type
        switch (vessel_type) {
            case VesselType::SMALL_RECREATIONAL:
                // Fast, responsive control for small boats
                objectives.speed_weight = 800.0;
                objectives.heading_weight = 600.0;
                objectives.position_weight = 200.0;
                objectives.control_effort_weight = 0.05;
                objectives.control_rate_weight = 0.5;
                objectives.time_optimal_weight = 0.02;
                
                constraints.u_min << -1500, -1500, -500;
                constraints.u_max << 3000, 3000, 500;
                constraints.du_max << 800, 800, 300;
                constraints.max_speed = 15.0;
                constraints.max_turn_rate = 1.0;
                break;
                
            case VesselType::MEDIUM_WORKBOAT:
                // Balanced performance for work vessels
                objectives.speed_weight = 1000.0;
                objectives.heading_weight = 800.0;  
                objectives.position_weight = 300.0;
                objectives.control_effort_weight = 0.1;
                objectives.control_rate_weight = 1.0;
                objectives.time_optimal_weight = 0.01;
                
                constraints.u_min << -3000, -3000, -1500;
                constraints.u_max << 8000, 8000, 1500;
                constraints.du_max << 2000, 2000, 800;
                constraints.max_speed = 12.0;
                constraints.max_turn_rate = 0.6;
                break;
                
            case VesselType::LARGE_COMMERCIAL:
                // Smooth, efficient control for large vessels
                objectives.speed_weight = 1200.0;
                objectives.heading_weight = 1000.0;
                objectives.position_weight = 400.0;
                objectives.control_effort_weight = 0.2;
                objectives.control_rate_weight = 2.0;
                objectives.time_optimal_weight = 0.005;
                
                constraints.u_min << -10000, -10000, -3000;
                constraints.u_max << 25000, 25000, 3000;
                constraints.du_max << 5000, 5000, 1500;
                constraints.max_speed = 8.0;
                constraints.max_turn_rate = 0.3;
                break;
                
            case VesselType::HIGH_SPEED_CRAFT:
                // Aggressive, high-performance control
                objectives.speed_weight = 600.0;
                objectives.heading_weight = 400.0;
                objectives.position_weight = 100.0;
                objectives.control_effort_weight = 0.02;
                objectives.control_rate_weight = 0.2;
                objectives.time_optimal_weight = 0.05;
                
                constraints.u_min << -5000, -5000, -2000;
                constraints.u_max << 15000, 15000, 2000;
                constraints.du_max << 3000, 3000, 1200;
                constraints.max_speed = 25.0;
                constraints.max_turn_rate = 1.5;
                break;
                
            case VesselType::HEAVY_CARGO:
                // Conservative, fuel-efficient control
                objectives.speed_weight = 1500.0;
                objectives.heading_weight = 1200.0;
                objectives.position_weight = 600.0;
                objectives.control_effort_weight = 0.5;
                objectives.control_rate_weight = 5.0;
                objectives.time_optimal_weight = 0.001;
                
                constraints.u_min << -15000, -15000, -5000;
                constraints.u_max << 40000, 40000, 5000;
                constraints.du_max << 3000, 3000, 1000;
                constraints.max_speed = 6.0;
                constraints.max_turn_rate = 0.2;
                break;
                
            case VesselType::CUSTOM:
                // Keep existing parameters
                objectives = controller_.getObjectives();
                constraints = controller_.getConstraints();
                break;
        }
        
        // Adjust for operating conditions
        adjustForConditions(conditions, objectives, constraints);
        
        // Apply to controller
        controller_.setObjectives(objectives);
        controller_.setConstraints(constraints);
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Applied preset for " << vessel_type_names[vessel_type] << " vessel" << std::endl;
    }

    template <int Tnx_obs>
    typename VesselMPCTuning<Tnx_obs>::PerformanceMetrics VesselMPCTuning<Tnx_obs>::autoTune(
        const TuningConfig& config,
        double vessel_mass,
        double vessel_length, 
        double max_speed)
    {
        current_config_ = config;
        
        // Calculate base parameters from vessel characteristics
        auto objectives = calculateBaseObjectives(config.vessel_type, vessel_mass, vessel_length, max_speed);
        auto constraints = calculateBaseConstraints(config.vessel_type, vessel_mass, vessel_length, max_speed);
        
        // Adjust based on tuning priorities
        if (config.speed_priority > 0.5) {
            // Prioritize fast response
            objectives.time_optimal_weight *= 2.0;
            objectives.control_rate_weight *= 0.8;
        }
        
        if (config.accuracy_priority > 0.4) {
            // Prioritize tracking accuracy
            objectives.speed_weight *= 1.5;
            objectives.heading_weight *= 1.5;
        }
        
        if (config.efficiency_priority > 0.3) {
            // Prioritize fuel efficiency
            objectives.control_effort_weight *= 2.0;
            objectives.control_rate_weight *= 1.5;
        }
        
        if (config.comfort_priority > 0.2) {
            // Prioritize passenger comfort
            objectives.control_rate_weight *= 3.0;
            constraints.du_max *= 0.7; // Reduce rate limits
        }
        
        // Apply safety margins
        constraints.u_max *= (1.0 - config.actuator_margin);
        constraints.u_min *= (1.0 - config.actuator_margin);
        
        // Adjust for operating conditions
        adjustForConditions(config.conditions, objectives, constraints);
        
        // Apply to controller
        controller_.setObjectives(objectives);
        controller_.setConstraints(constraints);
        
        // Configure solver parameters
        auto solver_params = getOptimizedSolverParameters(config.real_time_mode);
        controller_.setOptimizerParameters(solver_params);
        
        // Generate performance estimate
        PerformanceMetrics estimated_performance;
        
        // Estimate settling time based on vessel characteristics
        double inertia_factor = vessel_mass / 10000.0; // Normalized to 10-ton vessel
        double length_factor = vessel_length / 20.0;   // Normalized to 20m vessel
        
        estimated_performance.settling_time = 10.0 * inertia_factor * length_factor;
        estimated_performance.overshoot_percentage = std::max(5.0, 15.0 - config.accuracy_priority * 10.0);
        estimated_performance.steady_state_error = 0.1 / (1.0 + config.accuracy_priority * 4.0);
        estimated_performance.fuel_efficiency = config.efficiency_priority;
        estimated_performance.comfort_index = config.comfort_priority;
        estimated_performance.stability_maintained = true;
        
        last_metrics_ = estimated_performance;
        
        Logger::instance().log(Logger::LogType::INFO) 
            << "Auto-tuning completed for " << vessel_mass << "kg, " << vessel_length 
            << "m vessel with " << max_speed << "m/s max speed" << std::endl;
        
        return estimated_performance;
    }

    template <int Tnx_obs>
    NLParameters VesselMPCTuning<Tnx_obs>::getOptimizedSolverParameters(bool real_time_mode) const
    {
        NLParameters params;
        
        if (real_time_mode) {
            // Real-time optimized parameters - favor speed over accuracy
            params.maximum_iteration = 100;
            params.relative_ftol = 1e-6;
            params.relative_xtol = 1e-6;  
            params.absolute_ftol = 1e-6;
            params.absolute_xtol = 1e-6;
            params.time_limit = 1.5; // Strict time limit for real-time
        } else {
            // Simulation optimized parameters - favor accuracy
            params.maximum_iteration = 200;
            params.relative_ftol = 1e-9;
            params.relative_xtol = 1e-9;
            params.absolute_ftol = 1e-9;
            params.absolute_xtol = 1e-9;
            params.time_limit = 5.0; // More relaxed for simulation
        }
        
        // Adjust based on vessel type and conditions
        switch (current_config_.vessel_type) {
            case VesselType::HIGH_SPEED_CRAFT:
                params.time_limit *= 0.8; // Faster response needed
                break;
            case VesselType::HEAVY_CARGO:
                params.maximum_iteration *= 1.5; // Allow more iterations for complex dynamics
                break;
            default:
                break;
        }
        
        // Adjust for sea conditions
        if (current_config_.conditions.sea_state > 4) {
            params.relative_ftol *= 10; // Relax convergence in rough seas
            params.relative_xtol *= 10;
        }
        
        params.hard_constraints = true;  // Always enforce safety constraints
        params.enable_warm_start = true; // Always use warm start for performance
        
        return params;
    }

    template <int Tnx_obs>
    std::vector<std::string> VesselMPCTuning<Tnx_obs>::validateAndRecommend() const
    {
        std::vector<std::string> recommendations;
        
        auto objectives = controller_.getObjectives();
        auto constraints = controller_.getConstraints();
        
        // Check weight ratios
        double total_tracking_weight = objectives.speed_weight + objectives.heading_weight + objectives.position_weight;
        double control_weight_ratio = objectives.control_effort_weight / total_tracking_weight;
        
        if (control_weight_ratio > 0.1) {
            recommendations.push_back("Control effort weight may be too high - could cause sluggish response");
        }
        
        if (control_weight_ratio < 0.001) {
            recommendations.push_back("Control effort weight may be too low - could cause excessive actuator usage");
        }
        
        // Check constraint feasibility
        double thrust_range = (constraints.u_max - constraints.u_min).mean();
        double rate_limit_effectiveness = constraints.du_max.mean() / thrust_range;
        
        if (rate_limit_effectiveness > 0.5) {
            recommendations.push_back("Actuator rate limits may be too aggressive for the thrust range");
        }
        
        // Check speed and turn rate compatibility
        double speed_turn_ratio = constraints.max_speed / constraints.max_turn_rate;
        if (speed_turn_ratio > 50.0) {
            recommendations.push_back("Maximum turn rate may be too low for the maximum speed");
        }
        
        // Check for operating condition compatibility
        if (current_config_.conditions.sea_state > 6 && constraints.max_speed > 10.0) {
            recommendations.push_back("Maximum speed may be too high for the current sea state");
        }
        
        if (current_config_.conditions.restricted_waters && objectives.time_optimal_weight > 0.02) {
            recommendations.push_back("Time optimal weight should be reduced in restricted waters");
        }
        
        if (recommendations.empty()) {
            recommendations.push_back("Parameter configuration appears well-balanced");
        }
        
        return recommendations;
    }

    // Private helper methods
    template <int Tnx_obs>
    typename VesselMPC<Tnx_obs>::ControlObjectives VesselMPCTuning<Tnx_obs>::calculateBaseObjectives(
        VesselType vessel_type, double vessel_mass, double vessel_length, double max_speed)
    {
        typename VesselMPC<Tnx_obs>::ControlObjectives objectives;
        
        // Scale weights based on vessel characteristics
        double mass_factor = std::sqrt(vessel_mass / 10000.0);  // Normalized to 10-ton vessel
        double length_factor = vessel_length / 20.0;            // Normalized to 20m vessel
        double speed_factor = max_speed / 10.0;                 // Normalized to 10 m/s
        
        // Base weights (will be adjusted by vessel type preset)
        objectives.speed_weight = 1000.0 * mass_factor;
        objectives.heading_weight = 800.0 * length_factor;
        objectives.position_weight = 200.0;
        objectives.control_effort_weight = 0.1 / mass_factor;
        objectives.control_rate_weight = 1.0 * length_factor;
        objectives.time_optimal_weight = 0.01 * speed_factor;
        
        return objectives;
    }

    template <int Tnx_obs>
    typename VesselMPC<Tnx_obs>::ConstraintLimits VesselMPCTuning<Tnx_obs>::calculateBaseConstraints(
        VesselType vessel_type, double vessel_mass, double vessel_length, double max_speed)
    {
        typename VesselMPC<Tnx_obs>::ConstraintLimits constraints;
        
        // Scale constraints based on vessel characteristics
        double mass_factor = vessel_mass / 10000.0;
        double thrust_estimate = mass_factor * 5000.0;  // Rough thrust scaling
        
        constraints.u_min << -thrust_estimate * 0.5, -thrust_estimate * 0.5, -thrust_estimate * 0.3;
        constraints.u_max << thrust_estimate, thrust_estimate, thrust_estimate * 0.3;
        constraints.du_max << thrust_estimate * 0.3, thrust_estimate * 0.3, thrust_estimate * 0.2;
        
        constraints.max_speed = max_speed * 1.1; // Allow some margin above design speed
        constraints.max_turn_rate = std::max(0.1, 2.0 / vessel_length); // Larger vessels turn slower
        
        return constraints;
    }

    template <int Tnx_obs>
    void VesselMPCTuning<Tnx_obs>::adjustForConditions(const OperatingConditions& conditions,
                                                       typename VesselMPC<Tnx_obs>::ControlObjectives& objectives,
                                                       typename VesselMPC<Tnx_obs>::ConstraintLimits& constraints)
    {
        // Adjust for sea state
        if (conditions.sea_state > 4) {
            // Rough seas - reduce aggressive control
            objectives.control_rate_weight *= (1.0 + conditions.sea_state * 0.2);
            constraints.du_max *= (1.0 - conditions.sea_state * 0.05);
            constraints.max_speed *= (1.0 - conditions.sea_state * 0.1);
        }
        
        // Adjust for wind
        if (conditions.wind_speed > 10.0) {
            // Strong wind - need more control authority
            objectives.heading_weight *= (1.0 + conditions.wind_speed * 0.01);
            constraints.max_turn_rate *= (1.0 + conditions.wind_speed * 0.01);
        }
        
        // Adjust for restricted waters
        if (conditions.restricted_waters) {
            objectives.position_weight *= 2.0;     // Higher position accuracy
            objectives.control_rate_weight *= 2.0; // Smoother control
            constraints.max_speed *= 0.7;          // Reduce speed limits
            constraints.du_max *= 0.8;             // Reduce rate limits
        }
        
        // Adjust for dynamic positioning mode
        if (conditions.dynamic_positioning) {
            objectives.position_weight *= 5.0;     // Very high position accuracy
            objectives.speed_weight *= 0.5;        // Less emphasis on speed
            objectives.time_optimal_weight *= 0.1; // Station keeping, not transit
        }
        
        // Adjust for cargo loading
        if (conditions.cargo_loading != 1.0) {
            double loading_factor = conditions.cargo_loading;
            objectives.control_effort_weight *= loading_factor; // Heavier vessels need more control effort consideration
            constraints.max_turn_rate *= (2.0 - loading_factor); // Heavier vessels turn slower
        }
    }

    // Explicit template instantiations
    template class VesselMPCTuning<0>;
    template class VesselMPCTuning<10>;
    template class VesselMPCTuning<20>;
    template class VesselMPCTuning<50>;

} // namespace mpc