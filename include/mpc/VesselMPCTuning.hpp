/*
 *   Copyright (c) 2023-2025 Nicola Piccinelli
 *   All rights reserved.
 */
#pragma once

#include <mpc/VesselMPC.hpp>
#include <string>
#include <map>
#include <functional>

namespace mpc
{
    /**
     * @brief Parameter tuning and configuration utilities for VesselMPC
     * 
     * This class provides comprehensive parameter tuning capabilities for 
     * VesselMPC controllers, including:
     * - Pre-configured parameter sets for common vessel types
     * - Adaptive parameter tuning based on vessel characteristics
     * - Performance analysis and optimization recommendations
     * - Real-time parameter adjustment capabilities
     */
    template <int Tnx_obs = 0>
    class VesselMPCTuning
    {
    public:
        /**
         * @brief Vessel type classifications for parameter presets
         */
        enum class VesselType {
            SMALL_RECREATIONAL,    // <10m recreational boats
            MEDIUM_WORKBOAT,       // 10-30m commercial/research vessels  
            LARGE_COMMERCIAL,      // 30-100m commercial vessels
            HIGH_SPEED_CRAFT,      // Fast patrol/racing boats
            HEAVY_CARGO,           // Slow, heavy cargo vessels
            CUSTOM                 // User-defined parameters
        };

        /**
         * @brief Operating conditions that affect optimal parameters
         */
        struct OperatingConditions {
            double sea_state = 0;           // Sea state (0-9 scale)
            double wind_speed = 0;          // Wind speed (m/s)  
            double current_speed = 0;       // Current speed (m/s)
            double water_depth = 100;       // Water depth (m) - affects shallow water effects
            double cargo_loading = 1.0;     // Loading factor (0.5-1.5, 1.0 = design load)
            bool restricted_waters = false;  // Harbor/channel navigation
            bool dynamic_positioning = false; // DP mode vs transit mode
        };

        /**
         * @brief Performance metrics for tuning evaluation
         */
        struct PerformanceMetrics {
            double settling_time = 0;       // Time to reach 95% of target (s)
            double overshoot_percentage = 0; // Maximum overshoot (%)
            double steady_state_error = 0;  // Final tracking error
            double control_effort = 0;      // RMS control effort
            double fuel_efficiency = 0;     // Control effort per distance traveled
            double comfort_index = 0;       // Acceleration-based comfort metric
            bool stability_maintained = true; // System stability assessment
        };

        /**
         * @brief Tuning session configuration
         */
        struct TuningConfig {
            VesselType vessel_type = VesselType::MEDIUM_WORKBOAT;
            OperatingConditions conditions;
            
            // Tuning priorities (weights sum to 1.0)
            double speed_priority = 0.4;       // Fast response priority
            double accuracy_priority = 0.3;    // Tracking accuracy priority  
            double efficiency_priority = 0.2;  // Fuel efficiency priority
            double comfort_priority = 0.1;     // Passenger comfort priority
            
            // Safety margins
            double actuator_margin = 0.1;      // Reserve actuator capacity (10%)
            double stability_margin = 0.2;     // Stability margin factor
            
            bool adaptive_tuning = true;       // Enable online adaptation
            bool real_time_mode = false;       // Real-time vs simulation mode
        };

    public:
        /**
         * @brief Constructor
         * 
         * @param vessel_controller Reference to the VesselMPC to be tuned
         */
        VesselMPCTuning(VesselMPC<Tnx_obs>& vessel_controller) 
            : controller_(vessel_controller) {}

        /**
         * @brief Apply pre-configured parameter set based on vessel type
         * 
         * @param vessel_type Type of vessel for parameter selection
         * @param conditions Current operating conditions
         */
        void applyVesselTypePreset(VesselType vessel_type, 
                                  const OperatingConditions& conditions = OperatingConditions());

        /**
         * @brief Perform automatic parameter tuning based on vessel characteristics
         * 
         * @param config Tuning configuration and priorities
         * @param vessel_mass Vessel mass (kg)
         * @param vessel_length Vessel length overall (m)
         * @param max_speed Maximum design speed (m/s)
         * @return PerformanceMetrics Tuning results and performance assessment
         */
        PerformanceMetrics autoTune(const TuningConfig& config,
                                   double vessel_mass,
                                   double vessel_length, 
                                   double max_speed);

        /**
         * @brief Fine-tune parameters based on performance feedback
         * 
         * @param current_metrics Current performance metrics
         * @param target_metrics Desired performance targets
         * @return bool True if tuning successful, false if limits reached
         */
        bool refineTuning(const PerformanceMetrics& current_metrics,
                         const PerformanceMetrics& target_metrics);

        /**
         * @brief Adaptive parameter adjustment during operation
         * 
         * @param conditions Current operating conditions
         * @param performance_feedback Recent performance metrics
         */
        void adaptiveAdjustment(const OperatingConditions& conditions,
                              const PerformanceMetrics& performance_feedback);

        /**
         * @brief Get recommended solver parameters for current configuration
         * 
         * @param real_time_mode Whether controller is running in real-time
         * @return NLParameters Optimized solver parameters
         */
        NLParameters getOptimizedSolverParameters(bool real_time_mode = false) const;

        /**
         * @brief Validate current parameter set and provide recommendations
         * 
         * @return std::vector<std::string> List of tuning recommendations
         */
        std::vector<std::string> validateAndRecommend() const;

        /**
         * @brief Export current parameters to configuration string
         * 
         * @return std::string Serialized parameter configuration
         */
        std::string exportParameters() const;

        /**
         * @brief Import parameters from configuration string
         * 
         * @param config_string Serialized parameter configuration
         * @return bool True if import successful
         */
        bool importParameters(const std::string& config_string);

        /**
         * @brief Get current tuning configuration
         * 
         * @return const TuningConfig& Current configuration
         */
        const TuningConfig& getTuningConfig() const { return current_config_; }

    private:
        VesselMPC<Tnx_obs>& controller_;
        TuningConfig current_config_;
        PerformanceMetrics last_metrics_;

        /**
         * @brief Calculate base parameters for vessel type
         */
        typename VesselMPC<Tnx_obs>::ControlObjectives calculateBaseObjectives(
            VesselType vessel_type, double vessel_mass, double vessel_length, double max_speed);

        /**
         * @brief Calculate constraint limits for vessel type  
         */
        typename VesselMPC<Tnx_obs>::ConstraintLimits calculateBaseConstraints(
            VesselType vessel_type, double vessel_mass, double vessel_length, double max_speed);

        /**
         * @brief Adjust parameters for operating conditions
         */
        void adjustForConditions(const OperatingConditions& conditions,
                               typename VesselMPC<Tnx_obs>::ControlObjectives& objectives,
                               typename VesselMPC<Tnx_obs>::ConstraintLimits& constraints);

        /**
         * @brief Calculate performance metrics from simulation results
         */
        PerformanceMetrics calculatePerformanceMetrics(
            const std::vector<mpc::cvec<6>>& state_history,
            const std::vector<mpc::cvec<3>>& control_history,
            const std::vector<double>& time_history,
            const mpc::cvec<6>& target_state) const;

        /**
         * @brief Estimate settling time from response
         */
        double estimateSettlingTime(const std::vector<mpc::cvec<6>>& state_history,
                                   const std::vector<double>& time_history,
                                   const mpc::cvec<6>& target_state) const;

        /**
         * @brief Calculate overshoot percentage
         */
        double calculateOvershoot(const std::vector<mpc::cvec<6>>& state_history,
                                 const mpc::cvec<6>& target_state) const;

        /**
         * @brief Assess system stability from response
         */
        bool assessStability(const std::vector<mpc::cvec<6>>& state_history,
                           const std::vector<mpc::cvec<3>>& control_history) const;
    };

    /**
     * @brief Vessel type name mapping for display purposes
     */
    inline std::map<typename VesselMPCTuning<>::VesselType, std::string> vessel_type_names = {
        {VesselMPCTuning<>::VesselType::SMALL_RECREATIONAL, "Small Recreational"},
        {VesselMPCTuning<>::VesselType::MEDIUM_WORKBOAT, "Medium Workboat"},
        {VesselMPCTuning<>::VesselType::LARGE_COMMERCIAL, "Large Commercial"},
        {VesselMPCTuning<>::VesselType::HIGH_SPEED_CRAFT, "High Speed Craft"},
        {VesselMPCTuning<>::VesselType::HEAVY_CARGO, "Heavy Cargo"},
        {VesselMPCTuning<>::VesselType::CUSTOM, "Custom"}
    };

    /**
     * @brief Quick setup function for common vessel configurations
     * 
     * @tparam Tnx_obs Number of obstacle constraints
     * @param controller VesselMPC controller to configure
     * @param vessel_type Type of vessel
     * @param vessel_mass Vessel mass (kg)
     * @param vessel_length Vessel length (m)  
     * @param max_speed Maximum speed (m/s)
     * @param conditions Operating conditions
     * @return PerformanceMetrics Expected performance with these parameters
     */
    template <int Tnx_obs = 0>
    typename VesselMPCTuning<Tnx_obs>::PerformanceMetrics quickSetup(
        VesselMPC<Tnx_obs>& controller,
        typename VesselMPCTuning<Tnx_obs>::VesselType vessel_type,
        double vessel_mass,
        double vessel_length,
        double max_speed,
        const typename VesselMPCTuning<Tnx_obs>::OperatingConditions& conditions = {})
    {
        VesselMPCTuning<Tnx_obs> tuner(controller);
        
        // Apply vessel type preset
        tuner.applyVesselTypePreset(vessel_type, conditions);
        
        // Auto-tune for vessel characteristics
        typename VesselMPCTuning<Tnx_obs>::TuningConfig config;
        config.vessel_type = vessel_type;
        config.conditions = conditions;
        
        return tuner.autoTune(config, vessel_mass, vessel_length, max_speed);
    }

} // namespace mpc