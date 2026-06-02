#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <ruckig/calculator.hpp>
#include <ruckig/error.hpp>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/result.hpp>
#include <ruckig/trajectory.hpp>
#include <ruckig/utils.hpp>


namespace ruckig {

enum class TrackigMode {
    Optimized,
    Fast,
};


//! The kinematic target state for Trackig.
template<size_t DOFs = 0, template<class, size_t> class CustomVector = StandardVector>
class TargetState {
    template<class T> using Vector = CustomVector<T, DOFs>;

    void resize(size_t dofs) {
        if constexpr (DOFs == 0) {
            position.resize(dofs);
            velocity.resize(dofs);
            acceleration.resize(dofs);
        }
    }

    void initialize() {
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            position[dof] = 0.0;
            velocity[dof] = 0.0;
            acceleration[dof] = 0.0;
        }
    }

public:
    size_t degrees_of_freedom;

    Vector<double> position, velocity, acceleration;

    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit TargetState(): degrees_of_freedom(DOFs) {
        initialize();
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit TargetState(size_t dofs): degrees_of_freedom(dofs) {
        resize(dofs);
        initialize();
    }

    void interpolate(const TargetState<DOFs, CustomVector>& other, double alpha) {
        alpha = std::clamp(alpha, 0.0, 1.0);
        const double beta = 1.0 - alpha;
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            position[dof] = beta * position[dof] + alpha * other.position[dof];
            velocity[dof] = beta * velocity[dof] + alpha * other.velocity[dof];
            acceleration[dof] = beta * acceleration[dof] + alpha * other.acceleration[dof];
        }
    }

    bool operator!=(const TargetState<DOFs, CustomVector>& other) const {
        return !(
            degrees_of_freedom == other.degrees_of_freedom
            && position == other.position
            && velocity == other.velocity
            && acceleration == other.acceleration
        );
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "\ntarget.position = [" << join(position, degrees_of_freedom) << "]\n";
        ss << "target.velocity = [" << join(velocity, degrees_of_freedom) << "]\n";
        ss << "target.acceleration = [" << join(acceleration, degrees_of_freedom) << "]\n";
        return ss.str();
    }
};


//! Main interface for the Trackig algorithm.
template<size_t DOFs = 0, template<class, size_t> class CustomVector = StandardVector>
class Trackig {
    template<class T> using Vector = CustomVector<T, DOFs>;

    InputParameter<DOFs, CustomVector> current_input;
    bool current_input_initialized {false};
    Vector<double> target_velocity_limit, target_acceleration_limit;
    bool target_velocity_limit_enabled {false};
    bool target_acceleration_limit_enabled {false};

    TargetState<DOFs, CustomVector> make_target_state() const {
        if constexpr (DOFs >= 1) {
            return TargetState<DOFs, CustomVector>();
        } else {
            return TargetState<DOFs, CustomVector>(degrees_of_freedom);
        }
    }

    OutputParameter<DOFs, CustomVector> make_output_parameter() const {
        if constexpr (DOFs >= 1) {
            return OutputParameter<DOFs, CustomVector>();
        } else {
            return OutputParameter<DOFs, CustomVector>(degrees_of_freedom);
        }
    }

    Trajectory<DOFs, CustomVector> make_trajectory() const {
        if constexpr (DOFs >= 1) {
            return Trajectory<DOFs, CustomVector>();
        } else {
            return Trajectory<DOFs, CustomVector>(degrees_of_freedom);
        }
    }

    void initialize_target_limit_vectors() {
        if constexpr (DOFs == 0) {
            target_velocity_limit.resize(degrees_of_freedom);
            target_acceleration_limit.resize(degrees_of_freedom);
        }

        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            target_velocity_limit[dof] = std::numeric_limits<double>::infinity();
            target_acceleration_limit[dof] = std::numeric_limits<double>::infinity();
        }
    }

    void validate_limit_vector(const Vector<double>& limits, const std::string& name) const {
        if constexpr (DOFs == 0) {
            if (limits.size() != degrees_of_freedom) {
                throw RuckigError(name + " size does not match degrees of freedom.");
            }
        }

        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            if (std::isnan(limits[dof]) || limits[dof] < 0.0) {
                throw RuckigError(name + " limit " + std::to_string(limits[dof]) +
                                  " of DoF " + std::to_string(dof) +
                                  " should be larger than or equal to zero.");
            }
        }
    }

    TargetState<DOFs, CustomVector> apply_target_limits(
        const TargetState<DOFs, CustomVector>& target_state,
        const InputParameter<DOFs, CustomVector>& input) const {
        TargetState<DOFs, CustomVector> limited_target = target_state;

        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            const double velocity_max = target_velocity_limit_enabled ?
                target_velocity_limit[dof] : input.max_velocity[dof];
            const double velocity_min = target_velocity_limit_enabled ?
                -target_velocity_limit[dof] :
                (input.min_velocity ? input.min_velocity.value()[dof] : -input.max_velocity[dof]);

            limited_target.velocity[dof] = std::clamp(
                limited_target.velocity[dof], velocity_min, velocity_max);

            const double acceleration_max = target_acceleration_limit_enabled ?
                target_acceleration_limit[dof] : input.max_acceleration[dof];
            const double acceleration_min = target_acceleration_limit_enabled ?
                -target_acceleration_limit[dof] :
                (input.min_acceleration ? input.min_acceleration.value()[dof] : -input.max_acceleration[dof]);

            limited_target.acceleration[dof] = std::clamp(
                limited_target.acceleration[dof], acceleration_min, acceleration_max);
        }

        return limited_target;
    }

    TargetState<DOFs, CustomVector> predict_constant_acceleration(
        double time, const TargetState<DOFs, CustomVector>& target_state,
        bool& valid) const {
        valid = true;
        TargetState<DOFs, CustomVector> predicted = make_target_state();
        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            predicted.position[dof] = target_state.position[dof] +
                time * target_state.velocity[dof] +
                0.5 * time * time * target_state.acceleration[dof];
            predicted.velocity[dof] = target_state.velocity[dof] +
                time * target_state.acceleration[dof];
            predicted.acceleration[dof] = target_state.acceleration[dof];
        }
        return predicted;
    }

    InputParameter<DOFs, CustomVector> target_to_input(
        const TargetState<DOFs, CustomVector>& target_state,
        const InputParameter<DOFs, CustomVector>& input,
        double prediction_time,
        double minimum_duration) const {
        if constexpr (DOFs == 0) {
            if (target_state.degrees_of_freedom != degrees_of_freedom ||
                input.degrees_of_freedom != degrees_of_freedom) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
        }

        InputParameter<DOFs, CustomVector> tracking_input = input;

        bool valid_prediction {true};
        TargetState<DOFs, CustomVector> predicted_target =
            prediction_model(prediction_time, target_state, valid_prediction);
        if (!valid_prediction) {
            predicted_target = target_state;
        }
        predicted_target = apply_target_limits(predicted_target, tracking_input);

        for (size_t dof = 0; dof < degrees_of_freedom; ++dof) {
            tracking_input.target_position[dof] = predicted_target.position[dof];
            tracking_input.target_velocity[dof] = predicted_target.velocity[dof];
            tracking_input.target_acceleration[dof] = predicted_target.acceleration[dof];
        }

        if (minimum_duration > 0.0) {
            if (tracking_input.minimum_duration) {
                tracking_input.minimum_duration = std::max(
                    tracking_input.minimum_duration.value(), minimum_duration);
            } else {
                tracking_input.minimum_duration = minimum_duration;
            }
        }

        return tracking_input;
    }

    template<bool throw_validation_error = false>
    bool validate_input(const InputParameter<DOFs, CustomVector>& input) const {
        if (!input.template validate<throw_validation_error>(false, true)) {
            return false;
        }

        if (delta_time <= 0.0 && input.duration_discretization != DurationDiscretization::Continuous) {
            if constexpr (throw_validation_error) {
                throw RuckigError("delta time (control rate) parameter " + std::to_string(delta_time) + " should be larger than zero.");
            }
            return false;
        }

        return true;
    }

    bool calculate_candidate(const TargetState<DOFs, CustomVector>& target_state,
                             const InputParameter<DOFs, CustomVector>& input,
                             double horizon,
                             double prediction_time,
                             InputParameter<DOFs, CustomVector>& candidate_input,
                             Trajectory<DOFs, CustomVector>& candidate_trajectory,
                             Result& result,
                             bool& was_interrupted) {
        candidate_input = target_to_input(target_state, input, prediction_time, horizon);
        was_interrupted = false;

        if (!validate_input<false>(candidate_input)) {
            result = Result::ErrorInvalidInput;
            return false;
        }

        result = calculator.template calculate<false>(
            candidate_input, candidate_trajectory, delta_time, was_interrupted);
        ++last_iterations_counter;

        return result == Result::Working ||
               result == Result::Finished ||
               result == Result::ErrorPositionalLimits;
    }

    bool is_time_consistent(const Trajectory<DOFs, CustomVector>& trajectory,
                            double horizon) const {
        const double tolerance = 1e-9 + 1e-6 * std::max(1.0, horizon);
        return trajectory.get_duration() <= horizon + tolerance;
    }

    Result calculate_tracking_trajectory(
        const TargetState<DOFs, CustomVector>& target_state,
        const InputParameter<DOFs, CustomVector>& input,
        InputParameter<DOFs, CustomVector>& best_input,
        Trajectory<DOFs, CustomVector>& best_trajectory,
        bool& best_was_interrupted) {
        last_iterations_counter = 0;

        const double bounded_reactiveness = std::clamp(reactiveness, 0.0, 1.0);
        const size_t iteration_limit = std::max<size_t>(1, max_iterations);

        if (bounded_reactiveness <= 0.0) {
            Result result {Result::Working};
            calculate_candidate(target_state, input, 0.0, 0.0, best_input,
                                best_trajectory, result, best_was_interrupted);
            return result;
        }

        InputParameter<DOFs, CustomVector> candidate_input = input;
        Trajectory<DOFs, CustomVector> candidate_trajectory = make_trajectory();
        bool candidate_was_interrupted {false};
        Result candidate_result {Result::Working};

        bool has_best {false};
        Result best_result {Result::Working};
        double fallback_gap {std::numeric_limits<double>::infinity()};
        bool has_fallback {false};
        InputParameter<DOFs, CustomVector> fallback_input = input;
        Trajectory<DOFs, CustomVector> fallback_trajectory = make_trajectory();
        bool fallback_was_interrupted {false};
        Result fallback_result {Result::Working};

        const auto remember_candidate = [&](double horizon, bool time_consistent) {
            if (time_consistent) {
                best_input = candidate_input;
                best_trajectory = candidate_trajectory;
                best_was_interrupted = candidate_was_interrupted;
                best_result = candidate_result;
                has_best = true;
                return;
            }

            const double gap = std::max(0.0, candidate_trajectory.get_duration() - horizon);
            if (!has_fallback || gap < fallback_gap) {
                fallback_gap = gap;
                fallback_input = candidate_input;
                fallback_trajectory = candidate_trajectory;
                fallback_was_interrupted = candidate_was_interrupted;
                fallback_result = candidate_result;
                has_fallback = true;
            }
        };

        double lower_horizon {0.0};
        double upper_horizon = std::max(delta_time, static_cast<double>(
            std::max<size_t>(1, look_ahead_cycles)) * delta_time);

        while (last_iterations_counter < iteration_limit) {
            const double prediction_time = bounded_reactiveness * upper_horizon;
            if (!calculate_candidate(target_state, input, upper_horizon,
                                     prediction_time, candidate_input,
                                     candidate_trajectory, candidate_result,
                                     candidate_was_interrupted)) {
                break;
            }

            const bool time_consistent = is_time_consistent(
                candidate_trajectory, upper_horizon);
            remember_candidate(upper_horizon, time_consistent);
            if (time_consistent) {
                break;
            }

            lower_horizon = upper_horizon;
            upper_horizon *= 2.0;
        }

        if (has_best && mode == TrackigMode::Optimized) {
            while (last_iterations_counter < iteration_limit) {
                const double horizon = 0.5 * (lower_horizon + upper_horizon);
                const double prediction_time = bounded_reactiveness * horizon;

                if (!calculate_candidate(target_state, input, horizon,
                                         prediction_time, candidate_input,
                                         candidate_trajectory, candidate_result,
                                         candidate_was_interrupted)) {
                    break;
                }

                const bool time_consistent = is_time_consistent(
                    candidate_trajectory, horizon);
                remember_candidate(horizon, time_consistent);

                if (time_consistent) {
                    upper_horizon = horizon;
                } else {
                    lower_horizon = horizon;
                }
            }
        }

        if (has_best) {
            return best_result;
        }

        if (has_fallback) {
            best_input = fallback_input;
            best_trajectory = fallback_trajectory;
            best_was_interrupted = fallback_was_interrupted;
            return fallback_result;
        }

        return Result::ErrorInvalidInput;
    }

public:
    //! Calculator for new trajectories.
    Calculator<DOFs, CustomVector> calculator;

    //! Degrees of freedom.
    const size_t degrees_of_freedom;

    //! Time step between updates (cycle time) in [s].
    const double delta_time;

    //! Different modes of the Trackig algorithm.
    TrackigMode mode {TrackigMode::Optimized};

    //! Weight for the reactiveness of the resulting trajectory, in [0.0, 1.0].
    double reactiveness {1.0};

    //! Number of cycles to look ahead for target prediction.
    size_t look_ahead_cycles {1};

    //! Maximum number of iterations for the optimization.
    size_t max_iterations {8};

    //! Number of iterations used during the last update.
    size_t last_iterations_counter {0};

    //! Model for predicting the target state into the future.
    std::function<TargetState<DOFs, CustomVector>(
        double, const TargetState<DOFs, CustomVector>&, bool&)> prediction_model;

    template<size_t D = DOFs, typename std::enable_if<(D >= 1), int>::type = 0>
    explicit Trackig(double delta_time)
        : degrees_of_freedom(DOFs), delta_time(delta_time) {
        initialize_target_limit_vectors();
        prediction_model = [this](
            double time, const TargetState<DOFs, CustomVector>& target_state,
            bool& valid) {
            return predict_constant_acceleration(time, target_state, valid);
        };
    }

    template<size_t D = DOFs, typename std::enable_if<(D == 0), int>::type = 0>
    explicit Trackig(size_t dofs, double delta_time)
        : current_input(InputParameter<DOFs, CustomVector>(dofs)),
          calculator(Calculator<DOFs, CustomVector>(dofs)),
          degrees_of_freedom(dofs),
          delta_time(delta_time) {
        initialize_target_limit_vectors();
        prediction_model = [this](
            double time, const TargetState<DOFs, CustomVector>& target_state,
            bool& valid) {
            return predict_constant_acceleration(time, target_state, valid);
        };
    }

    //! Reset the instance (e.g. to force a new calculation in the next update).
    void reset() {
        current_input_initialized = false;
    }

    //! Sets symmetric target velocity limits used before each tracking update.
    void setTargetVelocityLimits(const Vector<double>& max_velocity) {
        validate_limit_vector(max_velocity, "target velocity");
        target_velocity_limit = max_velocity;
        target_velocity_limit_enabled = true;
        reset();
    }

    //! Sets symmetric target acceleration limits used before each tracking update.
    void setTargetAccelerationLimits(const Vector<double>& max_acceleration) {
        validate_limit_vector(max_acceleration, "target acceleration");
        target_acceleration_limit = max_acceleration;
        target_acceleration_limit_enabled = true;
        reset();
    }

    //! Sets symmetric target velocity and acceleration limits.
    void setTargetLimits(const Vector<double>& max_velocity,
                         const Vector<double>& max_acceleration) {
        validate_limit_vector(max_velocity, "target velocity");
        validate_limit_vector(max_acceleration, "target acceleration");
        target_velocity_limit = max_velocity;
        target_acceleration_limit = max_acceleration;
        target_velocity_limit_enabled = true;
        target_acceleration_limit_enabled = true;
        reset();
    }

    //! Uses the input max velocity limits for tracking targets.
    void disableTargetVelocityLimits() {
        target_velocity_limit_enabled = false;
        reset();
    }

    //! Uses the input max acceleration limits for tracking targets.
    void disableTargetAccelerationLimits() {
        target_acceleration_limit_enabled = false;
        reset();
    }

    //! Uses the input max velocity and acceleration limits for tracking targets.
    void disableTargetLimits() {
        target_velocity_limit_enabled = false;
        target_acceleration_limit_enabled = false;
        reset();
    }

    bool getTargetVelocityLimitsEnabled() const {
        return target_velocity_limit_enabled;
    }

    bool getTargetAccelerationLimitsEnabled() const {
        return target_acceleration_limit_enabled;
    }

    const Vector<double>& getTargetVelocityLimits() const {
        return target_velocity_limit;
    }

    const Vector<double>& getTargetAccelerationLimits() const {
        return target_acceleration_limit;
    }

    //! Follow the given target state online.
    Result update(const TargetState<DOFs, CustomVector>& target_state,
                  InputParameter<DOFs, CustomVector>& input,
                  OutputParameter<DOFs, CustomVector>& output) {
        const auto start = std::chrono::steady_clock::now();

        if constexpr (DOFs == 0) {
            if (degrees_of_freedom != output.degrees_of_freedom) {
                throw RuckigError("mismatch in degrees of freedom (vector size).");
            }
        }

        output.new_calculation = false;
        InputParameter<DOFs, CustomVector> tracking_input = input;
        Trajectory<DOFs, CustomVector> tracking_trajectory = make_trajectory();
        bool was_calculation_interrupted {false};

        Result result {Result::Working};
        result = calculate_tracking_trajectory(target_state, input,
                                               tracking_input,
                                               tracking_trajectory,
                                               was_calculation_interrupted);

        if (result != Result::Working &&
            result != Result::Finished &&
            result != Result::ErrorPositionalLimits) {
            return result;
        }

        if (tracking_input != current_input || !current_input_initialized) {
            output.trajectory = tracking_trajectory;
            output.was_calculation_interrupted = was_calculation_interrupted;
            current_input = tracking_input;
            current_input_initialized = true;
            output.time = 0.0;
            output.new_calculation = true;
        }

        const size_t old_section = output.new_section;
        output.time += delta_time;
        output.trajectory.at_time(output.time, output.new_position,
                                  output.new_velocity, output.new_acceleration,
                                  output.new_jerk, output.new_section);
        output.did_section_change = (output.new_section > old_section);

        const auto stop = std::chrono::steady_clock::now();
        output.calculation_duration =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                stop - start).count() / 1000.0;

        output.pass_to_input(current_input);

        if (output.time > output.trajectory.get_duration()) {
            return Result::Finished;
        }

        return result;
    }

    //! Follow the fully-known trajectory in an offline manner.
    std::vector<OutputParameter<DOFs, CustomVector>> calculate_trajectory(
        const std::vector<TargetState<DOFs, CustomVector>>& trajectory,
        InputParameter<DOFs, CustomVector>& input) {
        reset();

        std::vector<OutputParameter<DOFs, CustomVector>> output_states;
        output_states.reserve(trajectory.size());
        OutputParameter<DOFs, CustomVector> output = make_output_parameter();

        for (const auto& target_state : trajectory) {
            const Result result = update(target_state, input, output);
            if (result < Result::Working) {
                break;
            }

            output_states.push_back(output);
            output.pass_to_input(input);
        }

        return output_states;
    }
};

} // namespace ruckig
