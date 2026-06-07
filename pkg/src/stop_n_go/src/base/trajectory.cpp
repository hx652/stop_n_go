#include <stop_n_go/base/trajectory.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stop_n_go::base
{

namespace
{

RobotState sampleTrajectory(const Trajectory & trajectory, double sample_time)
{
  if (trajectory.empty()) {
    throw std::invalid_argument("Cannot sample an empty Trajectory");
  }

  if (sample_time < 0.0 || sample_time > trajectory.totalDuration()) {
    throw std::out_of_range("Trajectory sample time is out of range");
  }

  if (trajectory.size() == 1 || sample_time == 0.0) {
    return trajectory.waypoint(0);
  }

  if (sample_time == trajectory.totalDuration()) {
    return trajectory.waypoint(trajectory.size() - 1);
  }

  std::size_t segment_index = 0;
  double segment_start_time = 0.0;
  double segment_end_time = trajectory.durationFromPrevious(1);

  // Locate the segment that contains the requested time.
  while (segment_index + 1 < trajectory.size() - 1 && sample_time >= segment_end_time) {
    ++segment_index;
    segment_start_time = segment_end_time;
    segment_end_time += trajectory.durationFromPrevious(segment_index + 1);
  }

  const double segment_duration = segment_end_time - segment_start_time;
  if (segment_duration <= 0.0) {
    // Degenerate zero-duration segments collapse to the later waypoint.
    return trajectory.waypoint(segment_index + 1);
  }

  const double ratio = (sample_time - segment_start_time) / segment_duration;
  return interpolate(
    trajectory.waypoint(segment_index),
    trajectory.waypoint(segment_index + 1),
    ratio,
    trajectory.layout());
}

}  // namespace

Trajectory::Trajectory(StateLayout::ConstPtr layout)
: layout_(std::move(layout))
{
  validateLayout();
}

const StateLayout & Trajectory::layout() const
{
  return *layout_;
}

const StateLayout::ConstPtr & Trajectory::layoutPtr() const
{
  return layout_;
}

std::size_t Trajectory::size() const
{
  return waypoints_.size();
}

bool Trajectory::empty() const
{
  return waypoints_.empty();
}

const std::vector<RobotState> & Trajectory::waypoints() const
{
  return waypoints_;
}

const std::vector<double> & Trajectory::durationsFromPrevious() const
{
  return durations_from_previous_;
}

const RobotState & Trajectory::waypoint(std::size_t index) const
{
  return waypoints_.at(index);
}

RobotState & Trajectory::waypoint(std::size_t index)
{
  return waypoints_.at(index);
}

double Trajectory::durationFromPrevious(std::size_t index) const
{
  return durations_from_previous_.at(index);
}

double Trajectory::durationFromStart(std::size_t index) const
{
  if (index >= size()) {
    throw std::out_of_range("Trajectory waypoint index out of range");
  }

  double duration = 0.0;
  for (std::size_t waypoint_index = 0; waypoint_index <= index; ++waypoint_index) {
    duration += durations_from_previous_[waypoint_index];
  }

  return duration;
}

double Trajectory::totalDuration() const
{
  double duration = 0.0;
  for (double dt : durations_from_previous_) {
    duration += dt;
  }

  return duration;
}

bool Trajectory::isCompatible(const RobotState & state) const
{
  return layout_->isCompatible(state);
}

void Trajectory::clear()
{
  waypoints_.clear();
  durations_from_previous_.clear();
}

void Trajectory::reserve(std::size_t size)
{
  waypoints_.reserve(size);
  durations_from_previous_.reserve(size);
}

void Trajectory::append(const RobotState & state, double duration_from_previous)
{
  validateWaypoint(state, duration_from_previous);
  waypoints_.push_back(state);
  durations_from_previous_.push_back(duration_from_previous);
}

void Trajectory::append(RobotState && state, double duration_from_previous)
{
  validateWaypoint(state, duration_from_previous);
  waypoints_.push_back(std::move(state));
  durations_from_previous_.push_back(duration_from_previous);
}

void Trajectory::appendHold(double duration)
{
  if (empty()) {
    throw std::invalid_argument("Cannot append a hold waypoint to an empty Trajectory");
  }

  append(waypoints_.back(), duration);
}

Trajectory Trajectory::resample(double time_step) const
{
  validateLayout();

  if (time_step <= 0.0) {
    throw std::invalid_argument("Trajectory resample time_step must be positive");
  }

  Trajectory resampled(layout_);
  if (empty()) {
    return resampled;
  }

  resampled.append(waypoints_.front(), 0.0);
  if (size() == 1) {
    return resampled;
  }

  const double total_duration = totalDuration();
  if (total_duration == 0.0) {
    if (waypoints_.back() != waypoints_.front()) {
      resampled.append(waypoints_.back(), time_step);
    }
    return resampled;
  }

  const std::size_t sample_count = static_cast<std::size_t>(std::ceil(total_duration / time_step));
  resampled.reserve(sample_count + 1);

  // Sample intermediate states on the uniform clock; the endpoint is appended explicitly below.
  for (std::size_t sample_index = 1; sample_index < sample_count; ++sample_index) {
    const double sample_time = static_cast<double>(sample_index) * time_step;
    resampled.append(sampleTrajectory(*this, sample_time), time_step);
  }

  // Preserve the exact endpoint state while delaying its arrival to the next time-step boundary.
  resampled.append(waypoints_.back(), time_step);
  return resampled;
}

void Trajectory::validateLayout() const
{
  if (layout_ == nullptr) {
    throw std::invalid_argument("Trajectory layout pointer must not be null");
  }
}

void Trajectory::validateWaypoint(const RobotState & state, double duration_from_previous) const
{
  validateLayout();

  if (!isCompatible(state)) {
    throw std::invalid_argument("RobotState is incompatible with Trajectory layout");
  }

  if (duration_from_previous < 0.0) {
    throw std::invalid_argument("Trajectory durations must be non-negative");
  }

  if (empty() && duration_from_previous != 0.0) {
    throw std::invalid_argument(
            "The first Trajectory waypoint must have zero duration from previous");
  }
}

}  // namespace stop_n_go::base
