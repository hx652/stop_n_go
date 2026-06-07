#include <stop_n_go/core/search_node.hpp>

#include <stdexcept>
#include <utility>

namespace stop_n_go::core
{

SearchNode::SearchNode() = default;

SearchNode::SearchNode(std::vector<stop_n_go::base::Trajectory> trajectories)
: trajectories_(std::move(trajectories))
{
}

const std::vector<stop_n_go::base::Trajectory> & SearchNode::trajectories() const
{
  return trajectories_;
}

std::vector<stop_n_go::base::Trajectory> & SearchNode::trajectories()
{
  return trajectories_;
}

const stop_n_go::base::Trajectory & SearchNode::trajectory(stop_n_go::base::RobotId robot_id) const
{
  return trajectories_.at(robot_id);
}

stop_n_go::base::Trajectory & SearchNode::trajectory(stop_n_go::base::RobotId robot_id)
{
  return trajectories_.at(robot_id);
}

std::size_t SearchNode::robotCount() const
{
  return trajectories_.size();
}

bool SearchNode::empty() const
{
  return trajectories_.empty();
}

double SearchNode::g() const
{
  return g_cost_;
}

double SearchNode::h() const
{
  return h_cost_;
}

double SearchNode::f() const
{
  return g_cost_ + h_cost_;
}

void SearchNode::setG(double g_cost)
{
  if (g_cost < 0.0) {
    throw std::invalid_argument("SearchNode g cost must be non-negative");
  }

  g_cost_ = g_cost;
}

void SearchNode::setH(double h_cost)
{
  if (h_cost < 0.0) {
    throw std::invalid_argument("SearchNode h cost must be non-negative");
  }

  h_cost_ = h_cost;
}

std::size_t SearchNode::tS() const
{
  return t_s_;
}

void SearchNode::setTS(std::size_t t_s)
{
  t_s_ = t_s;
}

}  // namespace stop_n_go::core
