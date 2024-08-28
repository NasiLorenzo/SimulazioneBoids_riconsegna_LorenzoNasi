#include "statistics.hpp"
namespace boids {

Sample::Sample(std::vector<double> const& entries)
    : Sample{}
{
  std::for_each(entries.begin(), entries.end(), [&](auto& x) {
    sum_x_ += x;
    sum_x2_ += x * x;
    N_++;
  });
}

Statistics Sample::result()
{
  if (N_ > 0 && !update_state_) {
    parameters_.mean  = sum_x_ / N_;
    parameters_.sigma = sqrt(sum_x2_ / N_ - pow(parameters_.mean, 2));
    update_state_     = true;
    return parameters_;
  } else {
    if (N_ == 0) {
      std::cerr << "Sample is empty" << "\n";
      return Statistics{0., -1}; // sentinel state
    } else {
      return parameters_;
    }
  }
}

void Sample::add(double entry)
{
  sum_x_ += entry;
  sum_x2_ += entry * entry;
  N_++;
  update_state_ = false;
}

void FlockStats::build_distance_entries(std::vector<BoidState> const& flock)
{
  std::for_each(flock.begin(), flock.end() - 1, [&](auto const& boid) {
    auto next_iter = std::next(flock.begin(), &boid - &(*flock.begin()) + 1);
    std::for_each(next_iter, flock.end(),
                  [&](auto& neighbor) { // executing diagonal search
                    distance_stats.add(
                        sqrt(distance(boid.pos(), neighbor.pos())));
                  });
  });
}
void FlockStats::build_pos_vel_entries(
    std::vector<BoidState> const& flock) // put together to optimize performace
{
  std::for_each(flock.begin(), flock.end(), [&](auto const& boid) {
    auto pos_iter = boid.pos().begin();
    auto vel_iter = boid.vel().begin();
    std::for_each(pos_stats.begin(), pos_stats.end(),
                  [&pos_iter](auto& sample) mutable {
                    sample.add(*pos_iter);
                    ++pos_iter;
                  });
    std::for_each(vel_stats.begin(), vel_stats.end(),
                  [&vel_iter](auto& sample) mutable {
                    sample.add(*vel_iter);
                    ++vel_iter;
                  });
    pos_mod_stats.add(mod(boid.pos()));
    vel_mod_stats.add(mod(boid.vel()));
  });
}

FlockStats::FlockStats(std::vector<BoidState> const& flock)
    : FlockStats{}
{
  build_pos_vel_entries(flock);
  build_distance_entries(flock);
}

} // namespace boids