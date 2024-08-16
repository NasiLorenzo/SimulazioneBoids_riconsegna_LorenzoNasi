#ifndef BOIDS_HPP
#define BOIDS_HPP
#include "doublevec.hpp"
namespace boids {

enum class Criterion : bool
{
  any     = 1,
  similar = 0,
};

struct paramlist
{
  double repulsione;
  double steering;
  double coesione;
  double neigh_align;
  double neigh_repulsion;
  double attraction;
  double alpha;
  double speedlimit;
  double speedminimum;
  float deltaT;
  unsigned int size;
  unsigned int flocksize;
  std::vector<unsigned int> pixel;
  double bordersize;
  double sigma;

  paramlist()
      : pixel(params::dim)
  {}
};
class boidstate
{
 private:
  DoubleVec pos_;
  DoubleVec vel_;
  unsigned int flockID{0};
  std::vector<boidstate const*> neighbors{};
  std::vector<boidstate const*> close_neighbors{};

 public:
  boidstate() = default;
  boidstate(DoubleVec pos, DoubleVec vel)
      : pos_{pos}
      , vel_{vel}
      , neighbors{}
      , close_neighbors{}
  {}
  auto get_pos() const
  {
    return this->pos_;
  }

  auto get_vel() const
  {
    return this->vel_;
  }
  auto get_velcopy()
  {
    return this->vel_;
  }
  auto get_ID() const
  {
    return this->flockID;
  }

  auto& set_pos()
  {
    return this->pos_;
  }

  auto& set_vel()
  {
    return this->vel_;
  }

  auto& set_ID()
  {
    return this->flockID;
  }

  auto get_neighbors() const
  {
    return this->neighbors;
  }

  auto get_close_neighbors() const
  {
    return this->close_neighbors;
  }

  void random_boid(std::default_random_engine&, paramlist const& params);

  void speedadjust(double speedlimit, double speedminimum);

  void bordercheck(std::vector<unsigned int> const& pixel,
                   const double bordersize, const double attraction);

  void update_neighbors(std::vector<boidstate> const& set,
                        const double align_distance, const double alpha,
                        Criterion criterion);

  void update_close_neighbors(std::vector<boidstate const*> const& set,
                              const double repulsion_distance);

  void update_close_neighbors(std::vector<boidstate> const& set,
                              const double repulsion_distance);

  void regola1(const double repulsione);
  void regola2_3(const double steering, const double cohesion);
  void pos_update(const float deltaT);

  void update_allneighbors(std::vector<boidstate> const& set,
                           const double repulsion_distance,
                           const double align_distance, const double alpha,
                           unsigned int size, unsigned int flocksize);
  void update_rules(paramlist const& params);
};

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params);

class flock
{
  std::vector<boidstate> set;

 public:
  flock(std::default_random_engine& eng, paramlist const& params)
      : set{generate_flock(eng, params)}
  {}
  flock(std::vector<boidstate> const& other)
      : set{other}
  {}
  std::vector<boidstate>& set_()
  {
    return set;
  }

  std::size_t size_()
  {
    return set.size();
  }

  void update(paramlist const& params);
};
} // namespace boids
#include "boids.tpp" // namespace boids
#endif