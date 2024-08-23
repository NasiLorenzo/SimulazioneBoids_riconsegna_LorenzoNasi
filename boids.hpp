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
  double view_range;
  double repulsion_range;
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
  int rows;
  int columns;
  paramlist()
      : pixel(params::dim)
  {}
};

struct gridID
{
  int rows;
  int columns;
};

struct boid
{
  DoubleVec pos_;
  unsigned int flockID{0};
  gridID GridID{};
  DoubleVec vel_;
  DoubleVec deltavel_{};
  boid() = default;
  boid(DoubleVec pos, DoubleVec vel)
      : pos_{pos}
      , vel_{vel}
      , deltavel_{}
  {}
  auto& cget_pos() const
  {
    return this->pos_;
  }

  auto& cget_vel() const
  {
    return this->vel_;
  }
  auto get_velcopy() const
  {
    return this->vel_;
  }
  auto get_poscopy() const
  {
    return this->pos_;
  }
  auto& cget_flockID() const
  {
    return this->flockID;
  }
};
class boidstate
{
 private:
  boid boid_;
  std::vector<boid const*> neighbors{};
  std::vector<boid const*> close_neighbors{};

 public:
  boidstate() = default;
  boidstate(boid& other)
      : boid_{other}
      , neighbors{}
      , close_neighbors{}
  {}
  auto& cget_pos() const
  {
    return this->boid_.pos_;
  }

  auto& cget_vel() const
  {
    return this->boid_.vel_;
  }
  auto get_velcopy() const
  {
    return this->boid_.vel_;
  }
  auto get_poscopy() const
  {
    return this->boid_.pos_;
  }
  auto& get_ID() const
  {
    return this->boid_.flockID;
  }
  auto& cget_flockID() const
  {
    return boid_.flockID;
  }
  auto& cget_GridID() const
  {
    return boid_.GridID;
  }

  auto& cget_boid() const
  {
    return this->boid_;
  }
  auto& get_pos()
  {
    // std::mutex idmutex;
    // std::lock_guard<std::mutex> lock(idmutex);
    return this->boid_.pos_;
  }

  auto& get_vel()
  {
    // std::mutex idmutex;
    // std::lock_guard<std::mutex> lock(idmutex);
    return this->boid_.vel_;
  }

  auto& set_ID()
  {
    return this->boid_.flockID;
  }

  auto& set_GridID()
  {
    return boid_.GridID;
  }
  auto& set_boid()
  {
    return boid_;
  }
  auto& get_neighbors()
  {
    return neighbors;
  }

  auto& cget_neighbors() const
  {
    return neighbors;
  }

  auto& get_close_neighbors()
  {
    return close_neighbors;
  }

  void random_boid(std::default_random_engine&, paramlist const& params);

  void speedadjust(double speedlimit, double speedminimum);

  void bordercheck(std::vector<unsigned int> const& pixel,
                   const double bordersize, const double attraction);

  void update_neighbors(std::unordered_multimap<int, boid const*> const& map,
                        const double align_distance, const double alpha,
                        Criterion const criterion, const int columns);

  void update_close_neighbors(std::vector<boid const*> const& set,
                              const double repulsion_distance);

  void
  update_close_neighbors(std::unordered_multimap<int, boid const*> const& map,
                         const double repulsion_distance, const int columns,
                         const double alpha);

  void regola1(const double repulsione);
  void regola2_3(const double steering, const double cohesion);
  void posvel_update(paramlist const& params);

  void update_allneighbors(std::unordered_multimap<int, boid const*> const& map,
                           const double repulsion_distance,
                           const double align_distance, const double alpha,
                           unsigned int size, unsigned int flocksize,
                           const int columns);
  void update_rules(paramlist const& params);
};

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params);

void UpdateID(boid& boid, const double view_range);

int hash_function(gridID const& GridID, const int columns);

class flock
{
  std::vector<boidstate> set;
  std::unordered_multimap<int, boid const*> HashMap{};

 public:
  flock()
      : set{}
  {}

  flock(std::default_random_engine& eng, paramlist const& params)
      : set{generate_flock(eng, params)}
  {}
  flock(std::vector<boidstate> const& other, paramlist const& params)
      : set{other}
  {
    std::for_each(set.begin(), set.end(), [&params](auto& boid) {
      UpdateID(boid.set_boid(), params.view_range);
    });
    update_HashMap(params);
    std::for_each(set.begin(), set.end(), [&params](auto& boid) {
      UpdateID(boid.set_boid(), params.view_range);
    });
    update_HashMap(params);
  }
  std::vector<boidstate>& set_()
  {
    return set;
  }

  auto& cget_set_() const {
    return this->set;
  }

  auto& get_set_()  {
    return this->set;
  }

  auto& cget_Map_() const {
    return this->HashMap;
  }

  std::size_t size_()
  {
    return set.size();
  }
  void update_HashMap(paramlist const& params);
  void update(paramlist const& params);
};
} // namespace boids
#include "boids.tpp" // namespace boids
#endif