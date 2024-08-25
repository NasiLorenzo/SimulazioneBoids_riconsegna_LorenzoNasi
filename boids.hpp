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
  int y;
  int x;
  paramlist()
      : pixel(params::dim)
  {}
};

struct gridID
{
  int y;
  int x;

  bool operator==(const gridID& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct gridID_hash
{
  std::size_t operator()(gridID const& other) const noexcept
  {
    return std::hash<int>{}(other.x) ^ (std::hash<int>{}(other.y) << 1);
  }
};

struct boid
{
  DoubleVec pos_;
  DoubleVec vel_;
  gridID GridID{};
  unsigned int flockID{0};
  boid(DoubleVec pos, DoubleVec vel)
      : pos_{pos}
      , vel_{vel}
  {}
  boid()
      : pos_{}
      , vel_{}
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
  DoubleVec deltavel_{};
  std::vector<boid const*> neighbors{};
  std::vector<boid const*> close_neighbors{};

 public:
  boidstate(boid const& other)
      : boid_{other}
      , neighbors{}
      , close_neighbors{}
  {}
  boidstate()
      : boid_{}
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

  auto& get_boid()
  {
    return this->boid_;
  }

  auto& get_deltavel()
  {
    return this->deltavel_;
  }
  void random_boid(std::default_random_engine&, paramlist const& params);

  /*void speedadjust(double speedlimit, double speedminimum);

  void bordercheck(std::vector<unsigned int> const& pixel,
                   const double bordersize, const double attraction);

  void update_neighbors(std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
                        const double align_distance, const double alpha,
                        Criterion const criterion, const int x);

  void update_close_neighbors(std::vector<boid const*> const& set,
                              const double repulsion_distance);

  void
  update_close_neighbors(std::unordered_multimap<gridID, boid const*, gridID_hash> const&
  map, const double repulsion_distance, const int x, const double alpha);*/

  void regola1(const double repulsione);
  void regola2_3(const double steering, const double cohesion);
  void posvel_update(paramlist const& params);

  void
  update_allneighbors(std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
                      const double repulsion_distance,
                      const double align_distance, const double alpha,
                      unsigned int size, unsigned int flocksize, const int x);
  void update_rules(paramlist const& params);
};

void regola1(boid const& boid_, DoubleVec& deltavel_,
             std::vector<boid const*> const& close_neighbors,
             const double repulsione);

void regola2_3(boid const& boid_, DoubleVec& deltavel_, const double steering,
               std::vector<boid const*> const& neighbors,
               const double cohesion);

void UpdateID(boid& boid, const double view_range);

void update_rules(boid const& boid_, DoubleVec& deltavel_,
                  std::vector<boid const*>& neighbors,
                  std::vector<boid const*>& close_neighbors,
                  paramlist const& params);

auto random_boid(std::default_random_engine& eng, paramlist const& params);

void speedadjust(boid& boid, double speedlimit, double speedminimum);

void bordercheck(boid& boid, std::vector<unsigned int> const& pixel,
                 const double bordersize, const double attraction);

void update_neighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                      std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
                      const double align_distance, const double alpha,
                      Criterion const criterion, const int x);

void update_close_neighbors(boid const& boid_,
                            std::vector<boid const*>& close_neighbors,
                            std::vector<boid const*> const& set,
                            const double repulsion_distance);

void update_close_neighbors(
    boid const& boid_, std::vector<boid const*>& close_neighbors,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    const double repulsion_distance, const int x, const double alpha);

void update_allneighbors(
    boid const& boid_, std::vector<boid const*>& neighbors,
    std::vector<boid const*>& close_neighbors,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    const double repulsion_distance, const double align_distance,
    const double alpha, unsigned int size, unsigned int flocksize, const int x);

std::size_t hash_function(gridID const& GridID);

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params);

struct gridID_hasher
{
  std::size_t operator()(const gridID& ID) const
  {
    return std::hash<int>()(ID.x) ^ std::hash<int>()(ID.y);
  }
};

struct gridID_compare
{
  bool operator()(const gridID& a, const gridID& b) const
  {
    return a.x == b.x && a.y == b.y;
  }
};



/*template<>
struct std::hash<gridID>
{
  std::size_t operator()(gridID const& other) const noexcept
  {
    return std::hash<int>{}(other.x) ^ (std::hash<int>{}(other.y) << 1);
  }
};*/

class flock
{
  std::vector<boidstate> set;
  std::unordered_multimap<gridID, boid const*,gridID_hash> HashMap{};

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

  auto& cget_set_() const
  {
    return this->set;
  }

  auto& get_set_()
  {
    return this->set;
  }

  auto& cget_Map_() const
  {
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