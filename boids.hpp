#ifndef BOIDS_HPP
#define BOIDS_HPP
#include "doublevec.hpp"
namespace boids {

enum class Criterion : bool
{
  any     = 1,
  similar = 0,
};

using GridID = std::array<int, params::dim>;

struct ParamList
{
  double repulsion_factor;
  double steering_factor;
  double cohesion_factor;
  double view_range;
  double repulsion_range;
  double border_repulsion;
  double alpha;
  double speedlimit;
  double speedminimum;
  float deltaT;
  unsigned int size;
  unsigned int flocksize;
  std::array<unsigned int, params::dim> pixel;
  double bordersize;
  double sigma;
  std::variant<std::execution::sequenced_policy,
               std::execution::parallel_unsequenced_policy>
      ExecPolicy = std::execution::seq;
  double rate;
  ParamList() = default;
  ParamList(std::string const& inputfile);
};

void check_parallelism(int argc, char* argv[], ParamList& params);

// declaration of custom hasher
struct gridID_hash
{
  std::size_t operator()(GridID const& other) const noexcept;
};

bool operator==(GridID const& lhs, GridID const& rhs);

struct Boid
{
  DoubleVec pos_;
  DoubleVec vel_;
  GridID GridID_{};
  unsigned int flockID_{0};

  Boid(DoubleVec pos, DoubleVec vel)
      : pos_{pos}
      , vel_{vel}
  {}
  Boid()
      : pos_{}
      , vel_{}
  {}
  auto& pos() const
  {
    return this->pos_;
  }

  auto& vel()
  {
    return this->vel_;
  }

  auto& pos()
  {
    return this->pos_;
  }

  auto& vel() const
  {
    return this->vel_;
  }
  auto& flockID() const
  {
    return this->flockID_;
  }
  auto& flockID()
  {
    return this->flockID_;
  }
  auto& GridID() const
  {
    return this->GridID_;
  }
  auto& GridID()
  {
    return this->GridID_;
  }
};
// custom hash_map
using MyHashMap = std::unordered_multimap<GridID, Boid const*, gridID_hash>;

class BoidState
{
 public:
  BoidState(Boid const& other)
      : boid_{other}
      , neighbors_{}
      , close_neighbors_{}
  {}
  BoidState()
      : boid_{}
  {}
  BoidState(DoubleVec&& pos, DoubleVec&& vel)
      : boid_{std::move(pos), std::move(vel)}
  {}
  auto& pos() const
  {
    return this->boid_.pos_;
  }
  auto& pos()
  {
    return this->boid_.pos_;
  }

  auto& vel() const
  {
    return this->boid_.vel_;
  }
  auto& vel()
  {
    return this->boid_.vel_;
  }
  auto& flockID() const
  {
    return this->boid_.flockID_;
  }
  auto& flockID()
  {
    return boid_.flockID_;
  }
  auto& GridID() const
  {
    return boid_.GridID_;
  }
  auto& GridID()
  {
    return boid_.GridID_;
  }

  auto& boid() const
  {
    return this->boid_;
  }
  auto& boid()
  {
    return this->boid_;
  }

  auto& neighbors() const
  {
    return neighbors_;
  }

  auto& neighbors()
  {
    return neighbors_;
  }

  auto& close_neighbors() const
  {
    return close_neighbors_;
  }

  auto& close_neighbors()
  {
    return close_neighbors_;
  }
  auto& deltavel() const
  {
    return this->deltavel_;
  }
  auto& deltavel()
  {
    return this->deltavel_;
  }

 private:
  Boid boid_;
  DoubleVec deltavel_{};
  std::vector<Boid const*> neighbors_{};
  std::vector<Boid const*> close_neighbors_{};
};

auto random_boid(std::default_random_engine&, ParamList const& params) noexcept;

void rule_repulsion(Boid const& boid_, DoubleVec& deltavel_,
                    std::vector<Boid const*> const& close_neighbors,
                    double repulsion_factor) noexcept;

void rules_cohesion_alignment(Boid const& boid_, DoubleVec& deltavel_,
                              double steering_factor,
                              std::vector<Boid const*> const& neighbors,
                              double cohesion) noexcept;

void update_id(Boid& boid, double view_range) noexcept;

void update_rules(Boid const& boid_, DoubleVec& deltavel_,
                  std::vector<Boid const*>& neighbors,
                  std::vector<Boid const*>& close_neighbors,
                  ParamList const& params) noexcept;

void speed_adjust(Boid& boid, double speedlimit, double speedminimum) noexcept;

void bordercheck(Boid& boid, std::vector<unsigned int> const& pixel,
                 double bordersize, double border_repulsion,
                 double rate) noexcept;

bool is_neighbor(Boid const& boid_, Boid const& neighbor, double view_range,
                 double alpha, Criterion criterion) noexcept;

void add_neighbors(GridID const& neighborID, Boid const& boid_,
                   double view_range, double alpha, Criterion criterion,
                   MyHashMap const& map,
                   std::vector<Boid const*>& neighbors) noexcept;

void update_neighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                      MyHashMap const& map, double align_distance, double alpha,
                      Criterion criterion) noexcept;

std::vector<GridID>
update_neighbors_testing(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         MyHashMap const& map, double align_distance,
                         double alpha, Criterion criterion) noexcept;

void update_close_neighbors(Boid const& boid_,
                            std::vector<Boid const*>& close_neighbors,
                            std::vector<Boid const*> const& set,
                            double repulsion_distance) noexcept;

void posvel_update(BoidState& boid, ParamList const& params) noexcept;

void update_allneighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         std::vector<Boid const*>& close_neighbors,
                         MyHashMap const& map, double repulsion_distance,
                         double align_distance, double alpha, unsigned int size,
                         unsigned int flocksize) noexcept;

void update_rules(Boid const& boid_, DoubleVec& deltavel_,
                  std::vector<Boid const*>& neighbors,
                  std::vector<Boid const*>& close_neighbors,
                  ParamList const& params) noexcept;

std::vector<BoidState> generate_flock(std::default_random_engine& eng,
                                      ParamList const& params);
class Flock
{
 public:
  Flock()
      : set_{}
  {}

  Flock(std::default_random_engine& eng, ParamList const& params)
      : set_{generate_flock(eng, params)}
  {}
  Flock(std::vector<BoidState> const& other, ParamList const& params)
      : set_{other}
  {
    std::for_each(set_.begin(), set_.end(), [&params](auto& boid) {
      update_id(boid.boid(), params.view_range);
    });
    update_hashMap();
    std::for_each(set_.begin(), set_.end(), [&params](auto& boid) {
      update_id(boid.boid(), params.view_range);
    });
    update_hashMap();
  }
  auto& set() const
  {
    return set_;
  }

  auto& set()
  {
    return set_;
  }

  auto& hashMap()
  {
    return this->hashMap_;
  }
  auto& hashMap() const
  {
    return this->hashMap_;
  }

  std::size_t size()
  {
    return set_.size();
  }
  void update_hashMap();
  void update(ParamList const& params);

 private:
  std::vector<BoidState> set_;
  MyHashMap hashMap_{};
};

} // namespace boids
#include "boids.tpp" // namespace boids
#endif