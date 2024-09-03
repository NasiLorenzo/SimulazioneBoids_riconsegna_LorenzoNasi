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

class Boid
{
 public:
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
  auto& gridID() const
  {
    return this->gridID_;
  }
  auto& gridID()
  {
    return this->gridID_;
  }

 private:
  DoubleVec pos_;
  DoubleVec vel_;
  GridID gridID_{};
  unsigned int flockID_{0};
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
    return this->boid_.pos();
  }
  auto& pos()
  {
    return this->boid_.pos();
  }

  auto& vel() const
  {
    return this->boid_.vel();
  }
  auto& vel()
  {
    return this->boid_.vel();
  }
  auto& flockID() const
  {
    return this->boid_.flockID();
  }
  auto& flockID()
  {
    return boid_.flockID();
  }
  auto& gridID() const
  {
    return boid_.gridID();
  }
  auto& gridID()
  {
    return boid_.gridID();
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

auto random_boid(std::default_random_engine&, ParamList const& params);

void rule_repulsion(Boid const& boid_, DoubleVec& deltavel_,
                    std::vector<Boid const*> const& close_neighbors,
                    double repulsion_factor);

void rules_cohesion_alignment(Boid const& boid_, DoubleVec& deltavel_,
                              double steering_factor,
                              std::vector<Boid const*> const& neighbors,
                              double cohesion);

void update_id(Boid& boid, double view_range);

void update_rules(Boid const& boid_, DoubleVec& deltavel_,
                  std::vector<Boid const*>& neighbors,
                  std::vector<Boid const*>& close_neighbors,
                  ParamList const& params);

void speed_adjust(Boid& boid, double speedlimit, double speedminimum);

void bordercheck(Boid& boid, std::vector<unsigned int> const& pixel,
                 double bordersize, double border_repulsion, double rate);

bool is_neighbor(Boid const& boid_, Boid const& neighbor, double view_range,
                 double alpha, Criterion criterion);

void add_neighbors(GridID const& neighborID, Boid const& boid_,
                   double view_range, double alpha, Criterion criterion,
                   MyHashMap const& map, std::vector<Boid const*>& neighbors);

void update_neighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                      MyHashMap const& map, double align_distance, double alpha,
                      Criterion criterion);

std::vector<GridID>
update_neighbors_testing(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         MyHashMap const& map, double align_distance,
                         double alpha, Criterion criterion);

void update_close_neighbors(Boid const& boid_,
                            std::vector<Boid const*>& close_neighbors,
                            std::vector<Boid const*> const& set,
                            double repulsion_distance);

void posvel_update(BoidState& boid, ParamList const& params);

void update_allneighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         std::vector<Boid const*>& close_neighbors,
                         MyHashMap const& map, double repulsion_distance,
                         double align_distance, double alpha, unsigned int size,
                         unsigned int flocksize);

void update_rules(Boid const& boid_, DoubleVec& deltavel_,
                  std::vector<Boid const*>& neighbors,
                  std::vector<Boid const*>& close_neighbors,
                  ParamList const& params);

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
    update_hashMap(params);
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
  void update_hashMap(ParamList const& params);
  void update(ParamList const& params);

 private:
  std::vector<BoidState> set_;
  MyHashMap hashMap_{};
};

} // namespace boids
#endif