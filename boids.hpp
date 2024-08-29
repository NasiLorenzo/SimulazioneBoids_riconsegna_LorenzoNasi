#ifndef BOIDS_HPP
#define BOIDS_HPP
#include "doublevec.hpp"
namespace boids {

enum class Criterion : bool
{
  any     = 1,
  similar = 0,
};

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
  ParamList(std::string const& inputfile)
      : ParamList{}
  {
    std::ifstream input{inputfile};
    if (!input) {
      throw std::runtime_error{"Input file not found"};
      // return 1;
    }
    std::string line{};
    while (std::getline(input, line)) {
      std::istringstream inputline(line);
      double value{};
      std::string name{};
      if (inputline >> name >> value) {
        if (name == "repulsion_factor")
          repulsion_factor = value;
        else if (name == "steering_factor")
          steering_factor = value;
        else if (name == "cohesion_factor")
          cohesion_factor = value;
        else if (name == "view_range")
          view_range = value;
        else if (name == "repulsion_range")
          repulsion_range = value;
        else if (name == "border_repulsion")
          border_repulsion = value;
        else if (name == "alpha")
          alpha = value * M_PI;
        else if (name == "speedlimit")
          speedlimit = value;
        else if (name == "speedminimum")
          speedminimum = value;
        else if (name == "deltaT")
          deltaT = static_cast<float>(value);
        else if (name == "size")
          size = static_cast<unsigned int>(value);
        else if (name == "flocksize")
          flocksize = static_cast<unsigned int>(value);
        else if (name == "pixel.x")
          pixel[0] = static_cast<unsigned int>(value);
        else if (name == "pixel.y")
          pixel[1] = static_cast<unsigned int>(value);
        else if (name == "pixel.z" && params::dim == 3)
          pixel[2] = static_cast<unsigned int>(value);
        else if (name == "rate")
          rate = value;
        else if (name == "bordersize")
          bordersize = value;
        else if (name == "sigma")
          sigma = value;
      }
    }
  }
};

void check_parallelism(int argc, char* argv[], ParamList& params);

using GridID = std::array<int, params::dim>;

inline bool operator==(GridID const& lhs, GridID const& rhs)
{
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

struct gridID_hash
{
  inline std::size_t operator()(GridID const& other) const noexcept
  {
    auto result = std::hash<int>{}(other[0]);
    std::for_each(other.begin() + 1, other.end(), [&](auto& ID_comp) {
      result ^= std::hash<int>{}(ID_comp) /*+ 0x9e3779b9*/ + (result << 6)
              + (result >> 2);
    });
    return result;
  }
};

struct boid
{
  DoubleVec pos_;
  DoubleVec vel_;
  GridID GridID_{};
  unsigned int flockID_{0};

  boid(DoubleVec pos, DoubleVec vel)
      : pos_{pos}
      , vel_{vel}
  {}
  boid()
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
class BoidState
{
 private:
  boid boid_;
  DoubleVec deltavel_{};
  std::vector<boid const*> neighbors_{};
  std::vector<boid const*> close_neighbors_{};

 public:
  BoidState(boid const& other)
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

  void random_boid(std::default_random_engine&, ParamList const& params);
};

using MyHashMap = std::unordered_multimap<GridID, boid const*, gridID_hash>;

void regola1(boid const& boid_, DoubleVec& deltavel_,
             std::vector<boid const*> const& close_neighbors,
             const double repulsion_factor);

void regola2_3(boid const& boid_, DoubleVec& deltavel_,
               const double steering_factor,
               std::vector<boid const*> const& neighbors,
               const double cohesion);

void update_id(boid& boid, const double view_range);

void update_rules(boid const& boid_, DoubleVec& deltavel_,
                  std::vector<boid const*>& neighbors,
                  std::vector<boid const*>& close_neighbors,
                  ParamList const& params);

auto random_boid(std::default_random_engine& eng, ParamList const& params);

void speed_adjust(boid& boid, double speedlimit, double speedminimum);

void bordercheck(boid& boid, std::vector<unsigned int> const& pixel,
                 const double bordersize, const double border_repulsion,
                 const double rate);
void update_neighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                      MyHashMap const& map, const double align_distance,
                      const double alpha, Criterion const criterion);

void update_close_neighbors(boid const& boid_,
                            std::vector<boid const*>& close_neighbors,
                            std::vector<boid const*> const& set,
                            const double repulsion_distance);
void update_close_neighbors(boid const& boid_,
                            std::vector<boid const*>& close_neighbors,
                            MyHashMap const& map,
                            const double repulsion_distance, const double alpha,
                            Criterion const criterion);

void update_allneighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                         std::vector<boid const*>& close_neighbors,
                         MyHashMap const& map, const double repulsion_distance,
                         const double align_distance, const double alpha,
                         unsigned int size, unsigned int flocksize);

std::size_t hash_function(GridID const& gridID);

std::vector<BoidState> generate_flock(std::default_random_engine& eng,
                                      ParamList const& params);

class Flock
{
  std::vector<BoidState> set_;
  MyHashMap hashMap_{};

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
};

} // namespace boids
#include "boids.tpp" // namespace boids
#endif