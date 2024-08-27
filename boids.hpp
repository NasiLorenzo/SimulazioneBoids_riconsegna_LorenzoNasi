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
  {
    std::ifstream input{inputfile};
    if (!input) {
      std::cerr << "File di input non trovato!\n";
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

using gridID = std::array<int, params::dim>;

inline bool operator==(gridID const& lhs, gridID const& rhs)
{
  bool result{1};
  auto rhs_it = rhs.begin();
  std::for_each(lhs.begin(), lhs.end(),
                [&](auto& x) { result = result && (x == *rhs_it); });
  return result;
}

struct gridID_hash
{
  inline std::size_t operator()(gridID const& other) const noexcept
  {
    int i       = 1;
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
class BoidState
{
 private:
  boid boid_;
  DoubleVec deltavel_{};
  std::vector<boid const*> neighbors{};
  std::vector<boid const*> close_neighbors{};

 public:
  BoidState(boid const& other)
      : boid_{other}
      , neighbors{}
      , close_neighbors{}
  {}
  BoidState()
      : boid_{}
  {}
  BoidState(DoubleVec&& pos, DoubleVec&& vel)
      : boid_{std::move(pos), std::move(vel)}
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

  auto& cget_close_neighbors() const
  {
    return close_neighbors;
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
  void random_boid(std::default_random_engine&, ParamList const& params);
};

using MyHashMap = std::unordered_multimap<gridID, boid const*, gridID_hash>;

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

std::size_t hash_function(gridID const& GridID);

std::vector<BoidState> generate_flock(std::default_random_engine& eng,
                                      ParamList const& params);

class Flock
{
  std::vector<BoidState> set;
  MyHashMap HashMap{};

 public:
  Flock()
      : set{}
  {}

  Flock(std::default_random_engine& eng, ParamList const& params)
      : set{generate_flock(eng, params)}
  {}
  Flock(std::vector<BoidState> const& other, ParamList const& params)
      : set{other}
  {
    std::for_each(set.begin(), set.end(), [&params](auto& boid) {
      update_id(boid.set_boid(), params.view_range);
    });
    update_HashMap(params);
    std::for_each(set.begin(), set.end(), [&params](auto& boid) {
      update_id(boid.set_boid(), params.view_range);
    });
    update_HashMap(params);
  }
  std::vector<BoidState>& set_()
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
  void update_HashMap(ParamList const& params);
  void update(ParamList const& params);
};
} // namespace boids
#include "boids.tpp" // namespace boids
#endif