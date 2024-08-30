#include "boids.hpp"
namespace boids {

using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
ParamList::ParamList(std::string const& inputfile)
    : ParamList{}
{
  std::ifstream input{inputfile};
  if (!input) {
    throw std::runtime_error{"Input file not found"};
  }
  std::string line{};
  while (std::getline(input, line)) {
    std::istringstream inputline(line);
    double value{};
    std::string name{};
    if (inputline >> name >> value) {
      if (name == "repulsion_factor") {
        repulsion_factor = value;
        if (repulsion_factor < 0)
          throw std::runtime_error{"repulsion factor must be non-negative"};
      } else if (name == "steering_factor") {
        steering_factor = value;
        if (steering_factor < 0)
          throw std::runtime_error{"steering factor must be non-negative"};
      } else if (name == "cohesion_factor") {
        cohesion_factor = value;
        if (cohesion_factor < 0)
          throw std::runtime_error{"cohesion factor must be non-negative"};
      } else if (name == "view_range") {
        view_range = value;
        if (view_range <= 0)
          throw std::runtime_error{"view_range must be positive"};
      } else if (name == "repulsion_range") {
        repulsion_range = value;
        if (repulsion_range <= 0)
          throw std::runtime_error{"repulsion_range must be positive"};
      } else if (name == "border_repulsion") {
        border_repulsion = value;
        if (border_repulsion <= 0)
          throw std::runtime_error{"border_repulsion must be positive"};
      } else if (name == "alpha") {
        alpha = value * M_PI;
        if (value < -1 || value > 1)
          throw std::runtime_error{"alpha must be between -1 and 1"};
      } else if (name == "speedlimit") {
        speedlimit = value;
        if (speedlimit <= 0)
          throw std::runtime_error{"speedlimit must be positive"};
      } else if (name == "speedminimum") {
        speedminimum = value;
        if (speedminimum <= 0)
          throw std::runtime_error{"speedminimum must be positive"};
        if (speedminimum >= speedlimit)
          throw std::runtime_error{"speedminimum must be smaller speedlimit"};
      } else if (name == "deltaT") {
        deltaT = static_cast<float>(value);
        if (deltaT <= 0)
          throw std::runtime_error{"deltaT must be positive"};
      } else if (name == "size") {
        if (value <= 0)
          throw std::runtime_error{"size must be positive"};
        size = static_cast<unsigned int>(value);
      } else if (name == "flocksize") {
        if (value <= 0)
          throw std::runtime_error{"flocksize must be positive"};
        flocksize = static_cast<unsigned int>(value);
        if (flocksize > size)
          throw std::runtime_error{"flocksize must be smaller than size"};
      } else if (name == "pixel.x") {
        if (value <= 0)
          throw std::runtime_error{"pixels must be positive"};
        pixel[0] = static_cast<unsigned int>(value);
      } else if (name == "pixel.y") {
        if (value <= 0)
          throw std::runtime_error{"pixels must be positive"};
        pixel[1] = static_cast<unsigned int>(value);
      } else if (name == "pixel.z" && params::dim == 3) {
        if (value <= 0)
          throw std::runtime_error{"pixels must be positive"};
        pixel[2] = static_cast<unsigned int>(value);
      } else if (name == "rate") {
        rate = value;
        if (rate <= 0)
          throw std::runtime_error{"rate must be positive"};
      } else if (name == "bordersize") {
        bordersize = value;
        if (bordersize < 0)
          throw std::runtime_error{"bordersize must be non-negative"};
      } else if (name == "sigma") {
        sigma = value;
        if (sigma <= 0)
          throw std::runtime_error{"sigma must be positive"};
      }
    } else {
      throw std::runtime_error{
          "Can't properly read line in parameter initializer"};
    }
  }
}

void check_parallelism(int argc, char* argv[], ParamList& params)
{
  std::string argument = "--parallel";
  std::for_each(argv, argv + argc, [&](auto& opt) {
    auto new_opt = std::string(opt);
    if (new_opt == argument) {
      params.ExecPolicy = std::execution::par_unseq;
      std::cout << "parallel found! " << "\n";
    }
  });
}

std::size_t gridID_hash::operator()(GridID const& other) const noexcept
{
  auto result = std::hash<int>{}(other[0]);
  std::for_each(other.begin() + 1, other.end(), [&](auto& ID_comp) {
    result ^= std::hash<int>{}(ID_comp) /*+ 0x9e3779b9*/ + (result << 6)
            + (result >> 2);
  });
  return result;
}

bool operator==(GridID const& lhs, GridID const& rhs)
{
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

auto random_boid(std::default_random_engine& eng, ParamList const& params) noexcept
{
  BoidState newboid{};
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(newboid.vel().begin(), newboid.vel().end(),
                [&](double& x) { x = dist(eng); });
  return newboid;
}

void update_id(Boid& boid, const double view_range) noexcept
{
  auto posit = boid.pos().begin();
  std::for_each(boid.GridID().begin(), boid.GridID().end(), [&](auto& ID_comp) {
    ID_comp = static_cast<int>(std::floor(*posit / view_range) + 1);
    ++posit;
  });
}

void speed_adjust(Boid& boid, double speedlimit, double speedminimum) noexcept
{
  auto vmod = mod(boid.vel());
  if (vmod > speedlimit) {
    normalize(boid.vel());
    boid.vel() = boid.vel() * speedlimit;
  }
  if (vmod < speedminimum) {
    normalize(boid.vel());
    boid.vel() = boid.vel() * speedminimum;
  };
}
void bordercheck(Boid& boid, std::array<unsigned int, params::dim> const& pixel,
                 double bordersize, double border_repulsion, double rate) noexcept
{
  auto pix_iter = pixel.begin();
  auto pos_iter = boid.pos().begin();
  std::for_each(boid.vel().begin(), boid.vel().end(), [&](auto& vel) mutable {
    if (*pos_iter > rate * (*pix_iter - bordersize)) {
      vel -= border_repulsion;
    } else if (*pos_iter < rate * bordersize) {
      vel += border_repulsion;
    }
    pos_iter++;
    pix_iter++;
  });
}

bool is_neighbor(Boid const& boid_, Boid const& neighbor, double view_range,
                 double alpha, Criterion criterion) noexcept
{
  auto distanza = distance_squared(boid_.pos(), neighbor.pos());
  if (distanza < pow(view_range, 2) && distanza != 0 /*&neighbor != &boid_*/
      && (criterion == Criterion::any
          || (criterion == Criterion::similar
              && boid_.flockID() == neighbor.flockID()))) {
    auto cosangolo =
        cos_angle_between(neighbor.pos() - boid_.pos(), boid_.vel());
    if ((cosangolo) > std::cos(alpha)) {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

void add_neighbors(GridID const& neighborID, Boid const& boid_,
                   double view_range, double alpha, Criterion criterion,
                   MyHashMap const& map, std::vector<Boid const*>& neighbors) noexcept
{
  auto neigh_range = map.equal_range(neighborID);
  std::for_each(neigh_range.first, neigh_range.second, [&](auto& neighbor) {
    if (is_neighbor(boid_, *(neighbor.second), view_range, alpha, criterion))
      neighbors.push_back(neighbor.second);
  });
}

void update_neighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                      MyHashMap const& map, double align_distance, double alpha,
                      Criterion const criterion) noexcept
{
  neighbors.clear();
  std::array<int, 3> grid_range = {-1, 0, 1};
  std::size_t combinations_size =
      static_cast<std::size_t>(std::pow(3, params::dim));

  for (std::size_t i = 0; i < combinations_size; ++i) {
    GridID neighbor_ID;
    std::size_t index = i;

    for (std::size_t j = 0; j < params::dim; ++j) {
      neighbor_ID[j] = grid_range[index % 3] + boid_.GridID()[j];
      index /= 3;
    }
    add_neighbors(neighbor_ID, boid_, align_distance, alpha, criterion, map,
                  neighbors);
  }
}

std::vector<GridID>
update_neighbors_testing(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         MyHashMap const& map, double align_distance,
                         double alpha, Criterion criterion) noexcept
{
  neighbors.clear();
  std::array<int, 3> grid_range = {-1, 0, 1};
  std::size_t combinations_size =
      static_cast<std::size_t>(std::pow(3, params::dim));
  std::vector<GridID> all_combinations{};
  for (std::size_t i = 0; i < combinations_size; ++i) {
    GridID neighbor_ID;
    std::size_t index = i;

    for (std::size_t j = 0; j < params::dim; ++j) {
      neighbor_ID[j] = grid_range[index % 3] + boid_.GridID()[j];
      index /= 3;
    }
    all_combinations.push_back(neighbor_ID);
    add_neighbors(neighbor_ID, boid_, align_distance, alpha, criterion, map,
                  neighbors);
  }
  return all_combinations;
}

void update_close_neighbors(Boid const& boid_,
                            std::vector<Boid const*>& close_neighbors,
                            std::vector<Boid const*> const& neighbors,
                            double repulsion_distance) noexcept
{
  close_neighbors.clear();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto distanza = distance_squared(boid_.pos(), neighbor->pos());
    if (distanza < pow(repulsion_distance, 2)
        && /*neighbor != &boid_*/ distanza != 0) {
      close_neighbors.emplace_back(neighbor);
    }
  });
}

void rule_repulsion(Boid const& boid_, DoubleVec& deltavel_,
             std::vector<Boid const*> const& close_neighbors,
             double repulsion_factor) noexcept
{
  std::for_each(close_neighbors.begin(), close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->pos() - boid_.pos();
                  deltavel_ += x * (-repulsion_factor);
                });
}

void regola2_3_old(Boid const& boid_, DoubleVec& deltavel_,
                   std::vector<Boid const*> const& neighbors,
                   double steering_factor, double cohesion)
{
  auto n = neighbors.size();
  // auto velcopia = this->get_vel();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->vel_ - boid_.vel_;
    deltavel_ += x * (steering_factor / static_cast<double>(n));
    auto y = neighbor->pos_ - boid_.pos_;
    deltavel_ += y * (cohesion / static_cast<double>(n));
  });
  // deltavel_+=(boid_.pos_)*(-cohesion);
  // deltavel_+=boid_.vel_*(-steering_factor);
}

void rules_cohesion_alignment(Boid const& boid_, DoubleVec& deltavel_,
               std::vector<Boid const*> const& neighbors,
               double steering_factor, double cohesion) noexcept
{
  auto n = neighbors.size();
  // auto velcopia = this->get_vel();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    deltavel_ += neighbor->vel_ * (steering_factor / static_cast<double>(n));
    deltavel_ += neighbor->pos_ * (cohesion / static_cast<double>(n));
  });
  if (neighbors.size() != 0) {
    deltavel_ -= boid_.pos_ * cohesion;
    deltavel_ -= boid_.vel_ * steering_factor;
  }
}

void posvel_update(BoidState& boid, ParamList const& params) noexcept
{
  boid.vel() += boid.deltavel();
  speed_adjust(boid.boid(), params.speedlimit, params.speedminimum);
  boid.pos() += (boid.vel()) * params.deltaT;
  bordercheck(boid.boid(), params.pixel, params.bordersize,
              params.border_repulsion, params.rate);
  boid.deltavel() = {0., 0.};
  update_id(boid.boid(), params.view_range);
}

void update_allneighbors(Boid const& boid_, std::vector<Boid const*>& neighbors,
                         std::vector<Boid const*>& close_neighbors,
                         MyHashMap const& map, double repulsion_distance,
                         double align_distance, double alpha, unsigned int size,
                         unsigned int flocksize) noexcept
{
  if (flocksize < size) {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::similar);
    update_neighbors(boid_, close_neighbors, map, repulsion_distance, alpha,
                     Criterion::similar);
  } else {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::any);
    update_close_neighbors(boid_, close_neighbors, neighbors,
                           repulsion_distance);
  }
}

void update_rules(Boid const& boid_, DoubleVec& deltavel_,
                  std::vector<Boid const*>& neighbors,
                  std::vector<Boid const*>& close_neighbors,
                  ParamList const& params) noexcept
{
  rule_repulsion(boid_, deltavel_, close_neighbors, params.repulsion_factor);
  rules_cohesion_alignment(boid_, deltavel_, neighbors, params.steering_factor,
            params.cohesion_factor);
}

std::vector<BoidState> generate_flock(std::default_random_engine& eng,
                                      ParamList const& params)
{
  std::vector<BoidState> set{};
  MyHashMap HashMap{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    BoidState boidprova{random_boid(eng, params)};
    boidprova.flockID() = i / params.flocksize;
    // std::cout << "Il Flock id vale: " << boidprova.cget_boid().flockID <<
    // "\n";
    update_id(boidprova.boid(), params.view_range);
    for (auto it = boidprova.pos().begin(); it != boidprova.pos().end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          params.bordersize * params.rate,
          static_cast<double>((*pix - params.bordersize) * params.rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }

  return set;
}

void Flock::update_hashMap()
{
  hashMap_.clear();
  std::for_each(set_.begin(), set_.end(), [&](auto& boid) {
    hashMap_.insert({boid.GridID(), &(boid.boid())});
  });
}

void Flock::update(ParamList const& params)
{
  auto update_neighbors_rules = [&](auto&& policy) {
    std::for_each(policy, set_.begin(), set_.end(), [&](auto& boid) {
      update_allneighbors(boid.boid(), boid.neighbors(), boid.close_neighbors(),
                          hashMap_, params.repulsion_range, params.view_range,
                          params.alpha, params.size, params.flocksize);
      update_rules(boid.boid(), boid.deltavel(), boid.neighbors(),
                   boid.close_neighbors(), params);
    });
    std::for_each(policy, set_.begin(), set_.end(),
                  [&](auto& boid) { posvel_update(boid, params); });
  };
  std::visit(update_neighbors_rules, params.ExecPolicy);
  update_hashMap();
}

} // namespace boids
