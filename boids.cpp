#include "boids.hpp"
namespace boids {

double params::rate{1};
using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

void check_parallelism(int argc, char* argv[], ParamList& params)
{
  const char* argument = "--parallel";
  auto it              = std::find(argv, argv + argc, argument);
  std::for_each(argv, argv + argc,
                [](auto& opt) { std::cout << "Opzione: " << opt << "\n"; });
  if (it != argv + argc + 1) {
    std::cout << "Found parallel" << "!" << std::endl;
    params.ExecPolicy = std::execution::par_unseq;
  }
}

auto random_boid(std::default_random_engine& eng, ParamList const& params)
{
  BoidState newboid{};
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(newboid.get_vel().begin(), newboid.get_vel().end(),
                [&](double& x) { x = dist(eng); });
  return newboid;
}

void UpdateID(boid& boid, const double view_range)
{
  auto posit = boid.cget_pos().begin();
  std::for_each(boid.GridID.begin(), boid.GridID.end(), [&](auto& ID_comp) {
    ID_comp = static_cast<int>(std::floor(*posit / view_range) + 1);
    ++posit;
  });
  // boid.GridID.gridID_[0] = static_cast<int>(std::floor(boid.pos_[0] /
  // view_range) + 1); boid.GridID.gridID_[1] =
  // static_cast<int>(std::floor(boid.pos_[1] / view_range) + 1);
}

/*std::size_t hash_function(gridID const& GridID)
{
  return std::hash<int>()(GridID.x) ^ std::hash<int>()(GridID.y);
}*/
void speedadjust(boid& boid, double speedlimit, double speedminimum)
{
  auto vmod = mod(boid.vel_);
  if (vmod > speedlimit) {
    normalize(boid.vel_);
    boid.vel_ = boid.vel_ * speedlimit;
  }
  if (vmod < speedminimum) {
    normalize(boid.vel_);
    boid.vel_ = boid.vel_ * speedminimum;
  };
}
void bordercheck(boid& boid, std::array<unsigned int, params::dim> const& pixel,
                 const double bordersize, const double border_repulsion)
{
  auto pix = pixel.begin();
  for (auto index = boid.pos_.begin(), velind = boid.vel_.begin();
       index != boid.pos_.end(); ++index, ++velind, ++pix) {
    if (*index > params::rate * (*pix - bordersize)) {
      *velind -= border_repulsion;
    } else if (*index < params::rate * bordersize) {
      *velind += border_repulsion;
    }
  }
}

bool is_neighbor(boid const& boid_, boid const& neighbor,
                 const double view_range, const double alpha,
                 Criterion criterion)
{
  auto distanza = distance(boid_.pos_, neighbor.pos_);
  if (distanza < pow(view_range, 2) && distanza != 0
      && (criterion == Criterion::any
          || (criterion == Criterion::similar
              && boid_.flockID == neighbor.flockID))) {
    auto cosangolo = cosangleij(neighbor.pos_ - boid_.pos_, boid_.vel_);
    if ((cosangolo) >= std::cos(alpha)) {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

void add_neighbors(
    gridID const& neighborID, boid const& boid_, const double view_range,
    const double alpha, Criterion criterion,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    std::vector<boid const*>& neighbors)
{
  auto neigh_range = map.equal_range(neighborID);
  std::for_each(neigh_range.first, neigh_range.second, [&](auto& neighbor) {
    if (is_neighbor(boid_, *(neighbor.second), view_range, alpha, criterion))
      neighbors.push_back(neighbor.second);
  });
}

auto generate_combinations(gridID const& boidID)
{
  std::vector<gridID> combinations{};

  // There are 3 possible values (-1, 0, 1) for each element
  std::array<int, 3> grid_range       = {-1, 0, 1};
  const std::size_t combinations_size = std::pow(3, params::dim);

  for (std::size_t i = 0; i < combinations_size; ++i) {
    std::array<int, params::dim> combination;
    std::size_t index = i;

    for (std::size_t j = 0; j < params::dim; ++j) {
      combination[j] = grid_range[index % 3] + boidID[j];
      index /= 3;
    }

    combinations.push_back(combination);
  }

  return combinations;
}

void update_neighbors(
    boid const& boid_, std::vector<boid const*>& neighbors,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    const double align_distance, const double alpha, Criterion const criterion)
{
  neighbors.clear();
  /*std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->boid_.pos_, neighbor.get_pos());
    if (distanza < pow(align_distance, 2) && distanza != 0
        && (criterion == Criterion::any
            || (criterion == Criterion::similar
                && this->boid_.flockID == neighbor.get_ID()))) {
      auto cosangolo =
          cosangleij(neighbor.get_pos() - this->boid_.pos_, this->boid_.vel_);
      if ((cosangolo) >= std::cos(alpha)) {
        this->neighbors.emplace_back(&(neighbor.boid_));
      }
    }
  });*/
  /*
  gridID startID{};
  startID[0] = boid_.GridID[0] - 2;
  startID[1] = boid_.GridID[1] - 1;
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 3; i++) {
        startID[0]++;
        auto neighrange = map.equal_range(startID);
        std::for_each(neighrange.first, neighrange.second, [&](auto& neighbor) {
          auto distanza = distance(boid_.pos_, neighbor.second->pos_);
          if (distanza < pow(align_distance, 2) && distanza != 0
              && (criterion == Criterion::any
                  || (criterion == Criterion::similar
                      && boid_.flockID == neighbor.second->flockID))) {
            auto cosangolo =
                cosangleij(neighbor.second->pos_ - boid_.pos_, boid_.vel_);
            if ((cosangolo) >= std::cos(alpha)) {
              neighbors.emplace_back(neighbor.second);
            }
          }
        });
        //std::cout<<"L'ID in uscita vale: ("<<startID[0]<<",
  "<<startID[1]<<")"<<"\n";
      }
      startID[0] += -3;
      startID[1]++;
    }*/
  /*auto combinations{generate_combinations(boid_.GridID)};
  std::for_each(combinations.begin(), combinations.end(), [&](auto& ID) {
    auto neigh_range = map.equal_range(ID);
    std::for_each(neigh_range.first, neigh_range.second, [&](auto& neighbor) {
      if (is_neighbor(boid_, *(neighbor.second), align_distance, alpha,
                      criterion))
        neighbors.push_back(neighbor.second);
    });
  });*/
  std::array<int, 3> grid_range       = {-1, 0, 1};
  const std::size_t combinations_size = std::pow(3, params::dim);

  for (std::size_t i = 0; i < combinations_size; ++i) {
    std::array<int, params::dim> combination;
    std::size_t index = i;

    for (std::size_t j = 0; j < params::dim; ++j) {
      combination[j] = grid_range[index % 3] + boid_.GridID[j];
      index /= 3;
    }
    // std::cout<<"la dimensione di comb è "<<combination.size()<<"\n";
    // std::cout << "controllo vicini in: (" << combination[0] << ", "
    //         << combination[1] <<" e "<<combination[2]<< ")\n";
    add_neighbors(combination, boid_, align_distance, alpha, criterion, map,
                  neighbors);
  }
}

void update_close_neighbors(boid const& boid_,
                            std::vector<boid const*>& close_neighbors,
                            std::vector<boid const*> const& set,
                            const double repulsion_distance)
{
  close_neighbors.clear();
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(boid_.pos_, neighbor->pos_);
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      close_neighbors.emplace_back(neighbor);
    }
  });
}
void update_close_neighbors(
    boid const& boid_, std::vector<boid const*>& close_neighbors,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    const double repulsion_distance, const double alpha,
    Criterion const criterion)
{
  close_neighbors.clear();

  /*gridID startID{};
  startID[0] = boid_.GridID[0] - 2;
  startID[1] = boid_.GridID[1] - 1;
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      startID[0]++;
      auto neighrange = map.equal_range(startID);
      std::for_each(neighrange.first, neighrange.second, [&](auto& neighbor) {
        auto distanza = distance(boid_.pos_, neighbor.second->pos_);
        if (distanza < pow(repulsion_distance, 2) && (distanza != 0)) {
          auto cosangolo =
              cosangleij(neighbor.second->pos_ - boid_.pos_, boid_.vel_);
          if ((cosangolo) >= std::cos(alpha)) {
            close_neighbors.emplace_back(neighbor.second);
          }
        }
      });
    }
    startID[0] += -3;
    startID[1]++;
  }*/
  std::array<int, 3> grid_range       = {-1, 0, 1};
  const std::size_t combinations_size = std::pow(3, params::dim);

  for (std::size_t i = 0; i < combinations_size; ++i) {
    std::array<int, params::dim> combination;
    std::size_t index = i;

    for (std::size_t j = 0; j < params::dim; ++j) {
      combination[j] = grid_range[index % 3] + boid_.GridID[j];
      index /= 3;
    }
    add_neighbors(combination, boid_, repulsion_distance, alpha, criterion, map,
                  close_neighbors);
  }
}

void regola1(boid const& boid_, DoubleVec& deltavel_,
             std::vector<boid const*> const& close_neighbors,
             const double repulsion_factor)
{
  std::for_each(close_neighbors.begin(), close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->pos_ - boid_.pos_;
                  deltavel_ += x * (-repulsion_factor);
                });
  // deltavel_+=boid_.pos_*(repulsion_factor);
}

void regola2_3(boid const& boid_, DoubleVec& deltavel_,
               std::vector<boid const*> const& neighbors,
               const double steering_factor, const double cohesion)
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

void posvel_update(BoidState& boid, ParamList const& params)
{
  // boid.get_neighbors().clear();
  // boid.get_close_neighbors().clear();
  boid.get_boid().vel_ += boid.get_deltavel();
  speedadjust(boid.get_boid(), params.speedlimit, params.speedminimum);
  boid.get_boid().pos_ += (boid.get_boid().vel_) * params.deltaT;
  bordercheck(boid.get_boid(), params.pixel, params.bordersize,
              params.border_repulsion);
  boid.get_deltavel() = {0., 0.};
  UpdateID(boid.get_boid(), params.view_range);
}

void update_allneighbors(
    boid const& boid_, std::vector<boid const*>& neighbors,
    std::vector<boid const*>& close_neighbors,
    std::unordered_multimap<gridID, boid const*, gridID_hash> const& map,
    const double repulsion_distance, const double align_distance,
    const double alpha, unsigned int size, unsigned int flocksize)
{
  if (flocksize < size) {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::similar);
    update_close_neighbors(boid_, close_neighbors, map, repulsion_distance,
                           alpha, Criterion::similar);
  } else {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::any);
    update_close_neighbors(boid_, close_neighbors, neighbors,
                           repulsion_distance);
  }
}

void update_rules(boid const& boid_, DoubleVec& deltavel_,
                  std::vector<boid const*>& neighbors,
                  std::vector<boid const*>& close_neighbors,
                  ParamList const& params)
{
  regola1(boid_, deltavel_, close_neighbors, params.repulsion_factor);
  regola2_3(boid_, deltavel_, neighbors, params.steering_factor,
            params.cohesion_factor);
}

std::vector<BoidState> generate_flock(std::default_random_engine& eng,
                                      ParamList const& params)
{
  std::vector<BoidState> set{};
  std::unordered_multimap<gridID, boid const*, gridID_hash> HashMap{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    BoidState boidprova{random_boid(eng, params)};
    // boidprova.random_boid(eng, params);
    boidprova.set_ID() = i / params.flocksize;
    // std::cout << "Il flock id vale: " << boidprova.cget_boid().flockID <<
    // "\n";
    UpdateID(boidprova.set_boid(), params.view_range);
    for (auto it = boidprova.get_pos().begin(); it != boidprova.get_pos().end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          params.bordersize * params::rate,
          static_cast<double>((*pix - params.bordersize) * params::rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }

  return set;
}

void flock::update_HashMap(ParamList const& params)
{
  // auto t1=high_resolution_clock::now();
  HashMap.clear();
  std::for_each(set.begin(), set.end(), [&](auto& boid) {
    HashMap.insert({boid.set_GridID(), &(boid.set_boid())});
  });
  /*auto t2=high_resolution_clock::now();
  duration<double, std::milli> ms_double=t2-t1;
  std::cout<<"Tempo creazione mappa: "<<ms_double.count()<<" ms"<<"\n";*/
}

void flock::update(ParamList const& params)
{
  auto update_neighbors_rules = [&](auto&& policy) {
    std::for_each(policy, set.begin(), set.end(), [&](auto& boid) {
      update_allneighbors(boid.cget_boid(), boid.get_neighbors(),
                          boid.get_close_neighbors(), HashMap,
                          params.repulsion_range, params.view_range,
                          params.alpha, params.size, params.flocksize);
      update_rules(boid.cget_boid(), boid.get_deltavel(), boid.get_neighbors(),
                   boid.get_close_neighbors(), params);
    });
    std::for_each(policy, set.begin(), set.end(),
                  [&](auto& boid) { posvel_update(boid, params); });
  };
  std::visit(update_neighbors_rules, params.ExecPolicy);
  update_HashMap(params);
}

} // namespace boids
