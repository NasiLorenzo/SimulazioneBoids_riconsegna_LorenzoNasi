#include "boids.hpp"
namespace boids {

using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

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

auto random_boid(std::default_random_engine& eng, ParamList const& params)
{
  BoidState newboid{};
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(newboid.vel().begin(), newboid.vel().end(),
                [&](double& x) { x = dist(eng); });
  return newboid;
}

void update_id(boid& boid, const double view_range)
{
  auto posit = boid.pos().begin();
  std::for_each(boid.GridID().begin(), boid.GridID().end(), [&](auto& ID_comp) {
    ID_comp = static_cast<int>(std::floor(*posit / view_range)+1);
    ++posit;
  });
}

void speed_adjust(boid& boid, double speedlimit, double speedminimum)
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
void bordercheck(boid& boid, std::array<unsigned int, params::dim> const& pixel,
                 const double bordersize, const double border_repulsion,
                 const double rate)
{
  auto pix = pixel.begin();
  for (auto index = boid.pos().begin(), velind = boid.vel().begin();
       index != boid.pos().end(); ++index, ++velind, ++pix) {
    if (*index > rate * (*pix - bordersize)) {
      *velind -= border_repulsion;
    } else if (*index < rate * bordersize) {
      *velind += border_repulsion;
    }
  }
}

bool is_neighbor(boid const& boid_, boid const& neighbor,
                 const double view_range, const double alpha,
                 Criterion criterion)
{
  auto distanza = distance_squared(boid_.pos(), neighbor.pos());
  if (distanza < pow(view_range, 2) && distanza != 0 /*&neighbor != &boid_*/
      && (criterion == Criterion::any
          || (criterion == Criterion::similar
              && boid_.flockID() == neighbor.flockID()))) {
    auto cosangolo =
        cos_angle_between(neighbor.pos() - boid_.pos(), boid_.vel());
    std::cout<<"cosangolo: "<<cosangolo<<" e cosalpha"<<std::cos(alpha)<<"\n";
    std::cout<<"Stato angolo: "<<isgreaterequal((cosangolo),std::cos(alpha))<<"\n";
    if (isgreaterequal((cosangolo),std::cos(alpha))) {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

void add_neighbors(GridID const& neighborID, boid const& boid_,
                   const double view_range, const double alpha,
                   Criterion criterion, MyHashMap const& map,
                   std::vector<boid const*>& neighbors)
{
  auto neigh_range = map.equal_range(neighborID);
  std::for_each(neigh_range.first, neigh_range.second, [&](auto& neighbor) {
    if (is_neighbor(boid_, *(neighbor.second), view_range, alpha, criterion))
      neighbors.push_back(neighbor.second);
  });
}


void update_neighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                      MyHashMap const& map, const double align_distance,
                      const double alpha, Criterion const criterion)
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
update_neighbors_testing(boid const& boid_, std::vector<boid const*>& neighbors,
                         MyHashMap const& map, const double align_distance,
                         const double alpha, Criterion const criterion)
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

void update_close_neighbors(boid const& boid_,
                            std::vector<boid const*>& close_neighbors,
                            std::vector<boid const*> const& neighbors,
                            const double repulsion_distance)
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

void regola1(boid const& boid_, DoubleVec& deltavel_,
             std::vector<boid const*> const& close_neighbors,
             const double repulsion_factor)
{
  std::for_each(close_neighbors.begin(), close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->pos() - boid_.pos();
                  deltavel_ += x * (-repulsion_factor);
                });
}

void regola2_3_old(boid const& boid_, DoubleVec& deltavel_,
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

void regola2_3(boid const& boid_, DoubleVec& deltavel_,
               std::vector<boid const*> const& neighbors,
               const double steering_factor, const double cohesion)
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

void posvel_update(BoidState& boid, ParamList const& params)
{
  // boid.get_neighbors().clear();
  // boid.get_close_neighbors().clear();
  boid.vel() += boid.deltavel();
  speed_adjust(boid.boid(), params.speedlimit, params.speedminimum);
  boid.pos() += (boid.vel()) * params.deltaT;
  bordercheck(boid.boid(), params.pixel, params.bordersize,
              params.border_repulsion, params.rate);
  boid.deltavel() = {0., 0.};
  update_id(boid.boid(), params.view_range);
}

void update_allneighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                         std::vector<boid const*>& close_neighbors,
                         MyHashMap const& map, const double repulsion_distance,
                         const double align_distance, const double alpha,
                         unsigned int size, unsigned int flocksize)
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
  // auto t1=high_resolution_clock::now();
  hashMap_.clear();
  std::for_each(set_.begin(), set_.end(), [&](auto& boid) {
    hashMap_.insert({boid.GridID(), &(boid.boid())});
  });
  /*auto t2=high_resolution_clock::now();
  duration<double, std::milli> ms_double=t2-t1;
  std::cout<<"Tempo creazione mappa: "<<ms_double.count()<<" ms"<<"\n";*/
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
      /*std::cout << "Numero vicini e molto " << boid.get_neighbors().size()
                << " e " << boid.get_close_neighbors().size() << "\n"
                << "___________\n\n";*/
    });
    std::for_each(policy, set_.begin(), set_.end(),
                  [&](auto& boid) { posvel_update(boid, params); });
  };
  std::visit(update_neighbors_rules, params.ExecPolicy);
  update_hashMap();
}

} // namespace boids
