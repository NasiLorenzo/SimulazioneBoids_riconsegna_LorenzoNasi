#include "boids.hpp"
namespace boids {

double params::rate{1};
using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
auto random_boid(std::default_random_engine& eng, paramlist const& params)
{
  boidstate newboid{};
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(newboid.get_vel().begin(), newboid.get_vel().end(),
                [&](double& x) { x = dist(eng); });
  return newboid;
}

void UpdateID(boid& boid, const double view_range)
{
  boid.GridID.columns =
      static_cast<int>(std::floor(boid.pos_[0] / view_range) + 1);
  boid.GridID.rows =
      static_cast<int>(std::floor(boid.pos_[1] / view_range) + 1);
}

int hash_function(gridID const& GridID, const int columns)
{
  int key = (GridID.rows - 1) * columns + GridID.columns;
  return key;
}

void speedadjust(boid& boid, double speedlimit, double speedminimum)
{
  auto vmod = mod(boid.vel_);
  if (vmod > speedlimit) {
    normalize(boid.vel_);
    boid.vel_ = speedlimit * boid.vel_;
  }
  if (vmod < speedminimum) {
    normalize(boid.vel_);
    boid.vel_ = speedminimum * boid.vel_;
  };
}
void bordercheck(boid& boid, std::vector<unsigned int> const& pixel,
                 const double bordersize, const double attraction)
{
  auto pix = pixel.begin();
  for (auto index = boid.pos_.begin(), velind = boid.vel_.begin();
       index != boid.pos_.end(); ++index, ++velind, ++pix) {
    if (*index > params::rate * (*pix - bordersize)) {
      *velind -= attraction;
    } else if (*index < params::rate * bordersize) {
      *velind += attraction;
    }
  }
}

void update_neighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                      std::unordered_multimap<int, boid const*> const& map,
                      const double align_distance, const double alpha,
                      Criterion criterion, const int columns)
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
  gridID startID{};
  if (boid_.GridID.columns == 1)
    startID.columns = 1;
  else
    startID.columns = boid_.GridID.columns - 1;
  if (boid_.GridID.rows == 1)
    startID.rows = 1;
  else
    startID.rows = boid_.GridID.rows - 1;
  auto startkey = hash_function(startID, columns);
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      std::cout<<"La startkey vale: "<<startkey<<"\n";
      auto neighrange = map.equal_range(startkey);
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
            std::cout<<"La posizione  vicino vale: "<<neighbor.second->pos_[0]<<" e "<<neighbor.second->pos_[1]<<"\n";
          }
        }
      });
      startkey++;
    }
    startkey += columns - 3;
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
      // std::cout<<"Stato angolo: "<<bool{cosangleij(neighbor->pos_ -
      // this->boid_.pos_,
      //                               this->boid_.vel_)>=cos(0.55*M_PI)}<<"\n";
      // std::cout<<"Stato distanza: "<<bool{distanza < pow(repulsion_distance,
      // 2)}<<"\n"; std::cout<<"Valore distanza: "<<distanza<<"\n";
      close_neighbors.emplace_back(neighbor);
    }
  });
}

void update_close_neighbors(
    boid const& boid_, std::vector<boid const*>& close_neighbors,
    std::unordered_multimap<int, boid const*> const& map,
    const double repulsion_distance, const int columns, const double alpha)
{
  close_neighbors.clear();
  gridID startID{};
  if (boid_.GridID.columns == 1)
    startID.columns = 1;
  else
    startID.columns = boid_.GridID.columns - 1;
  if (boid_.GridID.rows == 1)
    startID.rows = 1;
  else
    startID.rows = boid_.GridID.rows - 1;
  auto startkey = hash_function(startID, columns);
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      // std::cout<<"Valore startkey: "<<startkey<<"\n";
      auto neighrange = map.equal_range(startkey);
      std::for_each(neighrange.first, neighrange.second, [&](auto& neighbor) {
        auto distanza = distance(boid_.pos_, neighbor.second->pos_);
        // std::cout<<"Stato distanza: "<<bool{distanza <
        // pow(repulsion_distance, 2)}<<"\n"; std::cout<<"Valore distanza:
        // "<<sqrt(distanza)<<"\n";
        if (distanza < pow(repulsion_distance, 2) && (distanza != 0)) {
          auto cosangolo =
              cosangleij(neighbor.second->pos_ - boid_.pos_, boid_.vel_);
          if ((cosangolo) >= std::cos(alpha)) {
            close_neighbors.emplace_back(neighbor.second);
          }
        }
      });
      startkey++;
    }
    startkey += columns - 3;
  }
}

void regola1(boid const& boid_, DoubleVec& deltavel_,
             std::vector<boid const*> const& close_neighbors,
             const double repulsione)
{
  std::for_each(close_neighbors.begin(), close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->pos_ - boid_.pos_;
                  deltavel_ += -repulsione * x;
                });
}

void regola2_3(boid const& boid_, DoubleVec& deltavel_,
               std::vector<boid const*> const& neighbors, const double steering,
               const double cohesion)
{
  auto n = neighbors.size();
  // auto velcopia = this->get_vel();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->vel_ - boid_.vel_;
    deltavel_ += steering / static_cast<double>(n) * x;
    auto y = neighbor->pos_ - boid_.pos_;
    deltavel_ += cohesion / static_cast<double>(n) * y;
  });
}

void posvel_update(boidstate& boid, paramlist const& params)
{
  //boid.get_neighbors().clear();
  //boid.get_close_neighbors().clear();
  boid.get_boid().vel_ += boid.get_deltavel();
  speedadjust(boid.get_boid(), params.speedlimit, params.speedminimum);
  boid.get_boid().pos_[0] += (boid.get_boid().vel_[0]) * params.deltaT;
  boid.get_boid().pos_[1] += (boid.get_boid().vel_[1]) * params.deltaT;
  bordercheck(boid.get_boid(), params.pixel, params.bordersize,
              params.attraction);
  boid.get_deltavel() = {0., 0.};
  UpdateID(boid.get_boid(), params.view_range);
}

void update_allneighbors(boid const& boid_, std::vector<boid const*>& neighbors,
                         std::vector<boid const*>& close_neighbors,
                         std::unordered_multimap<int, boid const*> const& map,
                         const double repulsion_distance,
                         const double align_distance, const double alpha,
                         unsigned int size, unsigned int flocksize,
                         const int columns)
{
  if (flocksize < size) {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::similar, columns);
    update_close_neighbors(boid_, close_neighbors, map, repulsion_distance,
                           columns, alpha);
  } else {
    update_neighbors(boid_, neighbors, map, align_distance, alpha,
                     Criterion::any, columns);
    update_close_neighbors(boid_, close_neighbors, neighbors,
                           repulsion_distance);
  }
}

void update_rules(boid const& boid_, DoubleVec& deltavel_,
                  std::vector<boid const*>& neighbors,
                  std::vector<boid const*>& close_neighbors,
                  paramlist const& params)
{
  regola1(boid_, deltavel_, close_neighbors, params.repulsione);
  regola2_3(boid_, deltavel_, neighbors, params.steering, params.coesione);
}

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params)
{
  std::vector<boidstate> set{};
  std::unordered_multimap<int, boid const*> HashMap{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    boidstate boidprova{random_boid(eng, params)};
    // boidprova.random_boid(eng, params);
    boidprova.set_ID() = i / params.flocksize;
    std::cout << "Il flock id vale: " << boidprova.cget_boid().flockID << "\n";
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

void flock::update_HashMap(paramlist const& params)
{
  // auto t1=high_resolution_clock::now();
  HashMap.clear();
  std::for_each(set.begin(), set.end(), [&](auto& boid) {
    HashMap.insert(
        {hash_function(boid.set_GridID(), params.columns), &(boid.set_boid())});
  });
  /*auto t2=high_resolution_clock::now();
  duration<double, std::milli> ms_double=t2-t1;
  std::cout<<"Tempo creazione mappa: "<<ms_double.count()<<" ms"<<"\n";*/
}

void flock::update(paramlist const& params)
{
  std::for_each(/*std::execution::par_unseq,*/ set.begin(), set.end(),
                [&](auto& boid) {
                  auto t1 = high_resolution_clock::now();
                  update_allneighbors(boid.cget_boid(), boid.get_neighbors(),
                                      boid.get_close_neighbors(), HashMap,
                                      params.repulsion_range, params.view_range,
                                      params.alpha, params.size,
                                      params.flocksize, params.columns);
                  auto t2 = high_resolution_clock::now();
                  duration<double, std::milli> ms_double = t2 - t1;
                  // std::cout<<"Tempo creazione vicini e operazioni:
                  // "<<ms_double.count()<<" ms"<<"\n";
                  update_rules(boid.cget_boid(), boid.get_deltavel(),
                                    boid.get_neighbors(),
                                    boid.get_close_neighbors(), params);
                  // std::cout<<"Chiave boid "<<boid.set_GridID().columns<<" e
                  //"<<boid.set_GridID().rows<<"\n";
                  /*std::cout << "La chiave vale: "
                            << hash_function(boid.set_GridID(), params.columns)
                            << "\n";*/
                  std::cout << "il numero di vicini e molto vicini è "
                            << boid.get_neighbors().size() << " e "
                            << boid.get_close_neighbors().size() << "\n"
                            << "----------" << "\n\n";
                });
  std::for_each(/*std::execution::par_unseq,*/ set.begin(), set.end(),
                [&](auto& boid) { posvel_update(boid, params); });
  update_HashMap(params);
}

} // namespace boids
