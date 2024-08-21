#include "boids.hpp"
namespace boids {

double params::rate{1};
using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
void boidstate::random_boid(std::default_random_engine& eng,
                            paramlist const& params)
{
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(boid_.vel_.begin(), boid_.vel_.end(),
                [&](double& x) { x = dist(eng); });
}

void UpdateID(boid& boid, const double view_range)
{
  boid.GridID.columns =
      static_cast<int>(std::floor(boid.pos_[0] / view_range) + 1);
  boid.GridID.rows =
      static_cast<int>(std::floor(boid.pos_[1] / view_range) + 1);
  // std::cout << "Valori ID in uscita: " << boid.GridID.columns << " " <<
  // boid.GridID.rows << "\n";
}

int hash_function(gridID const& GridID, const int columns)
{
  std::cout << "Valori ID: " << GridID.columns << " " << GridID.rows << "\n";
  int key = (GridID.rows - 1) * columns + GridID.columns;
  // assert(key>0);
  return key;
}

void create_neighbors_range(
    boidstate& boid_, std::unordered_multimap<int, boid const*> const& map,
    std::vector<boid const*>& neighbors, const double view_range,
    const int columns)
{
  neighbors.clear();
  gridID startID{};
  if (boid_.set_boid().GridID.columns == 1)
    startID.columns = 1;
  else
    startID.columns = boid_.set_GridID().columns - 1;
  if (boid_.set_boid().GridID.rows == 1)
    startID.rows = 1;
  else
    startID.rows = boid_.set_GridID().rows - 1;
  auto startkey = hash_function(startID, columns);
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      auto neighrange = map.equal_range(startkey);
      std::for_each(neighrange.first, neighrange.second,
                    [&](auto& neighbor) mutable {
                      neighbors.emplace_back(neighbor.second);
                    });
      startkey++;
    }
    startkey += columns - 3;
  }
}

void boidstate::speedadjust(double speedlimit, double speedminimum)
{
  auto vmod = mod(this->boid_.vel_);
  if (vmod > speedlimit) {
    normalize(this->boid_.vel_);
    this->boid_.vel_ = speedlimit * this->boid_.vel_;
  }
  if (vmod < speedminimum) {
    normalize(this->boid_.vel_);
    this->boid_.vel_ = speedminimum * this->boid_.vel_;
  }
}
void boidstate::bordercheck(std::vector<unsigned int> const& pixel,
                            const double bordersize, const double attraction)
{
  auto pix = pixel.begin();
  for (auto index = this->boid_.pos_.begin(), velind = this->boid_.vel_.begin();
       index != this->boid_.pos_.end(); ++index, ++velind, ++pix) {
    if (*index > params::rate * (*pix - bordersize)) {
      *velind -= attraction;
    } else if (*index < params::rate * bordersize) {
      *velind += attraction;
    }
  }
}

void boidstate::update_neighbors(
    std::unordered_multimap<int, boid const*> const& map,
    const double align_distance, const double alpha, Criterion criterion,
    const int columns)
{
  neighbors.clear();
  assert(neighbors.empty());
  /*this->neighbors.clear();
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
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
  else {
    startID.columns = boid_.GridID.columns - 1;
  }
  if (boid_.GridID.rows == 1)
    startID.rows = 1;
  else {
    startID.rows = boid_.GridID.rows - 1;
  }
  auto startkey = hash_function(startID, columns);
  //std::cout << "Valore startkey " << startkey << "\n";
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      auto neighrange = map.equal_range(startkey);
      std::for_each(neighrange.first, neighrange.second, [&](auto& neighbor) {
        auto distanza = distance(this->boid_.pos_, neighbor.second->pos_);
        if ((distanza < pow(align_distance, 2) && distanza != 0
             && (criterion == Criterion::any
                 || (criterion == Criterion::similar
                     && this->boid_.flockID == neighbor.second->flockID)))) {
          auto cosangolo = cosangleij(neighbor.second->pos_ - this->boid_.pos_,
                                      this->boid_.vel_);
          if ((cosangolo) >= std::cos(alpha)) {
            this->neighbors.emplace_back(neighbor.second);
          }
        }
      });
      startkey++;
    }
    startkey += columns - 3;
  }
  /*neighbors.erase(
      std::remove_if(
          std::execution::par_unseq, neighbors.begin(), neighbors.end(),
          [&](boid const*& neighbor) {
            auto cosangolo =
                cosangleij(neighbor->pos_ - this->boid_.pos_, this->boid_.vel_);
            auto distanza = distance(this->boid_.pos_, neighbor->pos_);
            return ((distanza >= pow(align_distance, 2))
                    || (this->boid_.flockID != neighbor->flockID)
                    || (cosangolo < std::cos(alpha)) || distanza == 0);
          }),
      neighbors.end());*/
}

void boidstate::update_close_neighbors(std::vector<boid const*> const& set,
                                       const double repulsion_distance)
{
  this->close_neighbors.clear();
  assert(close_neighbors.empty());
  std::for_each(/*std::execution::par_unseq,*/ set.begin(), set.end(),
                [&](auto& neighbor) {
                  auto distanza = distance(this->boid_.pos_, neighbor->pos_);
                  if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
                    // std::lock_guard<std::mutex> guard(neighs_mutex);
                    this->close_neighbors.emplace_back(neighbor);
                  }
                });
}

void boidstate::update_close_neighbors(
    std::unordered_multimap<int, boid const*> const& map,
    const double repulsion_distance, const double align_distance,
    const int columns, const double alpha)
{
  close_neighbors.clear();
  assert(close_neighbors.empty());
  // create_neighbors_range(*this, map, close_neighbors, align_distance,
  // columns);
  /*std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->boid_.pos_, neighbor.second.get_pos());
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      this->close_neighbors.emplace_back(&(neighbor.second.boid_));
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
      auto neighrange = map.equal_range(startkey);
      std::for_each(neighrange.first, neighrange.second, [&](auto& neighbor) {
        auto distanza = distance(this->boid_.pos_, neighbor.second->pos_);
        if ((distanza < pow(repulsion_distance, 2) && distanza != 0
             && this->boid_.flockID == neighbor.second->flockID)) {
          auto cosangolo = cosangleij(neighbor.second->pos_ - this->boid_.pos_,
                                      this->boid_.vel_);
          if ((cosangolo) >= std::cos(alpha)) {
            this->close_neighbors.emplace_back(neighbor.second);
          }
        }
      });
      startkey++;
    }
    startkey += columns - 3;
  }

  /*close_neighbors.erase(
      std::remove_if(std::execution::par_unseq,
          close_neighbors.begin(), close_neighbors.end(),
          [&](auto& neighbor) {
            auto cosangolo =
                cosangleij(neighbor->pos_ - this->boid_.pos_, this->boid_.vel_);
            auto distanza = distance(this->boid_.pos_, neighbor->pos_);
            return ((distanza >= pow(align_distance, 2))
                    || (this->boid_.flockID != neighbor->flockID)
                    || (cosangolo < std::cos(alpha)) || distanza == 0);
          }),
      close_neighbors.end());*/
}

void boidstate::regola1(const double repulsione)
{
  std::for_each(this->close_neighbors.begin(), this->close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->get_poscopy() - this->boid_.get_poscopy();
                  // auto x{DoubleVec{1,1}};
                  this->boid_.deltavel_ += -repulsione * x;
                });
}

void boidstate::regola2_3(const double steering, const double cohesion)
{
  auto n        = neighbors.size();
  auto velcopia = this->get_vel();
  std::for_each(
      this->neighbors.begin(), this->neighbors.end(), [&](auto& neighbor) {
        auto x = neighbor->vel_ - velcopia;
        this->boid_.deltavel_ += steering / static_cast<double>(n) * x;
        auto y = neighbor->pos_ - this->boid_.pos_;
        this->boid_.deltavel_ += cohesion / static_cast<double>(n) * y;
      });
}

void boidstate::posvel_update(const float deltaT, const double view_range)
{
  boid_.vel_ += boid_.deltavel_;
  boid_.pos_[0] += (this->cget_vel()[0]) * deltaT;
  boid_.pos_[1] += (this->cget_vel()[1]) * deltaT;
  boid_.deltavel_ = {0., 0.};
  UpdateID(boid_, view_range);
  //std::cout << "La posizione è " << get_pos()[0] << " e " << get_pos()[1]         << "\n";
}

void boidstate::update_allneighbors(
    std::unordered_multimap<int, boid const*> const& map,
    const double repulsion_distance, const double align_distance,
    const double alpha, unsigned int size, unsigned int flocksize,
    const int columns)
{
  if (flocksize < size) {
    update_neighbors(map, align_distance, alpha, Criterion::similar, columns);
    update_close_neighbors(map, repulsion_distance, align_distance, columns,
                           alpha);
  } else {
    update_neighbors(map, align_distance, alpha, Criterion::any, columns);
    update_close_neighbors(this->neighbors, repulsion_distance);
  }
}

void boidstate::update_rules(paramlist const& params)
{
  regola1(params.repulsione);
  regola2_3(params.steering, params.coesione);
  speedadjust(params.speedlimit, params.speedminimum);
  // bordercheck(params.pixel, params.bordersize, params.attraction);
}

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params)
{
  std::vector<boidstate> set{};
  std::unordered_multimap<int, boid const*> HashMap{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    boidstate boidprova{};
    boidprova.random_boid(eng, params);
    boidprova.set_ID() = i / params.flocksize;
    UpdateID(boidprova.set_boid(), params.neigh_align);
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
  assert(HashMap.empty());
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
  std::for_each(
      /*std::execution::par_unseq,*/ set.begin(), set.end(), [&](auto& boid) {
        auto t1 = high_resolution_clock::now();
        boid.update_allneighbors(HashMap, params.neigh_repulsion,
                                 params.neigh_align, params.alpha, params.size,
                                 params.flocksize, params.columns);
        auto t2                                = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        // std::cout<<"Tempo creazione vicini e operazioni:
        // "<<ms_double.count()<<" ms"<<"\n";
        boid.update_rules(params);
        // std::cout<<"Chiave boid "<<boid.set_GridID().columns<<" e
        // "<<boid.set_GridID().rows<<"\n";

        // std::cout<<"il numero di vicini e molto vicini è
        // "<<boid.get_neighbors().size()<<" e
        // "<<boid.get_close_neighbors().size()<<"\n";
        std::cout << "Posizione e ID " << boid.get_pos()[0] << " "
                  << boid.get_pos()[1] << " " << boid.set_GridID().columns << " "
                  << boid.set_GridID().rows << "\n";
      });
  std::for_each(/*std::execution::par_unseq,*/ set.begin(), set.end(),
                [&](auto& boid) {
                  boid.posvel_update(params.deltaT, params.neigh_align);
                });
  update_HashMap(params);
}

} // namespace boids
