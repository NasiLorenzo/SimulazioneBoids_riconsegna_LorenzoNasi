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

void boidstate::update_neighbors(std::vector<boidstate> const& set,
                                 const double align_distance,
                                 const double alpha, Criterion criterion)
{
  this->neighbors.clear();
  std::for_each(std::execution::par_unseq,set.begin(), set.end(), [&](auto& neighbor) {
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
  });
}

void boidstate::update_close_neighbors(std::vector<boid const*> const& set,
                                       const double repulsion_distance)
{
  this->close_neighbors.clear();
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->boid_.pos_, neighbor->pos_);
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      this->close_neighbors.emplace_back(neighbor);
    }
  });
}

void boidstate::update_close_neighbors(std::vector<boidstate> const& set,
                                       const double repulsion_distance)
{
  this->close_neighbors.clear();
  std::for_each(std::execution::par_unseq,set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->boid_.pos_, neighbor.get_pos());
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      this->close_neighbors.emplace_back(&(neighbor.boid_));
    }
  });
}

void boidstate::regola1(const double repulsione)
{
  std::for_each(this->close_neighbors.begin(), this->close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->pos_ - this->boid_.pos_;
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

void boidstate::posvel_update(const float deltaT)
{
  boid_.vel_ += boid_.deltavel_;
  boid_.pos_[0] += (this->get_vel()[0]) * deltaT;
  this->boid_.pos_[1] += (this->get_vel()[1]) * deltaT;
  boid_.deltavel_ = {0., 0.};
}

void boidstate::update_allneighbors(std::vector<boidstate> const& set,
                                    const double repulsion_distance,
                                    const double align_distance,
                                    const double alpha, unsigned int size,
                                    unsigned int flocksize)
{
  if (flocksize < size) {
    update_neighbors(set, align_distance, alpha, Criterion::similar);
    update_close_neighbors(set, repulsion_distance);
  } else {
    update_neighbors(set, align_distance, alpha, Criterion::any);
    update_close_neighbors(this->neighbors, repulsion_distance);
  }
}

void boidstate::update_rules(paramlist const& params)
{
  regola1(params.repulsione);
  regola2_3(params.steering, params.coesione);
  speedadjust(params.speedlimit, params.speedminimum);
  bordercheck(params.pixel, params.bordersize, params.attraction);
}

std::vector<boidstate> generate_flock(std::default_random_engine& eng,
                                      paramlist const& params)
{
  std::vector<boidstate> set{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    boidstate boidprova{};
    boidprova.random_boid(eng, params);
    boidprova.set_ID() = i / params.flocksize;
    for (auto it = boidprova.set_pos().begin(); it != boidprova.set_pos().end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          0, static_cast<double>(*pix * params::rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }

  return set;
}

void flock::update(paramlist const& params)
{
  auto t1 = high_resolution_clock::now();
  std::for_each(oneapi::dpl::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) {
                  boid.update_allneighbors(set, params.neigh_repulsion,
                                           params.neigh_align, params.alpha,
                                           params.size, params.flocksize);
                  boid.update_rules(params);

                  // std::cout<<"il numero di vicini e molto vicini è
                  // "<<boid.get_neighbors().size()<<" e
                  // "<<boid.get_close_neighbors().size()<<"\n";
                });
  auto t2 = high_resolution_clock::now();
  duration<double, std::milli> ms_double = t2 - t1;
  std::cout << "tempo creazione vicini: " << ms_double.count() << " ms\n";
  std::for_each(std::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) { boid.posvel_update(params.deltaT); });
}

} // namespace boids
