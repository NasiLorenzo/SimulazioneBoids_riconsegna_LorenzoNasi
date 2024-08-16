#include "boids.hpp"

namespace boids {

double params::rate{1};

void boidstate::random_boid(std::default_random_engine& eng,
                            paramlist const& params)
{
  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(this->vel_.begin(), this->vel_.end(),
                [&](double& x) { x = dist(eng); });
}

void boidstate::speedadjust(double speedlimit, double speedminimum)
{
  auto vmod = mod(this->vel_);
  if (vmod > speedlimit) {
    normalize(this->vel_);
    this->vel_ = speedlimit * this->vel_;
  }
  if (vmod < speedminimum) {
    normalize(this->vel_);
    this->vel_ = speedminimum * this->vel_;
  }
}
void boidstate::bordercheck(std::vector<unsigned int> const& pixel,
                            const double bordersize, const double attraction)
{
  auto pix = pixel.begin();
  for (auto index = this->pos_.begin(), velind = this->vel_.begin();
       index != this->pos_.end(); ++index, ++velind, ++pix) {
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
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->pos_, neighbor.get_pos());
    if (distanza < pow(align_distance, 2) && distanza != 0
        && (criterion == Criterion::any
            || (criterion == Criterion::similar
                && this->flockID == neighbor.get_ID()))) {
      auto cosangolo = cosangleij(neighbor.get_pos() - this->pos_, this->vel_);
      if ((cosangolo) >= std::cos(alpha)) {
        this->neighbors.emplace_back(&neighbor);
      }
    }
  });
}

void boidstate::update_close_neighbors(std::vector<boidstate const*> const& set,
                                       const double repulsion_distance)
{
  this->close_neighbors.clear();
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->pos_, neighbor->get_pos());
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      this->close_neighbors.emplace_back(neighbor);
    }
  });
}

void boidstate::update_close_neighbors(std::vector<boidstate> const& set,
                                       const double repulsion_distance)
{
  this->close_neighbors.clear();
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(this->pos_, neighbor.get_pos());
    if (distanza < pow(repulsion_distance, 2) && distanza != 0) {
      this->close_neighbors.emplace_back(&neighbor);
    }
  });
}

void boidstate::regola1(const double repulsione)
{
  std::for_each(this->close_neighbors.begin(), this->close_neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->get_pos() - this->pos_;
                  this->vel_ += -repulsione * x;
                });
}

void boidstate::regola2_3(const double steering, const double cohesion)
{
  auto n        = neighbors.size();
  auto velcopia = this->vel_;
  std::for_each(this->neighbors.begin(), this->neighbors.end(),
                [&](auto& neighbor) {
                  auto x = neighbor->get_vel() - velcopia;
                  this->vel_ += steering / static_cast<double>(n) * x;
                  auto y = neighbor->get_pos() - this->pos_;
                  this->vel_ += cohesion / static_cast<double>(n) * y;
                });
}

void boidstate::pos_update(const float deltaT)
{
  /*for(auto it=this->pos_.begin(),velit=this->vel_.begin();it!=this->pos_.end();++it,++velit){
  *it+=deltaT * (*velit);
  }*/
  this->pos_[0]+=this->vel_[0]*deltaT;
  this->pos_[1]+=this->vel_[1]*deltaT;

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
  //regola2_3(params.steering, params.coesione);
  //speedadjust(params.speedlimit, params.speedminimum);
  //bordercheck(params.pixel, params.bordersize, params.attraction);
  pos_update(params.deltaT);
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
  std::for_each(set.begin(), set.end(), [&](auto& boid) {
    boid.update_allneighbors(set, params.neigh_repulsion, params.neigh_align,
                             params.alpha, params.size, params.flocksize);
    //std::cout<<"il numero di vicini e molto vicini è "<<boid.get_neighbors().size()<<" e "<<boid.get_close_neighbors().size()<<"\n";
  });
  std::for_each(set.begin(), set.end(),
                [&](auto& boid) { boid.update_rules(params); });
}

} // namespace boids
