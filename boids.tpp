#ifndef BOIDS_TPP
#define BOIDS_TPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
namespace boids {

template<class boidtype>
auto functions<boidtype>::generate(std::default_random_engine& eng,
                                   paramlist const& params)
{ // genera pos e vel di un boid distribuiti secondo
  // una gauss centrata in 0
  boidtype boid{};
  // std::array<double,2> Uniform2D {std::uniform_real_distribution<double>
  // dis(0, static_cast<double>(pixe * params::rate));};

  std::normal_distribution<double> dist(0.0, params.sigma);
  std::for_each(boid.vel.begin(), boid.vel.end(),
                [&](double& x) { x = dist(eng); });
  return boid;
}
template<class boidtype>
auto functions<boidtype>::generator(std::default_random_engine& eng,
                                    paramlist const& params)
{
  std::vector<boidtype> set{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin(); // puntatore ai pixel
    boidtype boidprova{generate(eng, params)};
    boidprova.flockID = i / params.flocksize;
    for (auto it = boidprova.pos.begin(); it != boidprova.pos.end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          0, static_cast<double>(*pix * params::rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }

  return set;
}
template<class boidtype>
void functions<boidtype>::speedadjust(boidtype& boid, const double speedlimit,
                                      const double speedminimum)
{
  auto vnorm = mod(boid.vel);
  if (vnorm > speedlimit) {
    normalize(boid.vel);
    boid.vel = speedlimit * boid.vel;
  }
  if (vnorm < speedminimum) {
    normalize(boid.vel);
    boid.vel = speedminimum * boid.vel;
  }
}
template<class boidtype>
void bordercheck_posupdate(boidtype& boid, std::vector<unsigned int> const& pixel,
                 const double bordersize, const double attraction, const float deltaT)
{
  auto pix = pixel.begin();
  for (auto index = boid.pos.begin(), velind = boid.vel.begin();
       index != boid.pos.end(); ++index, ++velind, ++pix) {
    if (*index > params::rate * (*pix - bordersize)) {
      *velind -= attraction;
    } else if (*index < params::rate * bordersize) {
      *velind += attraction;
    }
    *index+=*velind*deltaT;
  }
}

template<class boidtype>
template<Criterion criterion>
auto functions<boidtype>::neighbors(std::vector<boidtype> const& set,
                                    boidtype const& boid, const double d,
                                    const double alpha)
{
  std::vector<boidtype const*> neighbors{};
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(boid.pos, neighbor.pos);
    if (distanza < pow(d, 2) && distanza != 0
        && (criterion == Criterion::any
            || (criterion == Criterion::similar
                && boid.flockID == neighbor.flockID))) {
      auto cosangolo = cosangleij(neighbor.pos - boid.pos, boid.vel);
      if ((cosangolo) >= std::cos(alpha)) {
        neighbors.emplace_back(&neighbor);
      }
    }
  });
  return neighbors;
}
template<class boidtype>
auto functions<boidtype>::neighbors(std::vector<boidtype const*> const& set,
                                    boidtype const& boid, const double d)
{
  std::vector<boidtype const*> neighbors{};
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    auto distanza = distance(boid.pos, neighbor->pos);
    if (distanza < pow(d, 2) && distanza != 0) {
      neighbors.emplace_back(neighbor);
    }
  });
  return neighbors;
}
template<class boidtype>
void functions<boidtype>::regola1(std::vector<boidtype const*>& neighbors,
                                  boidtype& boid, const double repulsione)
{
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->pos - boid.pos;
    boid.vel += -repulsione * x;
  });
}

template<class boidtype>
void functions<boidtype>::regola2_3(std::vector<boidtype const*>& neighbors,
                                    boidtype const& oldboid, boidtype& boid,
                                    const double steering,
                                    const double cohesion)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->vel - oldboid.vel;
    boid.vel += steering / static_cast<double>(n) * x;
    auto y = neighbor->pos - boid.pos;
    boid.vel += cohesion / static_cast<double>(n) * y;
  });
}

template<class boidtype>
void ensemble<boidtype>::update(paramlist const& params)
{
  auto jt = newset.begin();
  std::for_each(set.begin(), set.end(), [&](auto& it) mutable {
    std::vector<boidtype const*> neighbor;
    std::vector<boidtype const*> close_neighbor;

    if (params.flocksize < params.size) {
      neighbor =
          boids::functions<boidtype>::template neighbors<Criterion::similar>(
              set, *jt, params.neigh_align, params.alpha);
      close_neighbor = functions<boidtype>::template neighbors<Criterion::any>(
          set, *jt, params.neigh_repulsion, params.alpha);
    } else {
      neighbor = boids::functions<boidtype>::template neighbors<Criterion::any>(
          set, *jt, params.neigh_align, params.alpha);
      close_neighbor =
          functions<boidtype>::neighbors(neighbor, *jt, params.neigh_repulsion);
    }

    functions<boidtype>::regola1(close_neighbor, *jt, params.repulsione);
    functions<boidtype>::regola2_3(neighbor, it, *jt, params.steering,
                                   params.coesione);
    functions<boidtype>::speedadjust(*jt, params.speedlimit,
                                     params.speedminimum);
    bordercheck_posupdate(*jt,params.pixel,params.bordersize,params.attraction,params.deltaT);
    ++jt;
  });

  // Update set to newset
  set = newset;
}

} // namespace boids
#endif