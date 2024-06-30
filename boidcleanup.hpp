#ifndef BOIDCLEANUP_HPP
#define BOIDCLEANUP_HPP
#include "doublevec.hpp"
namespace boids {

enum class Criterion : bool
{
  any     = 1,
  similar = 0,
};

struct paramlist
{
  double repulsione;
  double steering;
  double coesione;
  double neigh_align;
  double neigh_repulsion;
  double attraction;
  double alpha;
  double speedlimit;
  double speedminimum;
  float deltaT;
  unsigned int size;
  unsigned int flocknumber;
  std::vector<unsigned int> pixel{1010, 710};
};
struct boidstate
{
  DoubleVec pos;
  DoubleVec vel;
  unsigned int flockID{0};
};

template<typename Iterator, typename predicate, typename operation>
void // template argument deduction
for_each_if(Iterator begin, Iterator end, predicate p, operation op)
{
  for (; begin != end; begin++) {
    if (p()) {
      op(*begin);
    } else {
      break;
    }
  }
}

template<class boidtype>
struct functions
{
  static boidtype generate(std::default_random_engine& eng)
  { // genera pos e vel di un boid distribuiti secondo
    // una gauss centrata in 0
    boidtype boid{};
    // std::array<double,2> Uniform2D {std::uniform_real_distribution<double>
    // dis(0, static_cast<double>(pixe * params::rate));};

    std::normal_distribution<double> dist(0.0, params::sigma);
    std::for_each(boid.pos.begin(), boid.pos.end(),
                  [&](double& x) { x = dist(eng); });
    std::for_each(boid.vel.begin(), boid.vel.end(),
                  [&](double& x) { x = params::vel_factor * dist(eng); });
    return boid;
  }

  static std::vector<boidtype> generator(std::default_random_engine& eng,
                                         paramlist const& params)
  {
    std::vector<boidtype> set{};
    for (unsigned int i = 0; i < params.size; i++) {
      auto pix = params.pixel.begin(); // puntatore ai pixel
      boidtype boidprova{generate(eng)};
      boidprova.flockID = i / params.flocknumber;
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

  static void speedadjust(boidtype& boid, const double speedlimit,
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
  template<Criterion criterion>
  static auto neighbors(std::vector<boidtype> const& set, boidtype const& boid,
                        const double d, const double alpha)
  {
    std::vector<boidtype const*> neighbors{};
    int i = 0;
    std::for_each(set.begin(), set.end(), /*[&i]() { return i < 10; },*/
                  [&](auto& neighbor) {
                    auto distanza = distance(boid.pos, neighbor.pos);
                    if (distanza < pow(d, 2) && distanza != 0
                        && (criterion == Criterion::any
                            || (criterion == Criterion::similar
                                && boid.flockID == neighbor.flockID))) {
                      DoubleVec deltax = neighbor.pos - boid.pos;
                      DoubleVec y      = boid.vel;
                      if (mod(boid.vel) != 0)
                        deltax = normalize(deltax);
                      y                  = normalize(y);
                      double prodscalare = std::inner_product(
                          deltax.begin(), deltax.end(), y.begin(), 0.);
                      if ((prodscalare) >= std::cos(alpha)) {
                        neighbors.emplace_back(&neighbor);
                        ++i;
                      }
                    }
                  });
    return neighbors;
  }

  static auto neighbors(std::vector<boidtype const*> const& set,
                        boidtype const& boid, const double d,
                        const double alpha)
  {
    std::vector<boidtype const*> neighbors{};
    int i = 0;
    std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
      // if (i < 10) {
      if (distance(boid.pos, neighbor->pos) < pow(d, 2)
          && distance(boid.pos, neighbor->pos) != 0) {
        DoubleVec deltax = neighbor->pos - boid.pos;
        DoubleVec y      = boid.vel;
        if (mod(boid.vel) != 0)
          deltax = normalize(deltax);
        y = normalize(y);
        double prodscalare =
            std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
        if ((prodscalare) >= std::cos(alpha)) {
          neighbors.emplace_back(neighbor);
          i++;
        }
      }
      //}
    });
    return neighbors;
  }

  static void regola1(std::vector<boidtype const*>& neighbors, boidtype& boid,
                      const double repulsione)
  {
    std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
      auto x = neighbor->pos - boid.pos;
      boid.vel += -repulsione * x;
    });
  }

  static void regola2_3(std::vector<boidtype const*>& neighbors,
                        boidtype& oldboid, boidtype& boid,
                        const double steering, const double cohesion)
  {
    auto n = neighbors.size();
    std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
      auto x = neighbor->vel - oldboid.vel;
      boid.vel += steering / n * x;
      auto y = neighbor->pos - boid.pos;
      boid.vel += cohesion / n * y;
    });
  }

  static auto
  meanvel(std::vector<boidtype> const& set) // Velocità quadratica media
  {
    double s{};
    for (auto it = set.begin(); it != set.end(); ++it) {
      s += boids::mod(it->vel);
    }
    return sqrt(s) / static_cast<double>(set.size());
  }

  static auto
  compx(std::vector<boidtype> const& set) // Media delle componenti x di vel
  {
    double s{};
    for (auto it = set.begin(); it != set.end(); ++it) {
      s += (*it).vel[0];
    }

    return s / static_cast<double>(set.size());
  }

  static auto
  compy(std::vector<boidtype> const& set) // Media delle componenti y di vel
  {
    double s{};
    for (auto it = set.begin(); it != set.end(); ++it) {
      s += (*it).vel[1];
    }

    return s / static_cast<double>(set.size());
  }
};

template<class boidtype>
class ensemble
{
  std::vector<boidtype> set;
  std::vector<boidtype> newset{set};

 public:
  ensemble(std::vector<boidtype>& old)
      : set{old}
  {}
  std::vector<boidtype>& set_()
  {
    return set;
  }

  std::vector<boidtype>& newset_()
  {
    return newset;
  }

  std::size_t size_()
  {
    return set.size();
  }

  void update(paramlist const& params)
  {
    for (auto it = set.begin(), jt = newset.begin(); it != set.end();
         ++it, ++jt) {
      auto neighbor{
          boids::functions<boidtype>::template neighbors<Criterion::similar>(
              set, *jt, params.neigh_align, params.alpha)};
      auto close_neighbor{
          functions<boidtype>::template neighbors<Criterion::any>(
              set, *jt, params.neigh_repulsion, params.alpha)};
      functions<boidtype>::regola1(close_neighbor, *jt, params.repulsione);
      functions<boidtype>::regola2_3(neighbor, *it, *jt, params.steering,
                                     params.coesione);
      functions<boidtype>::speedadjust(*jt, params.speedlimit,
                                       params.speedminimum);
      auto pix = params.pixel.begin();
      for (auto index = jt->pos.begin(), velind = jt->vel.begin();
           index != (*jt).pos.end(); ++index, ++velind, ++pix) {
        if (*index > params::rate * (*pix - 100)) {
          *velind -= params.attraction;
        } else {
          if (*index < params::rate * 100) {
            *velind += params.attraction;
          }
        }
        (*index) += (*velind) * params.deltaT;
      }
    }
    set = newset;
  }
};
} // namespace boids

#endif