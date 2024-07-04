#ifndef BOIDS_HPP
#define BOIDS_HPP
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
  unsigned int flocksize;
  std::vector<unsigned int> pixel;
  double bordersize;
  double sigma;

  paramlist()
      : pixel(params::dim)
  {}
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
  static auto generate(std::default_random_engine&, paramlist const& params);

  static auto generator(std::default_random_engine& eng,
                        paramlist const& params);

  static void speedadjust(boidtype& boid, const double speedlimit,
                          const double speedminimum);
  template<Criterion criterion>
  static auto neighbors(std::vector<boidtype> const& set, boidtype const& boid,
                        const double d, const double alpha);

  static auto neighbors(std::vector<boidtype const*> const& set,
                        boidtype const& boid, const double d);

  static void regola1(std::vector<boidtype const*>& neighbors, boidtype& boid,
                      const double repulsione);

  static void regola2_3(std::vector<boidtype const*>& neighbors,
                        boidtype& oldboid, boidtype& boid,
                        const double steering, const double cohesion);
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

  void update(paramlist const& params);
};
} // namespace boids
#include "boids.tpp" // namespace boids
#endif