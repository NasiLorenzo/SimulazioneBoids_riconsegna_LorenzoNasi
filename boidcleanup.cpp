#include "boidcleanup.hpp"

namespace boids {

std::bitset<8> FlagOpt;
double params::rate=1;
/*
void ensemble::update2(paramlist const& params)
{
  std::vector<double> vicini(set.size(), 0);
  std::vector<std::array<double, params::dim>> deltavec(set.size(), {0., 0.});
  std::vector<std::array<double, params::dim>> deltaveclose(set.size(),
                                                            {0., 0.});
  auto neighcountit   = vicini.begin();
  auto neighcountjt   = vicini.begin() + 1;
  auto deltavecit     = deltavec.begin();
  auto deltavecjt     = deltavec.begin() + 1;
  auto deltavecloseit = deltaveclose.begin();
  auto deltaveclosejt = deltaveclose.begin() + 1;
  for (auto it = set.begin(); it != set.end() - 1; ++it) {
    for (auto jt = it + 1; jt != set.end(); ++jt) {
      auto distanza = distance(*it, *jt);
      if (distanza <= params.neigh_align) {
        *neighcountit++;
        *neighcountjt++;
        auto x = jt->vel - it->vel;
        *deltavecit += params.steering * x; // regola 2
        *deltavecjt += -params.steering * x;
        auto y = jt->pos - it->pos;
        *deltavecit += params.coesione * x; // regola3
        *deltavecjt += -params.coesione * x;
        if (distanza <= params.neigh_repulsion) {
          *deltavecit += -params.repulsione * y; // regola1, da non riscalare
          *deltavecjt += params.repulsione * y;
        }
      }
      ++neighcountjt;
      ++deltavecjt;
      ++deltaveclosejt;
    }
    ++deltavecit;
    ++deltavecloseit;
    ++neighcountit;
  }
  neighcountit   = vicini.begin();
  deltavecit     = deltavec.begin();
  deltavecloseit = deltaveclose.begin();
  for (auto it = set.begin(); it != set.end(); ++it) {
    it->vel += *deltavecit / (*neighcountit) + *deltavecloseit;
    it->pos += params.deltaT * it->vel;
    ++neighcountit;
    ++deltavecit;
    ++deltavecloseit;
  }
}*/
} // namespace boids
