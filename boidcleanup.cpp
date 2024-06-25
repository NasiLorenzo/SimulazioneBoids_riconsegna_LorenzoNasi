#include "boidcleanup.hpp"

namespace boids {

std::bitset<8> FlagOpt;
boidstate generate(std::default_random_engine& eng)
{ // genera pos e vel di un boid distribuiti secondo
  // una gauss centrata in 0
  boidstate boid{};
  // std::array<double,2> Uniform2D {std::uniform_real_distribution<double>
  // dis(0, static_cast<double>(pixe * params::rate));};

  std::normal_distribution<double> dist(0.0, params::sigma);
  std::for_each(boid.pos.begin(), boid.pos.end(),
                [&](double& x) { x = dist(eng); });
  std::for_each(boid.vel.begin(), boid.vel.end(),
                [&](double& x) { x = params::vel_factor * dist(eng); });
  return boid;
}

std::vector<RGB> generatecolors(std::default_random_engine& eng,
                                paramlist const& params)
{
  std::vector<RGB> colorvec{};
  for (int i = 0; i < params.size / params.flocknumber + 1; i++) {
    RGB color{};
    std::uniform_int_distribution dist(0, 255);
    color.red   = dist(eng);
    color.blue  = dist(eng);
    color.green = dist(eng);
    colorvec.push_back(color);
  }
  return colorvec;
};

auto mod_vel(boidstate const& boid) // Velocità singolo boid
{
  double sum{};
  for (auto it = boid.vel.begin(); it != boid.vel.end(); ++it) {
    sum += pow(*it, 2);
  }
  return sum;
}

double distance2(boidstate const& a, boidstate const& b)
{
  return pow(a.pos[0] - b.pos[0], 2) + pow(a.pos[1] - b.pos[1], 2);
}

double mod(std::array<double, params::dim> const& vec)
{
  return sqrt(
      std::accumulate(vec.begin(), vec.end(), 0,
                      [](double sum, double x) { return sum = sum + x * x; }));
}

std::array<double, params::dim> normalize(std::array<double, params::dim>& vec)
{
  auto modulo = boids::mod(vec);
  if (modulo == 0)
    modulo = 1.;
  return vec / modulo;
}
double distance(boidstate const& a, boidstate const& b)
{
  return std::transform_reduce(
      a.pos.begin(), a.pos.end(), b.pos.begin(), 0, std::plus<>(),
      [](double a, double b) { return pow(a - b, 2); });
}

std::array<double, params::dim>
operator+=(std::array<double, params::dim>& a,
           std::array<double, params::dim> const& b)
{
  std::transform(a.begin(), a.end(), b.begin(), a.begin(),
                 [](double a, double b) { return a + b; });
  return a;
}

std::array<double, params::dim>
operator+(const std::array<double, params::dim>& a,
          const std::array<double, params::dim>& b)
{
  std::array<double, params::dim> result{};
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [&](double c, double d) { return c + d; });
  return result;
}

std::array<double, params::dim>
operator-(std::array<double, params::dim> const& a,
          std::array<double, params::dim> const& b)
{
  std::array<double, params::dim> result;
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [](double c, double d) { return c - d; });
  return result;
}

std::array<double, params::dim> operator*(const double a,
                                          std::array<double, params::dim>& b)
{
  std::for_each(b.begin(), b.end(), [a](double& x) { x = a * x; });
  return b;
}

std::array<double, params::dim> operator/(double a,
                                          std::array<double, params::dim>& b)
{
  std::for_each(b.begin(), b.end(), [&a](double x) { return a / x; });
  return b;
}

std::array<double, params::dim> operator/(std::array<double, params::dim>& b,
                                          double a)
{
  std::for_each(b.begin(), b.end(), [&a](double& x) { x = x / a; });
  return b;
}

stormo generator(std::default_random_engine& eng, paramlist const& params)
{
  stormo set{};
  auto colorvec = generatecolors(eng, params);
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = pixel.begin(); // puntatore ai pixel
    boidstate boidprova{generate(eng)};
    boidprova.flockID = i / params.flocknumber;
    boidprova.arrow.setFillColor(sf::Color(colorvec[boidprova.flockID].red,
                                           colorvec[boidprova.flockID].green,
                                           colorvec[boidprova.flockID].blue));
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

void speedadjust(boidstate& boid, const double speedlimit,
                 const double speedminimum)
{
  auto vnorm = boids::mod(boid.vel);
  if (vnorm > speedlimit) {
    normalize(boid.vel);
    boid.vel = speedlimit * boid.vel;
  }
  if (vnorm < speedminimum) {
    normalize(boid.vel);
    boid.vel = speedminimum * boid.vel;
  }
}
template<bool val>
auto neighbors(stormo const& set, boidstate const& boid, const double d,
               const double alpha)
{
  std::vector<boidstate const*> neighbors{};
  int i = 0;
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    if (i < 10) {
      if (distance2(boid, neighbor) < pow(d, 2)
          && distance2(boid, neighbor) != 0
          && (val == 1 || (val == 0 && boid.flockID == neighbor.flockID))) {
        std::array<double, params::dim> deltax = neighbor.pos - boid.pos;
        std::array<double, params::dim> y      = boid.vel;
        if (boids::mod(boid.vel) != 0)
          deltax = normalize(deltax);
        y = normalize(y);
        double prodscalare =
            std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
        if ((prodscalare) >= std::cos(alpha)) {
          neighbors.emplace_back(&neighbor);
          i++;
        }
      }
    }
  });
  return neighbors;
}

auto neighbors(std::vector<boidstate const*> const& set, boidstate const& boid,
               const double d, const double alpha)
{
  std::vector<boidstate const*> neighbors{};
  int i = 0;
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    if (i < 10) {
      if (distance2(boid, *neighbor) < pow(d, 2)
          && distance2(boid, *neighbor) != 0) {
        std::array<double, params::dim> deltax = neighbor->pos - boid.pos;
        std::array<double, params::dim> y      = boid.vel;
        if (boids::mod(boid.vel) != 0)
          deltax = normalize(deltax);
        y = normalize(y);
        double prodscalare =
            std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
        if ((prodscalare) >= std::cos(alpha)) {
          neighbors.emplace_back(neighbor);
          i++;
        }
      }
    }
  });
  return neighbors;
}

void regola1(std::vector<boidstate const*>& neighbors, boidstate& boid,
             const double repulsione)
{
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->pos - boid.pos;
    boid.vel += -repulsione * (x);
  });
}

void regola2(std::vector<boidstate const*>& neighbors, boidstate& oldboid,
             boidstate& boid, const double steering)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->vel - oldboid.vel;
    boid.vel += steering / n * (x);
  });
}

void regola3(std::vector<boidstate const*>& neighbors, boidstate& boid,
             const double cohesion)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->pos - boid.pos;
    boid.vel += cohesion / n * (x);
  });
}

auto meanvel(stormo const& set) // Velocità quadratica media
{
  double s{};
  for (auto it = set.begin(); it != set.end(); ++it) {
    s += pow((*it).vel[0], 2) + pow((*it).vel[1], 2);
  }
  return sqrt(s) / static_cast<double>(set.size());
}

auto compx(stormo const& set) // Media delle componenti x di vel
{
  double s{};
  for (auto it = set.begin(); it != set.end(); ++it) {
    s += (*it).vel[0];
  }

  return s / static_cast<double>(set.size());
}

auto compy(stormo const& set) // Media delle componenti y di vel
{
  double s{};
  for (auto it = set.begin(); it != set.end(); ++it) {
    s += (*it).vel[1];
  }

  return s / static_cast<double>(set.size());
}

auto rotate(boidstate& boid, const double angle)
{
  auto mod{mod_vel(boid)};
  auto alpha = acos(boid.vel[0] / mod);
  std::cout << "alpha " << boid.vel[0] << "\n";
  /*auto index=trig.begin();
  for(auto it=boid.vel.begin();it!=boid.vel.end();++it, ++index){
    *it = mod * (*index)(alpha-angle);
  }*/
  boid.vel[0] = mod * cos(alpha - angle);
  boid.vel[1] = mod * sin(alpha - angle);

  return boid;
}

stormo& ensemble::set_()
{
  return set;
}

stormo& ensemble::newset_()
{
  return newset;
}

std::size_t ensemble::size_()
{
  return set.size();
}

double angle(boidstate const& boid)
{
  return atan2(boid.vel[1], boid.vel[0]);
}

void ensemble::update(paramlist const& params)
{
  for (auto it = set.begin(), jt = newset.begin(); it != set.end();
       ++it, ++jt) {
    auto neighbor{neighbors<0>(set, *it, params.neigh_align, params.alpha)};
    auto close_neighbor{
        neighbors<1>(set, *it, params.neigh_repulsion, params.alpha)};
    regola1(close_neighbor, *jt, params.repulsione);
    regola2(neighbor, *it, *jt, params.steering);
    regola3(neighbor, *jt, params.coesione);
    speedadjust(*jt, params.speedlimit, params.speedminimum);
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params.deltaT;
      if (*index > *pix - 150) {
        *velind -= params.attraction;
      } else {
        if (*index < 150) {
          *velind += params.attraction;
        }
      }
    }
  }
  set = newset;
}
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
}
} // namespace boids
