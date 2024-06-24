#include "boidcleanup.hpp"

namespace boids {

double paramms::repulsione   = 0.7;
double paramms::steering     = 0.1;
double paramms::coesione     = 100;
double paramms::neigh2       = 20;
double paramms::neigh_align  = 70;
double paramms::mod_align    = 0.000003;
double paramms::attraction   = 0.02;
double paramms::alpha        = M_PI;
double paramms::speedlimit   = 80;
double paramms::speedminimum = 2;
/*struct uniform2D{
  std::uniform_real_distribution<double> disX(0, static_cast<double>(pixel *
params::rate)); std::uniform_real_distribution<double> disY;

};*/

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

auto mod_vel(boidstate const& boid) // Velocità singolo boid
{
  double sum{};
  for (auto it = boid.vel.begin(); it != boid.vel.end(); ++it) {
    sum += pow(*it, 2);
  }
  return sum;
}
double mod(std::array<double, params::dim> const& vec)
{
  /*return sqrt(
      std::accumulate(vec.begin(), vec.end(), 0,
                      [](double sum, double x) { return sum = sum + x * x;
     }));*/
  return sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}

std::array<double, params::dim> normalize(std::array<double, params::dim>& vec)
{
  // assert(mod(vec)!=0);
  return vec / boids::mod(vec);
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

stormo generator(std::default_random_engine& eng)
{
  stormo set{};
  for (unsigned int i = 0; i < params::size; i++) {
    auto pix = pixel.begin(); // puntatore ai pixel
    boidstate boidprova{generate(eng)};
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

void speedadjust(boidstate& boid, const double speedlimit, const double speedminimum)
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

auto neighbors(stormo const& set, boidstate const& boid, const double d, const double alpha)
{
  std::vector<boidstate const*> neighbors{};
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    if (distance(boid, neighbor) < pow(d, 2) && distance(boid, neighbor) != 0) {
      std::array<double, params::dim> deltax = neighbor.pos - boid.pos;
      std::array<double, params::dim> y      = boid.vel;
      if (boids::mod(boid.vel) != 0)
        deltax = normalize(deltax);
      y = normalize(y);
      double prodscalare =
          std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
      if ((prodscalare) >= std::cos(alpha)) {
        auto prova = &neighbor;
        //std::cout << "prova " << prova->pos[0] << std::endl;
        neighbors.emplace_back(prova);
      }
    }
  });
  for (auto& it : neighbors) {
    auto prova = it->pos[0];
    //std::cout << "roba a caso dentro" << (double)it->pos[0] << std::endl;
  }
  return neighbors;
}

auto neighbors(std::vector<boidstate const*> const& set, boidstate const& boid,
               const double d, const double alpha)
{
  std::vector<boidstate const*> neighbors{};
  std::for_each(set.begin(), set.end(), [&](auto& neighbor) {
    if (distance(boid, *neighbor) < pow(d, 2)
        && distance(boid, *neighbor) != 0) {
      std::array<double, params::dim> deltax = neighbor->pos - boid.pos;
      std::array<double, params::dim> y      = boid.vel;
      if (boids::mod(boid.vel) != 0)
        deltax = normalize(deltax);
      y = normalize(y);
      double prodscalare =
          std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
      if ((prodscalare) >= std::cos(alpha)) {
        // std::cout<<"valore prima"<<neighbor->pos[0]<<"\n";
        neighbors.emplace_back(neighbor);
      }
    }
  });
  for (auto& it : neighbors) {
    // std::cout<<"roba a caso dentro"<<it->pos[0]<<std::endl;
  }
  return neighbors;
}

void regola1(std::vector<boidstate const*>& neighbors, boidstate& boid,const double repulsione)
{
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->pos - boid.pos;
   // std::cout << "roba a caso " << neighbor->pos[0] << "\n";
    boid.vel += -repulsione * (x);
  });
}

void regola2(std::vector<boidstate const*>& neighbors, boidstate& oldboid,
             boidstate& boid, const double steering)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](auto& neighbor) {
    auto x = neighbor->vel - oldboid.vel;
    boid.vel +=steering / n * (x);
  });
}

void regola3(std::vector<boidstate const*>& neighbors, boidstate& boid, const double cohesion)
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
  return sqrt(s) / set.size();
}

auto compx(stormo const& set) // Media delle componenti x di vel
{
  double s{};
  for (auto it = set.begin(); it != set.end(); ++it) {
    s += (*it).vel[0];
  }

  return s / set.size();
}

auto compy(stormo const& set) // Media delle componenti y di vel
{
  double s{};
  for (auto it = set.begin(); it != set.end(); ++it) {
    s += (*it).vel[1];
  }

  return s / set.size();
}

/*std::function<double(double)> cosine {[](double theta){return cos(theta);}};
std::function<double(double)> sine {[](double theta){return sin(theta);}};
std::vector<std::function<double(double)>> trig{sine, cosine};*/

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
stormo ensemble::set_()
{
  return set;
}
stormo ensemble::newset_()
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

void ensemble::update()
{
  for (auto it = set.begin(), jt = newset.begin(); it != set.end();
       ++it, ++jt) {
    auto neighbor{neighbors(set, *it, paramms::neigh_align,paramms::alpha)};
    auto close_neighbor{neighbors(neighbor, *it, paramms::neigh2,paramms::alpha)};
    regola2(neighbor, *it, *jt,paramms::steering);
    regola1(close_neighbor, *jt,paramms::repulsione);
    regola3(neighbor, *jt, paramms::coesione);
    speedadjust(*jt,paramms::speedlimit,paramms::speedminimum);
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params::deltaT;
      if (*index > *pix-150) {
          *velind -= paramms::attraction;
        } else {
          if (*index < 150) {
            *velind += paramms::attraction;
          }
        }
    }
  }
  set = newset;
}
} // namespace boids
