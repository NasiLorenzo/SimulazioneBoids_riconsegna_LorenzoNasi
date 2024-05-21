#include "boidcleanup.hpp"

namespace boids {

double paramms::repulsione  = 0.7;
double paramms::steering    = 0.1;
double paramms::coesione    = 100;
double paramms::neigh2      = 20;
double paramms::neigh_align = 70;
double paramms::mod_align   = 0.000003;
double paramms::attraction  = 0.02;
double paramms::alpha       = M_PI;
double paramms::speedlimit =80;
double paramms::speedminimum=2;
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

void speedadjust(boidstate& boid){
  auto vnorm=boids::mod(boid.vel);
  if(vnorm>paramms::speedlimit){
    normalize(boid.vel);
    boid.vel=paramms::speedlimit * boid.vel;
  }
  if(vnorm<paramms::speedminimum){
    normalize(boid.vel);
    boid.vel=paramms::speedminimum*boid.vel;
  }
}

auto neighbors(stormo const& set, boidstate const& boid, const double d)
{  
  auto t1=std::chrono::high_resolution_clock::now();
  stormo neighbors{};
  std::for_each(set.begin(), set.end(), [&](boidstate neighbor) {
    if (distance(boid, neighbor) < pow(d, 2)&&distance(boid, neighbor)!=0) {
      std::array<double, params::dim> deltax = neighbor.pos - boid.pos;
      std::array<double, params::dim> y      = boid.vel;
      if (boids::mod(boid.vel) != 0)
        deltax = normalize(deltax);
      y = normalize(y);
      double prodscalare =
          std::inner_product(deltax.begin(), deltax.end(), y.begin(), 0.);
      if ((prodscalare) >= std::cos(paramms::alpha)/*||prodscalare==-1||prodscalare==1*/) {
        neighbors.push_back(neighbor);

      }
    }
  });
  //std::cout<<"Numero vicini "<<neighbors.size()<<"\n";
  //auto t2=std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double, std::milli> ms_double = t2 - t1;
  //std::cout<<"Tempo passato "<<ms_double.count()<<" ms\n";
  return neighbors;
}

void regola1(stormo& neighbors, boidstate& boid)
{
  std::for_each(neighbors.begin(), neighbors.end(), [&](boidstate neighbor) {
    auto x = neighbor.pos - boid.pos;
    boid.vel += -paramms::repulsione * (x);
  });
}

void regola2(stormo& neighbors, boidstate& oldboid, boidstate& boid)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](boidstate neighbor) {
    auto x = neighbor.vel - oldboid.vel;
    boid.vel += paramms::steering / n * (x);
  });
}

void regola3(stormo& neighbors, boidstate& boid)
{
  auto n = neighbors.size();
  std::for_each(neighbors.begin(), neighbors.end(), [&](boidstate neighbor) {
    auto x = neighbor.pos - boid.pos;
    boid.vel += paramms::steering / n * (x);
  });
}

auto regola4(stormo& neighbors, boidstate& boid)
{
  double a{1};
  auto n = neighbors.size();
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    a += paramms::mod_align / n * (mod_vel(*index) - mod_vel(boid));
  }
  boid.vel = a * boid.vel;
  return boid;
}

void meiosi(stormo& set, stormo& neighborss, boidstate& boid,
            std::default_random_engine eng, double distance)
{
  stormo neighbor{neighbors(neighborss, boid, distance)};
  boidstate child{generate(eng)};
  if (neighbor.size() > 2) {
    for (auto it = child.pos.begin(), jt = boid.pos.begin();
         it != child.pos.end(); ++it, ++jt) {
      std::uniform_real_distribution<double> dist(*jt - distance,
                                                  *jt + distance);
      *it += dist(eng);
    }
  }
  set.push_back(child);
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
 // std::cout << "Angolo " << angle(set[0]) << "\n";
  for (auto it = set.begin(), jt = newset.begin(); it != set.end();
       ++it, ++jt) {
    stormo neighbor{neighbors(set, *it, paramms::neigh_align)};
    stormo close_neighbor{neighbors(neighbor, *it, paramms::neigh2)};
    regola2(neighbor, *it, *jt);
    regola1(close_neighbor, *jt);
    regola3(neighbor, *jt);
    speedadjust(*jt);
    //*jt      = regola4(neighbor, *jt);
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params::deltaT;
      /*(*index) = fmod(*index, *pix * params::rate);
      if (*index <= 0)
       *index += *pix * params::rate;*/
      // assert(*index <= *pix * params::rate);
      if (*index > *pix-40) {
          *velind -= paramms::attraction;
        } else {
          if (*index < 40) {
            *velind += paramms::attraction;
          }
        }
    }
    /*for (auto it = newset.begin(); it != newset.end(); ++it) {
      auto pix = pixel.begin();
      for (auto index = it->pos.begin(), velind = it->vel.begin();
           index != it->pos.end(); ++index, ++velind, ++pix) {
        if (*index > *pix-200) {
          *velind -= paramms::attraction;
        } else {
          if (*index < 200) {
            *velind += paramms::attraction;
          }
        }
      }
    }*/
  }
  set = newset;
}

void ensemble::brown_update(std::random_device& r)
{
  std::default_random_engine eng(r());
  for (auto it = set.begin(), jt = newset.begin(); it != set.end();
       ++it, ++jt) {
    stormo neighbor{neighbors(set, *it, params::neigh_co)};
    stormo close_neighbor{neighbors(set, *it, paramms::neigh2)};
    //*jt = regola1(close_neighbor, *jt);
    //*jt = regola2(neighbor, *jt);
    //*jt = regola3(neighbor, *jt);
    std::uniform_int_distribution<int> dist(0, params::rate2);
    std::uniform_real_distribution<double> dist2(-params::pi / 2,
                                                 params::pi / 2);
    std::cout << "dist " << dist(eng) << "\n";
    if (dist(eng) % params::rate2 == 0)
      *jt = rotate(*jt, dist2(eng));
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params::deltaT;
      // std::cout<<"pos "<<it-set.begin()+1<<" "<<*index<<" "<<"\n";
      //(*index) = fmod(*index, *pix); // reinserire fmod con *pix
      if (*index <= 0)
        *index += *pix;
      assert(*index <= *pix);
    }
  }
  std::cout << "Velocità media " << meanvel(newset) << "\n";
  set = newset;
}
} // namespace boids
