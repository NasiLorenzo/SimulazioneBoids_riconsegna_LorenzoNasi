#include "boidcleanup.hpp"

namespace boids {

double paramms::repulsione  = 0.7;
double paramms::steering    = 0.1;
double paramms::coesione    = 0.1;
double paramms::neigh2      = 20;
double paramms::neigh_align = 70;
double paramms::mod_align   = 0.000003;

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

auto neighbors(stormo const& set, boidstate const& boid, const double d)
{
  stormo neighbors{};
  std::for_each(set.begin(), set.end(), [&](boidstate neighbor) {
    if (distance(boid, neighbor) < pow(d, 2) && distance(boid, neighbor) != 0)
      neighbors.push_back(neighbor);
  });
  return neighbors;
}

boidstate regola1(stormo& neighbors, boidstate& boid_old)
{
  boidstate boid{boid_old};
  /*for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    for (auto it = boid.vel.begin(), jt = (*index).pos.begin(),
              i = boid.pos.begin();
         it != boid.vel.end(); ++it, ++jt, ++i) {
      *it += -paramms::repulsione * ((*jt) - *i);
      // std::cout<<"regola1: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
    }
  }*/
  // for (auto& index: neighbors) {
  // auto x=index.pos-boid.pos;
  // boid.vel += -paramms::repulsione * x;
  //  std::cout<<"regola1: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
  // boid.vel += -paramms::repulsione * index.pos + paramms::repulsione *
  // boid.pos;

  //}
  std::for_each(neighbors.begin(), neighbors.end(), [&](boidstate neighbor) {
    auto x = neighbor.pos - boid.pos;
    boid.vel += -paramms::repulsione * (x);
  });
  return boid;
}

void _regola2(stormo& neighbors, boidstate& boid_old, boidstate& boid)
{
  auto n = neighbors.size();
  std::cout << "vicini 2 " << n << "\n";
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    auto x = index->vel-boid_old.vel;
    boid.vel += paramms::steering / n * (x);
    //(*it) += paramms::steering / (n) * ((*jt));
    // std::cout<<"regola2: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
  }
  //boid.vel += -paramms::steering * (boid_old.vel);
}

void regola2(stormo& neighbors, boidstate& boid_old, boidstate& boid)
{
  auto n = neighbors.size();
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    for (auto it = boid.vel.begin(), jt = (*index).vel.begin(),
              i = boid_old.vel.begin();
         it != boid.vel.end(); ++it, ++jt, ++i) {
      (*it) += paramms::steering / (n) * ((*jt) - *i);
      //(*it) += paramms::steering / (n) * ((*jt));
      // std::cout<<"regola2: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
    }
  }
  // boid.vel+=-paramms::steering*(boid_old.vel);
}

auto regola3(stormo& neighbors, boidstate& boidi)
{
  boidstate boid{boidi};
  auto n = neighbors.size();
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    for (auto it = boid.vel.begin(), jt = (*index).pos.begin(),
              i = boid.pos.begin();
         it != boid.vel.end(); ++it, ++jt, ++i) {
      (*it) += paramms::coesione / (n) * ((*jt) - (*i));
    }
  }
  return boid;
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

void ensemble::update()
{
  for (auto it = set.begin(), jt = newset.begin(); it != set.end();
       ++it, ++jt) {
    stormo neighbor{neighbors(set, *it, paramms::neigh_align)};
    stormo close_neighbor{neighbors(neighbor, *it, paramms::neigh2)};
    *jt = regola1(close_neighbor, *it);
    regola2(neighbor, *it, *jt);
    *jt      = regola3(neighbor, *jt);
    //*jt      = regola4(neighbor, *jt);
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params::deltaT;
      //(*index) = fmod(*index, *pix * params::rate);
      /*if (*index <= 0)
       *index += *pix * params::rate;*/
      // assert(*index <= *pix * params::rate);
    }
  }
  std::cout << "Velocità x e y " << compx(newset) << " " << compy(newset)
            << "\n";
  std::cout << "velocità media " << meanvel(newset) << "\n";
  // "\n";
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
      (*index) = fmod(*index, *pix); // reinserire fmod con *pix
      if (*index <= 0)
        *index += *pix;
      assert(*index <= *pix);
    }
  }
  std::cout << "Velocità media " << meanvel(newset) << "\n";
  set = newset;
}
} // namespace boids
