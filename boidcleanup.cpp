#include "boidcleanup.hpp"

namespace boids {

double paramms::repulsione  = 0.7;
double paramms::steering    = 0.1;
double paramms::coesione    = 0.1;
double paramms::neigh2      = 100000;
double paramms::neigh_align = 100000;

/*inline boidstate generate(std::default_random_engine eng)
{ // genera pos e vel di un boid distribuiti secondo
  // una gauss centrata in 0
  boidstate boid{};
  std::normal_distribution<double> dist(0.0, params::sigma);
  for (auto it = boid.pos.begin(); it != boid.pos.end(); ++it) {
    *it = dist(eng);
  }
  for (auto it = boid.vel.begin(), last = boid.vel.end(); it != last; ++it) {
    *it = (params::vel_factor * dist(eng));
  }
  return boid;
}

inline double distance(const boidstate& a, const boidstate& b)
{ // sqrt dispendiosa
  double s{};
  for (auto it = a.pos.begin(), index = b.pos.begin(); it != a.pos.end();
       ++it, ++index) {
    s += pow((*it) - (*index), 2);
  }
  return s;
}

inline stormo generator(std::default_random_engine eng)
{
  stormo set;
  for (unsigned int i = 0; i < params::size; i++) {
    auto pix = pixel.begin();
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
*/

std::array<double, params::dim> operator+(std::array<double, params::dim> a,
                                          std::array<double, params::dim> b)
{
  std::array<double, params::dim> result{};
  for (auto a_it = a.begin(), b_it = b.begin(), r_it = result.begin();
       a_it != a.end(); ++a_it, ++b_it, ++r_it) {
    *r_it += *a_it + *b_it;
  };
  return result;
}

std::array<double, params::dim> operator+=(std::array<double, params::dim>& a,
                                           std::array<double, params::dim>const& b)
{
  auto b_it = b.begin();
  for (auto a_it = a.begin(); a_it != a.end();
       ++a_it, ++b_it) {
    *a_it += *b_it;
  };
  return a;
}

std::array<double, params::dim> operator*(std::array<double, params::dim> a,
                                          std::array<double, params::dim> b)
{
  std::array<double, params::dim> result{1., 1.};
  for (auto a_it = a.begin(), b_it = b.begin(), r_it = result.begin();
       a_it != a.end(); ++a_it, ++b_it, ++r_it) {
    *r_it += *a_it * *b_it;
  };
  return result;
}

stormo generator(std::default_random_engine eng)
{
  stormo set;
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
  for (auto index = set.begin(); index != set.end(); ++index) {
    if (distance(boid, *index) < pow(d, 2) && distance(boid, *index) != 0)
      neighbors.push_back(*index);
  }
  return neighbors;
}

boidstate regola1(stormo& neighbors, boidstate& boidi)
{
  boidstate boid{boidi};
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    for (auto it = boid.vel.begin(), jt = (*index).pos.begin(),
              i = boid.pos.begin();
         it != boid.vel.end(); ++it, ++jt, ++i) {
      *it += -paramms::repulsione * ((*jt) - *i);
      // std::cout<<"regola1: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
    }
  }
  return boid;
}

auto regola2(stormo& neighbors, boidstate& boidi)
{
  boidstate boid{boidi};
  boidstate boidcopia{boid};
  auto n = neighbors.size();
  for (auto index = neighbors.begin(); index != neighbors.end(); ++index) {
    for (auto it = boid.vel.begin(), jt = (*index).vel.begin(),
              i = boidcopia.vel.begin();
         it != boid.vel.end(); ++it, ++jt, ++i) {
      (*it) += paramms::steering / (n) * ((*jt) - *i);
      // std::cout<<"regola2: vel boid "<<it-boid.vel.begin()+1<<*it<<"\n";
    }
  }
  return boid;
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

auto mod_vel(boidstate const& boid) // Velocità singolo boid
{
  double sum{};
  for (auto it = boid.vel.begin(); it != boid.vel.end(); ++it) {
    sum += pow(*it, 2);
  }
  return sqrt(sum);
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
    // meiosi(set,neighbor,*jt,eng,params::reproduction);
    *jt      = regola1(close_neighbor, *jt);
    *jt      = regola2(neighbor, *jt);
    *jt      = regola3(neighbor, *jt);
    auto pix = pixel.begin();
    for (auto index = (*jt).pos.begin(), velind = (*jt).vel.begin();
         index != (*jt).pos.end(); ++index, ++velind, ++pix) {
      (*index) += (*velind) * params::deltaT;
      (*index) = fmod(*index, *pix * params::rate);
      if (*index <= 0)
        *index += *pix * params::rate;
      assert(*index <= *pix * params::rate);
    }
  }
  // std::cout << "Velocità x e y " << compx(newset) << " " << compy(newset)<<
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
  // std::cout << "Velocità media " << meanvel(newset) << "\n";
  set = newset;
}
} // namespace boids
