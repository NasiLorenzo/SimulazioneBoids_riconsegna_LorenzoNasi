#include "simulation.hpp"
#include "boids.hpp"
int main(int argc, char* argv[]){
    std::random_device r;
    boids::Simulation sim{"parametri.txt",r};
    check_parallelism(argc,argv,sim.params());
    sim.loop("pos.txt","vel.txt","pos_mod.txt","vel_mod.txt","distance.txt");
}