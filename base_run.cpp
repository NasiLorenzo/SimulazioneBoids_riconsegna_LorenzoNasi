#include "simulation.hpp"

int main(){
    std::random_device r;
    boids::Simulation sim{"parametri.txt",r};
    sim.loop("pos.txt","vel.txt","pos_mod.txt","vel_mod.txt","distance.txt");
}