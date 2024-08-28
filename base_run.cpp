#include "simulation.hpp"

int main(){
    std::random_device r;
    boids::Simulation sim{"parametrisfml.txt",r};
    sim.loop(50,200,"pos.txt","vel.txt","pos_mod.txt","vel_mod.txt","distance.txt");
}