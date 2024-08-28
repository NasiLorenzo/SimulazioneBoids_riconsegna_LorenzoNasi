#include "simulation.hpp"

int main(){
    std::random_device r;
    boids::Simulation sim{"parametri.txt",r};
    std::cout<<"Inserisci updates e update_rate: ";
    unsigned int a,b;
    std::cin>>a>>b;
    std::cout<<"Valgono: "<<a<<" "<<b<<"\n";
    sim.loop(a,b,"pos.txt","vel.txt","pos_mod.txt","vel_mod.txt","distance.txt");
}