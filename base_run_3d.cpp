#include "simulation.hpp"
#include "boids.hpp"
int main(int argc, char* argv[]){
    std::random_device r;
    try{
    boids::Simulation sim{"parameters3d.txt",r};
    check_parallelism(argc,argv,sim.params());
    sim.loop("./data/pos3d.txt","./data/vel3d.txt","./data/pos_mod3d.txt","./data/vel_mod3d.txt","./data/distance3d.txt");}
    catch(std::exception const& e){
        std::cerr<<"Failure: "<<e.what()<<"\n";
        return EXIT_FAILURE;
    }
}