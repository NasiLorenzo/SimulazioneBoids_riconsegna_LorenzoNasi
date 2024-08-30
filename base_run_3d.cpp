#include "simulation.hpp"
#include "boids.hpp"
int main(int argc, char* argv[]){
    std::random_device r;
    try{
    boids::Simulation sim{"parameters3d.txt",r};
    check_parallelism(argc,argv,sim.params());
    sim.loop("pos3d.txt","vel3d.txt","pos_mod3d.txt","vel_mod3d.txt","distance3d.txt");}
    catch(std::exception const& e){
        std::cerr<<"Failure: "<<e.what()<<"\n";
        return EXIT_FAILURE;
    }
}