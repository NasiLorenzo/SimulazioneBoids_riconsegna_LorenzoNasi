# Simulazione di boids su SFML
L'implementazione dei boids è suddivisa su tre headerfiles, i quali sono `doublevec.hpp`, `boids.hpp` e `sfmlboids.hpp`.
I primi due file contengono le dichiarazioni di funzioni e classi le quali possono essere utilizzate per creare boids in un numero di dimensioni corrispondenti al parametro statico `params::dim`, ma dato che per ora l'unica finalità è SFML, esso è fissato a 2 nella dichiarazione. La parte legata ad SFML è definita nell'ultimo header.

I test sono nel file `boids.test.cpp`, mentre il main di SFML è in `boidsfml.cpp`. Per la simulazione su SFML i parametri vengono letti dal file `parametrisfml.txt`. 

# Implementazione 

I vettori che specificano posizioni e velocità dei boids sono di tipo `DoubleVec`, che è un alias per `std::array<double, params::dim>`, in modo tale che la loro dimensione sia nota al compile time.



