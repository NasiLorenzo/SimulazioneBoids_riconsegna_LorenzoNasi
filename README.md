# Simulazione di boids su SFML

I componenti del gruppo sono Lorenzo Nasi, Tommaso Vicenzi e Alice Pezzi.

## Introduzione

L'implementazione dei boids è suddivisa su tre headerfiles, i quali sono `doublevec.hpp`, `boids.hpp` e `sfmlboids.hpp`, con i rispettivi `.cpp/tpp`.
I primi due file contengono le dichiarazioni di funzioni e classi le quali possono essere utilizzate per creare boids in un numero di dimensioni corrispondenti al parametro statico `params::dim`, ma dato che per ora l'unica finalità è SFML, quest'ultimo è fissato a 2 nella dichiarazione. La parte legata ad SFML è definita nell'ultimo header.

I test sono nel file `boids.test.cpp`, mentre il main di SFML è in `boidsfml.cpp`. Per la simulazione su SFML i parametri vengono letti dal file `parametrisfml.txt`. La compilazione avviene tramite CMake, tramite il `CMakeLists.txt` allegato.

## Scelte implementative 

I vettori che specificano posizioni e velocità dei boids sono di tipo `DoubleVec`, che è un alias per `std::array<double, params::dim>`, in modo tale che la loro dimensione sia nota al compile time.

Tutte le informazioni su di un boid sono racchiuse nella struct `boidstate`, la quale presenta due membri di tipo `DoubleVec`, denominate `pos` e `vel`, oltre a `flockID`, che ne specifica lo stormo di appartenza, inizializzato a zero. Per la parte di SFML è stata creata una classe derivata di `boidstate`, chiamata `SFMLboid`, la quale contiene il nuovo membro `arrow`, che è una `sf::ConvexShape`, ovvero una figura geometrica convessa modificabile contenuta in SFML, che rappresenta il boid. 
Inoltre il costruttore di default della classe derivata inizializza l'istanza di arrow in modo tale che sia direttamente disegnabile.



