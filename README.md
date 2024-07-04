# Simulazione di boids su SFML
L'implementazione dei boids è suddivisa su tre headerfiles, i quali sono `doublevec.hpp`, `boids.hpp` e `sfmlboids.hpp`.
I primi due file contengono le dichiarazioni di funzioni e classi le quali possono essere utilizzate per creare boids in un numero di dimensioni corrispondenti al parametro statico `params::dim`, ma dato che per ora l'unica finalità è SFML, esso è 