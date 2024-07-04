# Simulazione di boids su SFML

I componenti del gruppo sono Lorenzo Nasi, Tommaso Vicenzi e Alice Pezzi.

## Introduzione

L'implementazione dei boids è suddivisa su tre headerfiles, i quali sono `doublevec.hpp`, `boids.hpp` e `sfmlboids.hpp`, con i rispettivi `.cpp/tpp`.
I primi due file contengono le dichiarazioni di funzioni e classi le quali possono essere utilizzate per creare boids in un numero di dimensioni corrispondenti al parametro statico `params::dim`, ma dato che per ora l'unica finalità è SFML, quest'ultimo è fissato a 2 nella dichiarazione. La parte legata ad SFML è definita nell'ultimo header.

I test sono nel file `boids.test.cpp`, mentre il main di SFML è in `boidsfml.cpp`. Per la simulazione su SFML i parametri vengono letti dal file `parametrisfml.txt`. La compilazione avviene tramite CMake, tramite il `CMakeLists.txt` allegato.

## Scelte implementative 

### boidstate

I vettori che specificano posizioni e velocità dei boids sono di tipo `DoubleVec`, che è un alias per `std::array<double, params::dim>`, in modo tale che la loro dimensione sia nota al compile time. Sono state inoltre create altre funzioni esterne che agiscono su oggetti di tipo `DoubleVec`, come la funzione per calcolare la distanza tra due vettori (il modulo della differenza), la funzione per calcolare il modulo di un vettore, e quella per calcolarne l'angolo rispetto all'asse x, anche se ovviamente quest'ultima limita la generalità precedentemente citata rispetto al numero di dimensioni.

Tutte le informazioni su di un boid sono racchiuse nella struct `boidstate`, la quale presenta due membri di tipo `DoubleVec`, denominati `pos` e `vel`, oltre a `flockID`, che ne specifica lo stormo di appartenza, inizializzato a zero. Per la parte di SFML è stata creata una classe derivata di `boidstate`, chiamata `SFMLboid`, la quale contiene il nuovo membro `arrow`, che è una `sf::ConvexShape`, ovvero una figura geometrica convessa modificabile contenuta in SFML, che rappresenta il boid. 
Inoltre il costruttore di default della classe derivata inizializza l'istanza di arrow in modo tale che sia direttamente disegnabile.

### ensemble

Le informazioni sullo stormo necessarie per applicare le regole sono contenute nella classe `ensemble`, costituita dai membri privati `set` e `newset`, anche se questa scelta non è la più ottimale. L'aggiornamento dello stato dello stormo avviene attraverso il metodo `update()`, che agisce sui due membri. 

Per poter applicare le regole di volo è necessario definire delle funzioni che agiscano su degli `std::vector<boidstate>`, ed essendo queste funzioni esterne non è possibile sfruttare l'ereditarietà sulla classe derivata `SFMLboid`, e per questo motivo sono state definite come funzioni template, così come lo stesso vale per la classe ensemble.

### functions

Le funzioni template, con eccezione di `update()`, che è un metodo della classe template, sono inserite in una struct template chiamata `functions`, e sono dichiarate come membri statici, in modo da poter essere richiamate senza istanziare la struct, che ha lo scopo di introdurre un'unica definizione di template per tutte le funzioni.
Le prime due funzioni sono `generate`e `generator`, la prima che genera un boid nell'origine con la velocità distribuita secondo una gaussiana su tutti gli assi, mentre la seconda crea un `std::vector<boidstate>` con le posizioni distribuite uniformemente entro il range positivo definito dal parametro `pixel`. 

