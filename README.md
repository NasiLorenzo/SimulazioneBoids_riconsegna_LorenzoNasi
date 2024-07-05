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

Le informazioni sullo stormo necessarie per applicare le regole sono contenute nella classe `ensemble`, costituita dai membri privati `set` e `newset`, anche se questa scelta non è la più ottimale. L'aggiornamento dello stato dello stormo avviene attraverso il metodo `update()`, che agisce sui due membri. L'inizializzazione di un'istanza della classe ensemble viene fatta tramite un `std::vector<boidstate>` i cui valori vengono assegnati a `set` e `newset`.

Per poter applicare le regole di volo è necessario definire delle funzioni che agiscano su degli `std::vector<boidstate>`, ed essendo queste funzioni esterne non è possibile sfruttare l'ereditarietà sulla classe derivata `SFMLboid`, e per questo motivo sono state definite come funzioni template, e lo stesso vale per la classe ensemble.

### functions

Le funzioni template, con eccezione di `update()`, che è un metodo della classe template, sono inserite in una struct template chiamata `functions`, e sono dichiarate come membri statici, in modo da poter essere richiamate senza istanziare la struct, che ha lo scopo di introdurre un'unica definizione di template per tutte le funzioni.
Le prime due funzioni sono `generate` e `generator`, la prima che genera un boid nell'origine con la velocità distribuita secondo una gaussiana su tutti gli assi, mentre la seconda crea un `std::vector<boidstate>` con le posizioni distribuite uniformemente entro il range positivo definito dal parametro `pixel`. 

La funzione `neighbors` crea un vettore di puntatori costanti a `boidstate`, prendendo come input, alternativamente, un `std::vector<boidstate>` o direttamente un altro vettore di puntatori, oltre alla distanza di vicinanza, l'angolo visivo, e il boid di riferimento rispetto a cui calcolare la distanza. Inoltre essa contiene la possibilità di controllare, tramite un criterio definito in una `enum class`, se i vicini appartengono allo stesso stormo, e quindi se considerarli o meno. Il calcolo dell'angolo visivo avviene tramite la funzione `cosangleij`, che calcola il coseno dell'angolo compreso tra due `DoubleVec`.

Successivamente ci sono le regole di volo, suddivise nella `regola1`e nella `regola2_3`, quest'ultime accorpate in quanto agiscono sugli stessi vicini. Esse prendono in input un boid su cui applicare le regole, i parametri necessari, e un vettore di vicini da usare per applicare le regole. 
Dopo questo c'è la funzione `speedadjust`, che riscala la velocità di un boid preso in input entro il range determinato.
Infine la funzione `bordercheck_posupdate`, controlla le condizioni al bordo e aggiorna la posizione del boid.

Il metodo update, che ha accesso a `newset` e `set`, applica le regole di volo ad ogni boid, all'interno di un `std::for_each`. Quindi per ogni boid vengono creati i vettori dei vicini, e poi vengono applicate le tre regole citate nel paragrafo precedente. Le modifiche vengono fatte a newset, prendendo i dati di input delle velocità da set. Infine bisogna aggiornare lo stato di set, che per ora è stato usato solo in lettura.

### SFML

Per l'implementazione su SFML, all'interno di un main, in primo luogo vengono letti tutti i parametri delle funzioni dal file `parametrisfml.txt`. Successivamente, tramite la funzione `generatecolors`, si crea un vettore di elementi della struct `RGB`, che contiene le tre componenti di un colore all'interno di variabili di tipo `uint8_t`, che sono quelle utilizzate da SFML per identificare i colori. I colori sono generati casualmente, e la dimensione del vettore è pari al numero di stormi differenti presenti nella simulazione. Successivamente si assegnano i colori ai boid tramite la funzione `assigncolors`. 
Dopo avere tutto quello che serve per la simulazione, si apre la finestra, con dimensioni determinate dal parametro pixel. Si setta il tempo di un frame al parametro deltaT, si crea la variabile `clock`, di tipo `sf::Clock`, e si stabilisce un sistema di gestione degli eventi, che qui è minimo. Successivamente si azzera la variabile `clock`, si esegue la funzione update, si ripulisce la finestra, e si disegnano i boid. 
Per mantenere costante il framerate, si imposta un delay corrispondente alla differenza tra il tempo trascorso e il tempo di un frame. Dopodichè inizia la computazione per il frame successivo.
La simulazione procede fino a che non si chiude manualmente la finestra.

## Testing

Il testing è stato svolto sulle tre regole, sul raggio visivo, sull'angolo visivo, e sui limiti di velocità.

Per il testing sulle regole, è stata creata un'istanza di `ensemble` con 10 boid, le cui posizioni e velocità sono state impostate manualmente. Nonostante non sia formalmente corretto, invece di testare direttamente le funzioni che fanno riferimento alle regole, è stata testata la funzione update, in cui si impostando i parametri di distanza, angolo visivo, e limitì di velocità in modo che non apportino modifiche. Per controllare che le posizioni e velocità dei boid siano aggiornate correttamente, le si confrontano con quelle ottenute con un foglio excel, dove i calcoli per sono stati svolti manualmente. 
Per il testing del raggio e dell'angolo visivo, sono state posizionate manualmente coppie di boid in posizioni specifiche,e si è controllato se il primo boid è in grado di vedere gli altri o meno. Inoltre sono stati calcolati e controllati direttamente i valori della distanza e del coseno dell'angolo utilizzati all'interno della funzione `neighbors` per svolgere il controllo di vicinanza. In questo caso i valori degli angoli attesi sono stati calcolati manualmente.
Per il testing sui limiti di velocità, sono stati creati alcuni boid, è si è controllato il modulo della loro velocità prima e dopo l'applicazione della funzione `speedadjust`.