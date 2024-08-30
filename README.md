# Simulazione di boids

- [Introduzione](#introduzione)
- [Indicazioni per la compilazione](#indicazioni-per-la-compilazione)
- [Modifiche implementative](#)
## Introduzione
Questa è una riconsegna individuale da parte di Lorenzo Nasi del progetto di gruppo nominato "Simulazione di boids su SFML", che aveva come altri componenti Tommaso Vicenzi e Alice Pezzi. Tommaso Vicenzi ha abbandonato il progetto.

### Principali modifiche 
- modifica delle classi boidstate ed ensemble
- rimozione dell'utilizzo simulaneo di template e polimorfismo, (ora nessuno dei due è presente)
- completata flessibilità del codice di simulare stormi in 2 oppure 3 dimensioni
- implementazione di hashing spaziale per migliorare la performance e poter sostenere grandi aree generative su SFML
- supporto alla parallelizzazione per migliorare la velocità di esecuzione (considerazioni successivamente)
- implementazione delle statistiche dello stormo e produzione di grafici tramite ROOT
- aggiunte interfacce per le differenti simulazioni
- completamento del testing
- correzione di errori

## Indicazioni per la compilazione

Per poter produrre gli eseguibili è necessario possedere le librerie di SFML, con una versione 2.5 o superiore.
Per poter utilizzare il parallelismo della standard library, gli utenti di MacOS dovranno inoltre installare la libreria `oneDPL` del progetto oneAPI di Intel, la quale, tra le varie possibilità, consente il supporto alla parallelizzazione degli algoritmi della standard library su `c++17`, anche per utenti MacOS. Per scaricarla è sufficiente eseguire 
```bash
$ brew install onedpl
```
da terminale. Per chi compila tramite linux, questa parte può essere ignorata. Gli utenti linux devono però commentare la linea 34 di `CMakeLists.txt`, e tutti gli altri riferimenti alla librerira `TBB` (Thread Building Blocks) di `oneDPL`. Infine gli utenti linux devono modificare, all'interno del file `doublevec.hpp` i riferimenti a:
```c++
#include<oneapi/dpl/algorithms>
#include<oneapi/dpl/execution>
```
trasformandoli negli standard
```c++
#include<algorithms>
#include<execution>
```
Sia per `SFML` che per `TBB` è necessario indicare, all'interno delle `include_directories()` di CMake i rispettivi percorsi.
Infine è necessario aver installato ROOT per la visualizzazione dei grafici.

Per costruire la build in debug mode, eseguire:
```bash
$ cmake -S . -B build -DCMAKE_TYPE_BUILD=Debug
```
Per costruire la build in release mode, eseguire invece:
```bash
$ cmake -S . -B build -DCMAKE_TYPE_BUILD=Release
```
la quale abilita opzioni aggressive di ottimizzazione come `-03` e rimuove il debug con `-DNDEBUD`.

I target costruiti dal progetto sono: `boidsfml`, che avvia una simulazione su `SFML`, per cui i parametri della simulazione sono da modificare all'interno del file `parametersfml.txt`; `simulation`, che avvia una simulazione dello stormo in base ai parametri letti dal file `parameters.txt`. All'inizio del programma vengono chiesti i valori di `updates` e `update_rate`, il primo che rappresenta il numero di iterazioni su cui calcolare la statistica, il secondo che specifica il numero di update del sistema prima di ogni registrazione dei dati statistici. `simulation3d` analogamente avvia la stessa simulazione, ma in 3 dimensioni. Tutti questi eseguibili possono essere fatti eseguire in parallelo, utilizzando l'opzione `--parallel` quando si esegue il programma. Inoltre si segnala come la statistica si riferisca sempre a tutti i boid generati, e quindi, se vengono generati più stormi differenti, i risultati saranno meno significativi.
Sono poi presenti 3 differenti eseguibili di testing: `boids.t` per il testing dello stormo in 2 dimensioni, `boids3d.t` per il testing in 3 dimensioni, e `statistics.t` per il testing della classe di statistica.

## Modifiche implementative

### Boid e BoidState

Le seguenti modifiche sono state apportate:
La classe `BoidState` contiene come membro `boid_` la classe `Boid` , la quale ha a sua volta i membri privati `pos_` e `vel_` di tipo `DoubleVec`, oltre al `flockID_`, di tipo `unsigned int` e al `gridID_`, di tipo `GridID`, che è un alias per `std::array<int. params::dim>`. Il `gridID` rappresenta l'identificatore nella griglia dell'hashing spaziale in cui si trova il boid.
Inoltre la classe `BoidState` contiene come membri `deltavel_`, ovvero un `DoubleVec` che contiene gli update apportati dalle 3 regole, oltre a `neighbors` e `close_neighbors`, degli `std::vector<boid const*>`, ovvero dei vettori di puntatori costanti a boid, che contengono i puntatori ai vicini del boid.

### Flock

La classe `ensemble` è stata sostituita dalla classe `Flock`, che contiene il membro `set_`, un `std::vector<BoidState>` che contiene lo stormo, e `hashMap_`, una hash map costruita su `std::unordered_multimap<>`. Oltre ai metodi di accesso ai membri, essa possiede i metodi `update_hashMap` e `update`, che gestiscono l'update di tutto lo stormo e della mappa.

### Funzioni libere richiamate da `Flock::update()`

Le funzioni `speed_adjust`, `bordercheck`, e le regole di volo rimaste pressoché invariate a prima. E' stata aggiunta la funzione `random_boid` per generare un singolo boid casuale. Il `gridID` del boid viene aggiornato tramite la funzione `update_id`, secondo la convenzione per cui il quadrato della griglia più vicino all'origine, nel quadrante positivo, prende le coordinate (1,1) o (1,1,1) a seconda che ci si trovi in 2 o 3 dimensioni.

La funzione `is_neighbor` stabilisce se due boid sono vicini in base alla distanza, all'angolo di vista e al criterio di appartenenza allo stormo. In questa funzione, come in altre, vengono fatti confronti tra double, che possono dare risultati imprecisi, soprattuto quando si confrontano con lo zero, ma non è stato possibile trovare soluzioni soddisfacenti in tempo. Inoltre all'interno di questa funzione viene richiamata la funzione `cos_angle_between`, per calcolare l'angolo tra il vettore velocità di un boid, e il vettore differenza con un suo vicino. Quando uno dei due input è zero, la funzione restituisce 0, ma è in realtà indefinita, in quanto non si può stabilire un angolo. Invece di gestire questa possibilità all'interno della funzione, l'input viene garantito essere diverso da 0, in quanto il parametro `speedminimum` deve essere >0, mentre preliminarmente un boid non viene considerato come vicino se la `distanza==0`, escludendo quindi che il vettore differenza sia uguale a 0. La scelta di non considerare un boid che si trova sulla stessa posizione di un altro come vicino è stata fatta in quanto non è appunto possibile stabilire se in quel caso limite i due boid si vedano o meno, oltre ad essere una condizione rara e transitoria.

La funzione `add_neighbors` applica la funzione `is_neighbor` all'interno di un bucket della hash map, determinato in base ad un `gridID`, e nel caso i boid siano vicini, il neihgbor viene aggiunto al vettore dei vicini del boid. La funzione `update_neighbors` applica la funzione `add_neighbors` in tutti i bucket in un 3x3 o 3x3x3 intorno al bucket del boid di riferimento.
La funzione `update_close_neighbors` controlla solo la regola di vicinanza senza l'angolo o il criterio di appartenenza, e prende in input non tutto lo stormo, ma il vettore dei vicini, in quanto tutti i potenziali `close_neighbors` sono anche `neighbors`. Questa funzione viene utilizzata solo quanto è stato generato un solo stormo, mentre negli altri casi la ricerca tra `neighbors` e `close_neighbors` è indifferente. 
La funzione `update_all_neighbors` aggiorna lo stato di `neighbors_` e `close_neighbors_` di un `BoidState`, utilizzando le funzioni elencate in precedenza.
La funzione `update_rules` applica le regole di volo, modificando `deltavel_`. Tutte queste funzioni prendono generalmente in input singolarmente il membro `boid_`, `deltavel_`,`neighbors_` e `close_neighbors_` invece che tutta l'istanza di `BoidState` per evidenziare quali membri vengano usati in scrittura e quali in lettura in ogni fase dell'update, anche se questo non elimina la possibilità di una race condition