# Simulazione di boids

- [Introduzione](#introduzione)
- [Indicazioni per la compilazione](#indicazioni-per-la-compilazione)

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
Sia per `SFML` che per `TBB` è necessario indicare, all'interno delle `include_directories()` di CMAKE i rispettivi percorsi.
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