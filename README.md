# Simulazione di boids

-[Introduzione](#introduzione)\\
-[Indicazioni per la compilazione](#indicazioni-per-la-compilazione)

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

Per poter produrre gli eseguibili è necessario possedere le librerie di SFML, con una versione 2.5 o superiore.\\
Per poter utilizzare il parallelismo della standard library, gli utenti di MacOS dovranno inoltre scaricare la libreria `oneDPL` del progetto oneAPI di Intel, la quale, tra le varie possibilità, consente il supporto alla parallelizzazione degli algoritmi della standard library su `c++17`, anche per utenti MacOS. Per scaricarla è sufficiente eseguire 
```bash
brew install onedpl
```
da terminale. Per chi compila tramite linux, questa parte può essere ignorata. Gli utenti linux devono però commentare la linea 34 di `CMakeLists.txt`, e tutti gli altri riferimenti alla librerira `TBB` (Thread Building Blocks). Infine gli utenti linux devono modificare, all'interno del file `doublevec.hpp` i riferimenti a:
```c++
#include<oneapi/dpl/algorithms>
#include<oneapi/dpl/execution>
```
trasformandoli negli standard
```c++
#include<algorithms>
#include<execution>
```
Sia per 

Per costruire la build in debug mode, eseguire:
```bash
$ cmake -S . -B build -DCMAKE_TYPE_BUILD=Debug
```
Per costruire la build in release mode, eseguire invece:
```bash
$ cmake -S . -B build -DCMAKE_TYPE_BUILD=Release
```
la quale abilita opzioni aggressive di ottimizzazione come `-03` e rimuove il debug con `-DNDEBUD`.
