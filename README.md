# ROS2-Project

## Introduzione

ROS (Acronimo di Robot Operating System) rappresenta un insieme di librerie software e strumenti destinati alla realizzazione di applicazioni robotiche. Questo sistema include una vasta gamma di componenti, che vanno dai driver agli algoritmi all'avanguardia, fino agli strumenti di sviluppo potenti, fornendo tutto il necessario per lo sviluppo di progetti robotici sofisticati. ROS 2 è stato progettato come evoluzione di ROS 1, con l'obiettivo di rispondere alle esigenze in continua evoluzione nel campo della robotica e dell'automazione, sfruttando i punti di forza di ROS 1 e migliorando gli aspetti che presentavano limitazioni


## I primi passi

Per cominciare a capire il funzionamento di ROS e i suoi componenti principali, è possibile utilizzare varie finestre del terminale per farle successivamente interagire tra di loro. Prima di ciò, però, è importante avere chiaro i vari componenti che costituiscono il core di questo sistema operativo.


### Nodi

I nodi in ROS sono i componenti fondamentali, che permettono all'applicazione di funzionare correttamente. Ogni nodo è indipendente dagli altri e svolge un particolare compito che gli viene assegnato (tramite codice PYTHON). Per capire come questi nodi operano, è disponibile un pacchetto di demo chiamato 'demo_nodes_*' che ne contiene sia di semplici che di più complessi.  
Per fare qualche esempio, è disponibile un nodo di nome 'talker' che si preoccupa di mandare un messaggio di 'Hello World' ogni secondo; è inoltre disponbile un nodo chiamato 'listener' che invece si mette in ascolto di eventuali messaggi. Vi sono altri esempi più complessi, come il 'turtlesim' che rappresenta una tartaruga statica in uno spazio e il relativo nodo 'turtle_teleop_key' che manda i comandi di movimento alla tartaruga dall'input da tastiera.  
Questo insieme di nodi, ovviamente, può essere programmato ad hoc a seconda delle esigenze, prendendo magari spunto da un nodo di demo e modificato a proprio piacimento. Ne deriva inoltre che questi nodi non possono comunicare tra loro se non si adotta una certa politica di comunicazione che, come vedremo in seguito, prende il nome di 'Topic'.

### Topics

Sapendo quindi che ogni nodo è responsabile di una determinata azione, è importante chiarire il funzionamento dei topic che ne permettono la comunicazione tra essi. Quando si aprono finestre di terminale e si decide di assegnare per ognuna un nodo, la loro interazione segue tipicamente il paradigma 'pub-sub' a scambio di messaggi dove uno o più nodi fungono da publishers e altrettanti da subscribers. Quindi, i topic non sono altro che la rappresentazione di questa comunicazione.

<p>&nbsp;</p>

![Turtlesim-topic](/turtlesim.png)  

<p>&nbsp;</p>

Come si può osservare, in questo grafico (generato tramite rqt_graph con group 0 e Nodes/Topics view) si hanno i due nodi già menzionati (la tartaruga e il suo controller) e in mezzo vi sono i vari topic di comunicazione. In particolare
vi è il controller che comunica tramite topic 'cmd_vel' gli input da tastiera, mentre l'oggetto tartaruga comunica tramite due canali distinti il feedback e il suo stato attuale.  
Più nello specifico, il nodo di controllo si comporta da publisher degli input da tastiera, mentre il nodo turtlesim si iscrive a quest'ultimo topic. Allo stesso modo, turtlesim opera publishing di stato feedback e status mentre il controller si iscrive a questi ultimi.

### Servizi

I servizi in ROS rispondono all'esigenza dell'archiettura client-server il quale non è nativo per i topic così come sono stati creati. Infatti, un nodo tramite un topic classico manda un'informazione ogni x tempo ma non si aspetta una risposta dall'altra parte (come già accennato, segue il paradigma pub-sub).  
Tramite i servizi, invece, è possibile implementare ciò che caratterizza l'interazione tipicamente web in cui vi sono richieste e risposte, dei server che si preoccupano di esporre servizi e i client che ne usufruiscono.

<p>&nbsp;</p>
<div align="center">
  <img src="/services.png" alt="services.png">
</div>
<p>&nbsp;</p>

Nell'immagine soprastante vi è la rappresentazione dell'archiettura client-server in cui un client (ma potrebbero essere anche di più) manda richieste e riceve risposte tramite il servizio.  
Un esempio di come questo possa essere implementato può essere osservato sempre tramite il pacchetto demo utilizzando su un terminale add_two_ints_server (nodo Server) e in un'altra finestra la keyword 'call' aggiungendo i parametri a e b. Così facendo, si manderà una richiesta al server il quale risponderà con il risultato.


## Compilazione e Installazione

Normalmente, l'installazione (su Ubuntu) prevedere un semplice comando da terminale e l'installazione viene completata automaticamente. Nel nostro caso, però, si cercherà di compilare manualmente l'insieme di file per snellire il più possibile l'installazione e facilitarne il porting su un'architettura RISC-V (nonostante ROS sia progettato per architetture Intel)  
Quindi, si procederà alla creazione di un ambiente virtuale che consenta (nel caso di errori irreversibili) di non danneggiare il sistema e di conseguenza che si possano provare varie combinazioni di compilazione.  

### Installazione di un ambiente virtuale

Prima di procedere, controllare che la propria macchina sia compatibile con la virtualizzazione integrata UNIX. Lanciare quindi il comando:

```bash
$ egrep -c '(vmx|svm)' /proc/cpuinfo

