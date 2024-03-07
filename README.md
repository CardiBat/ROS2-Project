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

```sh
$ egrep -c '(vmx|svm)' /proc/cpuinfo
```

Se il comando restituisce un numero maggiore di 0, significa che il processore supporta la virtualizzazione. Se restituisce 0, potrebbe essere necessario abilitare la virtualizzazione nel BIOS.  
A questo punto è possibile procedere con l'installazione di KVM:  

```sh
sudo apt update
```
```sh
sudo apt install qemu-kvm libvirt-daemon-system libvirt-clients bridge-utils virt-manager
```

Occorre inoltre aggiungersi ai gruppi opportuni per evitare problemi:

```sh
sudo adduser `id -un` libvirt
```
```sh
sudo adduser `id -un` kvm
```

A questo punto, dopo aver riavviato la macchina per applicare le modifiche, si può procedere a lanciare KVM e a creare una VM Ubuntu 20.04 necessaria per ROS2 Foxy, con 4Gb di RAM e 4 Cores.  
Lanciare quindi il seguente comando:

```sh
virt-manager
```

e installare il sistema operativo da file ISO scaricato dal sito ufficiale.


### Installazione delle librerie e dipendenze richieste da ROS2 Foxy

Sarà opportuno installare i compilatori e le librerie Python richieste. Eseguire quindi i seguenti comandi:

```sh
sudo apt update
```
```sh
sudo apt install build-essential cmake git python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall
```

### Download del Codice Sorgente di ROS

Inizializzazione `rosdep` che aiuta a installare le dipendenze di sistema per i sorgenti da compilare.

```sh
sudo rosdep init
```
```sh
rosdep update
```

### Creazione del Workspace Catkin
Per iniziare la configurazione dell'ambiente ROS, è necessaria la creazione di un workspace Catkin. Questo workspace fungerà da contenitore per i sorgenti e i pacchetti ROS. La creazione può essere effettuata con i seguenti comandi:

```sh
mkdir -p ~/ros2_foxy_ws/src
```
```sh
cd ~/ros2_foxy_ws
```

### Selezione dei Pacchetti
L'utilizzo di `rosinstall_generator` permette di selezionare specifiche parti di ROS per la compilazione e l'installazione. È possibile optare per l'installazione di componenti essenziali come `ros_comm` o per una versione più completa con `desktop-full`. Ad esempio:

```sh
rosinstall_generator foxy --rosdistro foxy --deps --tar > foxy-desktop.rosinstall
```
```sh
wstool init -j8 src foxy-desktop.rosinstall
```

### Compilazione di ROS
È fondamentale garantire che tutte le dipendenze dei pacchetti selezionati siano soddisfatte. Questo è possibile mediante l'utilizzo di `rosdep`:

```sh
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

Utilizzare `colcon`, lo strumento di build consigliato per ROS 2, per compilare i pacchetti nel workspace:

```sh
colcon build --symlink-install
```


### Processo di Disinstallazione e Ripristino

Nel caso in cui ROS sia stato installato attraverso `catkin_make_isolated` con l'opzione `--install`, i componenti si troveranno all'interno di una sottodirectory `install_isolated` del workspace. Per procedere con la rimozione di ROS, è sufficiente eliminare tale directory insieme al resto del workspace di compilazione:

```sh
cd ~/ros_catkin_ws
```
```sh
rm -rf build_isolated devel_isolated install_isolated
```

Pulizia e Nuova Configurazione: Per iniziare nuovamente il processo di installazione da un ambiente pulito, è consigliabile rimuovere qualsiasi configurazione residua, inclusi i riferimenti all'ambiente ROS presenti nel file `.bashrc`.  
Questi riferimenti possono includere variabili d'ambiente, modifiche al `PATH`, o sorgenti per script di setup generati da ROS. Per rimuoverli, è necessario editare il file `.bashrc` utilizzando un editor di testo e cancellare manualmente le righe pertinenti. Ad esempio, si potrebbero cercare e rimuovere linee simili a queste:

```sh
source /opt/ros/noetic/setup.bash
source ~/ros_catkin_ws/devel_isolated/setup.bash
```

Dopo aver apportato le modifiche, salvare il file e riavviare il terminale per assicurarsi che le modifiche abbiano effetto. Questo passaggio garantisce che l'ambiente del terminale sia privo di qualsiasi configurazione relativa a ROS, permettendo una nuova installazione o la configurazione di un ambiente differente in modo pulito e senza conflitti.
