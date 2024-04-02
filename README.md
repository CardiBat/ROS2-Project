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

Sarà opportuno installare i compilatori e le librerie Python richieste, oltre a python se non è installato sulla propria macchina. Eseguire quindi i seguenti comandi:

```sh
sudo apt update
```
```sh
sudo apt install build-essential cmake git python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall
```

Nel caso non fosse installato, installare python:

```sh
sudo apt update
```
```sh
sudo apt install python3 python3-pip
```

Infine, installare il compilatore colcon:
```sh
sudo apt update
```
```sh
pip3 install -U colcon-common-extensions
```

Settare quindi permanentemente la variabile d'ambiente. Aprire la shell:
```sh
nano ~/.bashrc
```
Aggiungere quindi la seguente riga a fine file, salvare e riaprire il terminale:
```sh
export PATH="$PATH:$HOME/.local/bin"
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
L'utilizzo di `rosinstall_generator` permette di selezionare specifiche parti di ROS da mettere nel file di configurazione .rosinstall. È possibile optare per l'inserimento di componenti essenziali come `ros_comm` o per una versione più completa con `desktop-full`. In ogni caso, bisogna inserire i pacchetti voluti al posto di PKGNAME. In generale quindi:

```sh
rosinstall_generator [PKGNAME] --rosdistro foxy --deps --tar > foxy-desktop.rosinstall
```
Un esempio potrebbe essere considerare pacchetti come rclcpp per la programmazione client in C++, rclpy per la programmazione client in Python, e example_interfaces che fornisce esempi di interfacce di servizio e messaggi che potrebbero essere utilizzati per esperimenti e apprendimento.

```sh
rosinstall_generator rclcpp rclpy example_interfaces --rosdistro foxy --deps --tar > foxy-desktop.rosinstall
```

Successivamente si scaricheranno i pacchetti presenti sul file di configurazione appena creato:

```sh
wstool init -j8 src foxy-desktop.rosinstall
```

### Compilazione di ROS
È fondamentale garantire che tutte le dipendenze dei pacchetti selezionati siano soddisfatte prima della compilazione. Questo è possibile mediante l'utilizzo di `rosdep`:

```sh
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

Utilizzando `colcon`, lo strumento di build consigliato per ROS 2, si compilano i pacchetti scaricati con le relative dipendenze:

```sh
colcon build --symlink-install
```


### Processo di Disinstallazione e Ripristino

Per eliminare i componenti di ROS 2, è sufficiente rimuovere il workspace di compilazione, che tipicamente include le directory `build`, `install`, e `log`. Supponendo che il workspace si trovi in `~/ros2_foxy_ws`, eseguire i seguenti comandi:

```sh
cd ~/ros2_foxy_ws
```
```sh
rm -rf build install log
```
Questo rimuoverà tutte le directory relative alla compilazione e all'installazione dei pacchetti ROS 2.



Per riconfigurare l'ambiente di sviluppo da uno stato pulito, è opportuno rimuovere eventuali riferimenti residui a ROS 2 presenti nel file di configurazione della shell, come `.bashrc` o `.zshrc`, a seconda della shell in uso. Questi riferimenti possono includere l'aggiunta del percorso di installazione di ROS 2 al `PATH`, nonché il sourcing di script di setup.

Aprire il file di configurazione della shell con un editor di testo:

```sh
nano ~/.bashrc  # oppure utilizzare ~/.zshrc per zsh
```

Ricercare e rimuovere le linee associate all'ambiente ROS 2, che potrebbero apparire come:

```sh
source /opt/ros/foxy/setup.bash
```
```sh
source ~/ros2_foxy_ws/install/local_setup.bash
```

Dopo aver apportato le modifiche, salvare il file e riavviare il terminale per assicurarsi che le modifiche abbiano effetto. Questo passaggio garantisce che l'ambiente del terminale sia privo di qualsiasi configurazione relativa a ROS, permettendo una nuova installazione o la configurazione di un ambiente differente in modo pulito e senza conflitti.


## Verso il porting su RISC-V: Utilizzo del cluster Monte Cimone

### Collegamento e Login 

Per iniziare a esercitarsi sull'utilizzo di un processore di tipo RISC, è disponibile un cluster con vari nodi denominato Monte Cimone. Questa macchina è accessibile dal proprio PC (previa registrazione da form) tramite il comando di SSH:

```sh
ssh username@beta.dei.unibo.it -p 2223
```
> output: username@mcimone-login:~$ 


Dopo aver inserito la password fornita dal docente, cambierà il proprio username dal terminale e verrà stampato il comando di `nodeinfo` (richiamabile anche successivamente per avere informazioni in tempo reale). La schermata mostrerà quindi tutti i nodi (disponibili se e solo se sono in stato di `IDLE`):  

<p>&nbsp;</p>
<div align="center">
  <img src="/nodes.png" alt="nodes.png">
</div>
<p>&nbsp;</p>

NOTA IMPORTANTE: Il login è basato su architettura Intel (e quindi CISC). Sarà quindi opportuno proseguire collegandosi a un nodo per raggiungere effettivamente l'architettura RISC-V



### Breve descrizione dei nodi e il loro uso

I nodi disponibili sono di due tipologie a seconda della partizione. Vi sono i `sifive-nodes` i quali sono composti da 4 Core ciascuno, mentre vi sono i `milkv-nodes` che possiedono fino a 64 core per ognuno di essi. Per tutto questo sistema è installato `Slurm`, un job scheduler open source in grado di organizzare l'avvio di processi in modo semplice.  

Per entrare rapidamente in un nodo e avviare un job, è sufficiente digitare il seguente comando (con nx = n° di core):

```sh
srun -n4 --pty bash
```

Questo comando è utile per utilizzare il primo nodo disponibile, ma se invece si vuole cercare ad esempio di usare un `milkv` allora bisogna seguire il processo completo, ossia allocazione, SSH, eventuale uso e infine deallocazione.  
Per allocare un nodo si usa il comando `salloc` seguito dal n° di core e dal selettore di partizione con relativo nome. 

salloc -n64 -p mcimone-milkvs

Se non si sanno i nomi delle partizioni è sufficiente digitare il comando `sinfo` per averne l'elenco. In ogni caso, dopo questa operazione è possibile scoprire quale nodo della partizione è stato allocato con `nodeinfo` e infine eseguire il comando:

```sh
ssh fsimoni@mcimone-milkv-1
```

Riuscendo così ad entrare con Secure Shell dentro al nodo. Da qui, è possibile riutilizzare `srun` per avviare un job. Per deallocare, invece, digitare il comando `squeue` per scoprire il PID del nodo allocato e infine il comando di cancellazione seguito da quest'ultimo.

```sh
scancel [PID]
```

Per completezza, viene riportato l'utilizzo generale di questi due comandi:

```sh
salloc -n <> -t <hours:minutes:seconds> [-p <>] [-w <>] [--exclusive]
```

```sh
srun -n <n_task_to_allocate> [-N <n_nodes_to_run> ] [-p <partitions>] [-w <specific_node>] [-t <hours:minutes:seconds>] [--pty] command
```

Per una guida più approfondita, guardare la [guida specifico di CIMONE](https://gitlab.com/ecs-lab/courses/lab-of-big-data/riscv-hpc-perf/-/blob/main/2_slurm.md?ref_type=heads) oppure la [documentazione ufficiale](https://slurm.schedmd.com/overview.html)



### Installare le dipendenze sulla macchina RISC-V

Riassumendo, le dipendenze necessarie per ROS2 sono:  

- build-essential
- cmake
- git
- python3
- python3-pip  
- python3-rosdep
- python3-rosinstall-generator
- python3-wstool
- python3-rosinstall  

Bisogna quindi verificarne la presenza sulla macchina RISC-V, eventualmente installarle e a questo punto si potrà procedere con il porting.  
_Nota importante_: Non si hanno privilegi di root per installare programmi sulla macchina che si usa, perciò eventuali programmi saranno da installare sulla propria directory Desktop e dovrà quindi essere aggiornata la variabile d'ambiente PATH

Runniamo il seguente comando:

```sh
dpkg -l | grep <nome_pacchetto>
```

Sostituendo `<nome_pacchetto>` con python3, git, cmake e build-essential. Notando la mancanza di rosdep, rosinstall-generator, wstool e rosinstall, essi saranno da installare nella directory corrente e da aggiungere al PATH.


