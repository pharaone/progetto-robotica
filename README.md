# Progetto Robotica – Sistema di Pick and Place con TIAGo

## Descrizione Generale

Il progetto ha come obiettivo lo sviluppo di un **framework di simulazione** che consenta al robot **TIAGo** di eseguire attività di **pick and place** in un ambiente simulato, utilizzando **ROS 2**, **Gazebo**, **Rviz** e marker visivi **ArUco**. L’intero sistema è costruito su una struttura modulare, composta da nodi ROS che si occupano della percezione, controllo e gestione del comportamento del robot.

---

## Obiettivi del Progetto

- Modellare il robot TIAGo e il suo ambiente in **Gazebo**.
- Rilevare e localizzare oggetti tramite marker **ArUco**.
- Gestire trasformazioni tra frame di riferimento con TF2.
- Pianificare e controllare i movimenti del robot (navigazione e manipolazione).
- Eseguire operazioni di **pick and place** tramite cinematica e controllo.
- Coordinare il comportamento del robot tramite una **macchina a stati**.

---

## Tools e Conoscenze Utilizzate

- **ROS 2** (Robot Operating System) per la gestione dei nodi, messaggi e azioni.
- **Gazebo** per la simulazione fisica e visuale del robot TIAGo.
- **Rviz** per la visualizzazione di dati e debug.
- **OpenCV + ArUco** per la percezione visiva e stima della posa degli oggetti.
- **Cinematica diretta/inversa** per la manipolazione robotica.
- **Finite State Machine** per la gestione dei comportamenti complessi.
- Programmazione in **Python**.

---

## Sfide Principali Affrontate

### Task 1 – Modellazione e Percezione

- **Modellazione del robot in Gazebo**: personalizzazione del modello URDF e simulazione realistica del robot TIAGo.
- **Localizzazione degli oggetti**: utilizzo di marker ArUco e visione artificiale per stimare la posizione 3D degli oggetti nella scena.
- **Gestione delle trasformazioni**: utilizzo del sistema TF2 per convertire le coordinate tra i vari frame (camera, base, manipolatore).

### Task 2 – Controllo e Movimento

- **Pianificazione del movimento**: definizione di traiettorie per raggiungere gli oggetti ed eseguire il place.
- **Inversione cinematica**: calcolo dei giunti necessari per posizionare l’end effector in un punto desiderato.
- **Controllo del movimento**: attuazione dei comandi in modo fluido e sicuro, rispettando i vincoli del robot.

---

## Struttura del Sistema

### `aruco_pose_estimator_node.py`  
Nodo responsabile della percezione visiva:
- Acquisisce immagini dalla camera.
- Rileva i marker ArUco.
- Calcola e pubblica la posa dei marker.

### `kinematic1.py`  
Modulo per la cinematica del robot:
- Funzioni di cinematica diretta e inversa.
- Calcolo delle pose e trasformazioni per il braccio manipolatore.

### `head_action_server.py` & `head_action_client.py`  
Controllo della testa tramite azioni ROS:
- Il server esegue movimenti richiesti.
- Il client invia comandi e gestisce il feedback.

### `robot_state_machine.py`  
Cuore decisionale del robot:
- Implementa una macchina a stati finiti.
- Gestisce la transizione tra fasi come "cerca oggetto", "raggiungi", "afferra", "posa".

### `action_app.launch.py`  
Script di lancio:
- Avvia tutti i nodi necessari in modo coordinato.
- Prepara l'ambiente per l'esecuzione autonoma del task.

## Conclusione

Il progetto rappresenta un sistema completo e integrato per il controllo simulato del robot TIAGo in un’operazione di pick and place. Tutti i moduli sono stati sviluppati per essere riutilizzabili, estensibili e coerenti con le buone pratiche di programmazione ROS 2. La sfida principale è stata coordinare percezione, movimento e logica comportamentale in modo sinergico all’interno di un ambiente simulato realistico.

