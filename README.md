# Microcontroller Cluster Scheduler
## About this code
This is a code used for a prototype presented in our first project discussion. The number of slaves is currently static, which is only 2 but can be increased into more by simply increasing
the number of maximum slaves and changing certain pins. <br>
Below is a simple diagram to show the selection of slaves. <br>
![simple_diagram](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/4a26c9c4-e456-445d-8c9d-b42451483e05)
#### Note: The CAN bus used is a single bus, it was made based on a document published by SIEMENS and was proved to be working.
## Prototype 
![WhatsApp Image 2024-02-14 at 17 17 46_ff84d01b](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/7477fc13-4c88-455c-a22a-fe64d5569142) <br>
In this prototype, the slaves take their applications from the attached arduinos, the aruino was used to mimic a memory due to the lack of memories in the market.
## Purpose
The goal of the scheduler is to:
1. Take operations from the interface microcontroller.
2. Give the operations to a certain number of slaves, the selected operation depends on priority.
3. Give the slave acces to the ROM, which contains the application, the selected slave depends on the priority of the operation it holds.
## Data structures used
![image](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/fc3710c0-ea38-4518-bbdc-3489fc8edf0b)
### How slaves are stored
- The scheduler monitors each slave through what is called a slave control block, (inspired from the process control block of an OS).
- The scheduler contains an array of these slave control blocks.
### How operations are stored
- An operation is stored in a priority queue, since each operation has its own priority.
- The same control block was used, an operation control block.
### Use of different data structures
- In order to maximize performance in managing slaves, different data structures used, including a stack and a priority queue.
- The stack stores the idle slaves.
- A priority queue is used to store the slaves which have an operation but are waiting their turn to access the ROM, the priority is that of the operation the slave has.
- In both these data structures, it is not the control block stored but simply the index of the slave.
- With this approach, the priority queue can be used for both the slave array and the operations array.
#### Slave control block and opeations control block structures
![image](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/6bb6a704-2f2f-42a3-93bd-84d7301e7ea4)
#### A simple visualzied algorithm
![image](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/be4181a9-2097-45bb-a98c-1709276ea196)
#### A state machine diagaram for the whole aglorithm
![state_machine](https://github.com/AhmadSameh/microcontroller_cluster_scheduler/assets/65093152/1fb29df9-1228-48f7-ab1f-44eacba3fee5)
