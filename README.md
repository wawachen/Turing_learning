# Turing Learning Guidance
We investigate how an e-puck2 robot can use Turing Learning to infer the response of its laser rangefinder.  
## before the experiment
You need to download [Enki robot simulator](https://github.com/enki-community/enki), and use *cmake* and *make* operations to compile this folder. Then you can put other experiment folders you want to test into the root folder of **enki** and test them inside the **enki** folder. The makefile of each experiment needs to be changed for correct path.
 
 -----
## experiment design 
The sim-sim and sim-real experiments are tested.
For sim-sim experiment, it includes    

* TL_interactive:  interactive inference     
* TL_passive: passive inference with obstacle behaviours
* TL_source: original code
* Bluetooth_demo: demo for Bluetooth communication
* TL_phy_interactive: interactive inference with no random placement and orientation
* TL_phy_random: interactive inference with random movement.  
* TL_physical: interactive inference with physical experiment
* TL_noise: sensor readings with Gaussian noise

## data   
All the data is inside the data folder in each experiment. It contains the genes and fitness of classifiers and models.

## Bluetooth communication
Before you implement TL_physical, you have to ensure a successful connection between the robot and the PC.   
[Click here to get the instructions of Bluetooth communication](https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development)

