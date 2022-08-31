


# Eva-CAM_v1

## Sections 
* Overview
* Eva-CAM Compilation
* Input Configuration
* Content Addressable Memory Types
* Search Functions



## Overview

![Eva-CAMorganization_new](https://user-images.githubusercontent.com/61231770/161645499-15faefd6-22d5-45d6-beca-f3e5f75f44b8.png)


* Eva-CAM is a circuit/architecture-level modeling and evaluation tool for content addressable memories (CAMs) that can project the area, timing, dynamic energy, and leakage power of NVM-based CAMs. The NVM technologies include FeFET, RRAM, STT-MRAM, PCM. Eva-CAM supports Ternary CAM (TCAM), Analog CAM (ACAM), and Multi-bit CAM (MCAM) designs implemented in non-volatile memories, for both exact and approximate match types. It also allows for the exploration of CAM array structures and sensing circuits. Eva-CAM has been validated with HSPICE simulation results and chip measurements. 
* For the version v1, we release the code for TCAM designs with exact match function. The updates for supporting Analog CAM and multi-bit CAM with best match and threshold match will be released soon. 

## Eva-CAM Compilation and Execution

* Eva-CAM is programmed under C++, and primarily developed on Linux OSes. It also can be compiled with other Unix-like OSes or Windows. The source code can be built using make:
  * $ make 
* To run Eva-CAM, two configuration files (xxx.cfg & xxx.cell) need to be provided. For more details about input configurations, see below. 
* After the users specify the configuration files, use the command ./Eva-CAM xxx.cfg to run
  * $ ./Eva-CAM xxx.cfg 

## Input Configuration
### Supported NVM devices
* Two-terminal: RRAM, PCM, STT-MRAM
* Three-terminal: FeFET


## Content Addressable Memory Types

## Search Functions

## Reference

* Please refer to the following paper for details, 

    Liu Liu, _et al._,"Eva-CAM: A Circuit/Architecture-Level Evaluation Tool for General Content Addressable Memories", DATE 2022.
    
 If you have any questions, suggestions, or comments, please feel free to contact us. 
 
 * Liu Liu (lliu24@nd.edu)

## Acknowledgement

For more details about bank organization or CAM cell configurations, please refer to the following papers, 

* S. Li _et al._, "NVSim-CAM: A circuit-level simulator for emerging nonvolatile memory based Content-Addressable Memory," ICCAD, 2016.
* Xiangyu Dong _et al._, “NVSim: A Circuit-Level Performance, Energy, and Area Model for Emerging Nonvolatile Memory”, TCAD, vol. 31, no. 7, 2012.



