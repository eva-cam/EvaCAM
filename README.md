


# Eva-CAM

## Sections 
* Overview
* Eva-CAM Compilation
* Input Configuration
* Content Addressable Memory Types
* Search Functions



## Overview

<img width="1717" alt="EvaCAM_overview" src="https://github.com/user-attachments/assets/3a13673b-db1d-4700-99df-3218f0ed96df" />


* Eva-CAM is a circuit/architecture-level modeling and evaluation tool for content addressable memories (CAMs) that can project the area, timing, dynamic energy, and leakage power of NVM-based CAMs. The NVM technologies include FeFET, RRAM, STT-MRAM, PCM. Eva-CAM supports Ternary CAM (TCAM), Analog CAM (ACAM), and Multi-bit CAM (MCAM) designs implemented in non-volatile memories, for both exact and approximate match types. It also allows for the exploration of CAM array structures and sensing circuits. Eva-CAM has been validated with HSPICE simulation results and chip measurements. 
* For the version v1, we release the code for TCAM designs with exact match function. The updates for supporting Analog CAM and multi-bit CAM with best match and threshold match will be released soon. 

## Eva-CAM Compilation and Execution

* Eva-CAM is programmed under C++, and primarily developed on Linux OSes. It also can be compiled with other Unix-like OSes or Windows. The source code can be built using make:
  * $ make 
* To run Eva-CAM, two configuration files (xxx.cfg & xxx.cell) need to be provided. For more details about input configurations, see below. 
* After the users specify the configuration files, use the command ./Eva-CAM xxx.cfg to run
  * $ ./Eva-CAM xxx.cfg 
  * e.g. try ./Eva-CAM 2FeFET_TCAM.cfg.

## Input Configuration

### CAM cell characteristics
#### NVM device
* Device type: RRAM, PCM, STT-MRAM, FeFET
* On/Off resistance
* NVM read/write current or voltage
#### CAM cell specifications
* CAM types: TCAM, MCAM, ACAM
* Cell area (feature size), aspect ratio, technology node
* Wire configurations (e.g., BLs, WLs, SLs, MLs, etc.)
  * Access transistor size
  * Access transistor connect regions: gate, drain, diode, none(e.g., for 2FeFET design, no access transistor needed)
  * Wire width
  * Specific for MLs: isNVMdicharge (if NVM devices participate in the ML discharge process)
### Array specification
* Technology node and flavor (e.g., HP, LP)
* Process temperature
* Capacity
* Wordwidth
* SA types
* Mat/bank/mux specifications
* Optional peripherals, e.g., write drivers, output accumulator, priority encoder, output buffer, etc.
### Optimization 
* Overall optimization goal, e.g., latency, energy, area, EDP
* Wire interconnect optimization goal 

## Search Functions
<img width="654" alt="match_func_wv" src="https://github.com/user-attachments/assets/1f6255ad-46da-499b-a454-6716ba554708" />

*  Exact match: the stored entries that match exactly with the query
*  Threshold match: the stored entries whose distance is below specified thresholds.
*  Best match: the stored entry that has the smallest distance.

## Reference

* Please refer to the following paper for details, 

    Liu Liu, _et al._,"Eva-CAM: A Circuit/Architecture-Level Evaluation Tool for General Content Addressable Memories", DATE 2022.
    
 If you have any questions, suggestions, or comments, please feel free to contact us. 
 
 * Liu Liu (lliu24@nd.edu)

## Acknowledgement

For more details about bank organization or CAM cell configurations, please refer to the following papers, 

* S. Li _et al._, "NVSim-CAM: A circuit-level simulator for emerging nonvolatile memory based Content-Addressable Memory," ICCAD, 2016.
* Xiangyu Dong _et al._, “NVSim: A Circuit-Level Performance, Energy, and Area Model for Emerging Nonvolatile Memory”, TCAD, vol. 31, no. 7, 2012.



