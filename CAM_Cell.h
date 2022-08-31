/*
 * CAM_Cell.h
 *  the change and tailoring for the nvsim cell class
 */

#ifndef CAM_CELL_H_
#define CAM_CELL_H_


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MemCell.h"
#include "typedef.h"
#include "constant.h"

using namespace std;

struct CAM_CellPort{
	bool IsCol;
	CAM_PortType Type;
	CAM_CmosRegion ConnectedRegoin;
	int numCmos;
	bool isNMOS;
	double widthCmos;
	double volSetLRS;
	double volReset;
	double volSetMRS;
	double volSearch0;
	double volSearch1;
	double widthWire;
	bool leak;
};


class CAM_MemCell: public MemCell {
public:
	CAM_MemCell();
	virtual ~CAM_MemCell();

	/* Functions */
	void ReadCellFromFile(const std::string & inputFile);
	void PrintCell();

	/* Properties */


	/////////////////    Globe parameters     ////////////////////////
	//MemCellType memCellType;	/* Memory cell type (like MRAM, PCRAM, etc.) */
	//int processNode;        /* Cell original process technology node, Unit: nm*/
	//double area;			/* Cell area, Unit: F^2 */
	//double aspectRatio;		/* Cell aspect ratio, H/W */
	//double widthInFeatureSize;	/* Cell width, Unit: F */
	//double heightInFeatureSize;	/* Cell height, Unit: F */
	//CellAccessType accessType;	/* Cell access type: CMOS, BJT, or diode */
	CellAccessType accessType;
	/* redefine here: 	(1) cmos: ml is connected with cmos, e.g., ISSCC15-3t1r
	 * 					(2) diode: ml is connected with diode, e.g., VLSIT12-4t2r
	 * 					(3) none: ml is connected with cell, e.g., JSSC11-2t2r
	 */

	int camNumRow;
	int camNumCol;
	CAM_CellPort camPort[2][MAX_PORT];
	/*
	 * camPort[0][x] are rows
	 * camPort[1][x] are cols
	 * for rows, in the order of WL->SL and within, in the order of gate->others
	 * for cols, in the order of ML->BL and within, in the order of drain->source
	 */
	bool camMLC;
	//double widthAccessCMOS;	/* The gate width of CMOS access transistor, Unit: F */
	double camWidthMatchTran;		/* The gate width of CMOS access transistor, Unit: F */

	/* Different between widthAccessCMOS and camWidthMatchLineCMOS
	 * (1) for SRAM-based TCAM, access cmos is the nmos, and the matchline cmos is the transgate
	 * (2) for the cmos based (ISSCC15-3t1r), the access cmos is for write, the matchline is the pulldown
	 * (3) for the diode based (VLSIT12-4t2r), the access cmos is for write, the matchline is the diode
	 * (4) for the none based (2t2r), the access cmos is for write, the matchine is zero.	 *
	 */

	////////////// For the MTJ/Memoristor/GST itself  ////////////////
	//double resistanceOn;	/* Turn-on resistance */
	//double resistanceOff;	/* Turn-off resistance */
	//double capacitanceOn;   /* Cell capacitance when memristor is on */
	//double capacitanceOff;  /* Cell capacitance when memristor is off */


	/////////////  For the SA (for both read and search) /////////////
	//bool   readMode;		/* true = voltage-mode, false = current-mode */
	//double minSenseVoltage; /* Minimum sense voltage */

	/////////////  For the Read Operations  //////////////////////////
	//double readVoltage;		/* Read voltage */
	//double readCurrent;		/* Read current */
	//double readPower;       /* Read power per bitline (uW)*/

	/////////////  For the Write Operations  //////////////////////////
	//bool   resetMode;		/* true = voltage-mode, false = current-mode */
	//double resetVoltage;	/* Reset voltage */
	//double resetCurrent;	/* Reset current */
	//double resetPulse;		/* Reset pulse duration (ns) */
	//double resetEnergy;     /* Reset energy per cell (pJ) */
	//bool   setMode;			/* true = voltage-mode, false = current-mode */
	//double setVoltage;		/* Set voltage */
	//double setCurrent;		/* Set current */
	//double setPulse;		/* Set pulse duration (ns) */
	//double setEnergy;       /* Set energy per cell (pJ) */
	//double voltageDropAccessDevice;  /* The voltage drop on the access device, Unit: V */
	//double leakageCurrentAccessDevice;  /* Reverse current of access device, Unit: uA */









//	/* Optional properties */
//	int stitching;			/* If non-zero, add stitching overhead for every x cells */
//	double gateOxThicknessFactor; /* The oxide thickness of FBRAM could be larger than the traditional SOI MOS */
//	double widthSOIDevice; /* The gate width of SOI device as FBRAM element, Unit: F*/
//	double capDRAMCell;		/* The DRAM cell capacitance if the memory cell is DRAM, Unit: F */
//	double widthSRAMCellNMOS;	/* The gate width of NMOS in SRAM cells, Unit: F */
//	double widthSRAMCellPMOS;	/* The gate width of PMOS in SRAM cells, Unit: F */
//
//	/* For memristor */
//	bool readFloating;      /* If unselected wordlines/bitlines are floating to reduce total leakage */
//	double resistanceOnAtSetVoltage; /* Low resistance state when set voltage is applied */
//	double resistanceOffAtSetVoltage; /* High resistance state when set voltage is applied */
//	double resistanceOnAtResetVoltage; /* Low resistance state when reset voltage is applied */
//	double resistanceOffAtResetVoltage; /* High resistance state when reset voltage is applied */
//	double resistanceOnAtReadVoltage; /* Low resistance state when read voltage is applied */
//	double resistanceOffAtReadVoltage; /* High resistance state when read voltage is applied */
//	double resistanceOnAtHalfReadVoltage; /* Low resistance state when 1/2 read voltage is applied */
//	double resistanceOffAtHalfReadVoltage; /* High resistance state when 1/2 read voltage is applied */
//	double resistanceOnAtHalfResetVoltage; /* Low resistance state when 1/2 reset voltage is applied */
//
//	/* For NAND flash */
//	double flashEraseVoltage;		/* The erase voltage, Unit: V, highest W/E voltage in ITRS sheet */
//	double flashPassVoltage;		/* The voltage applied on the unselected wordline within the same block during programming, Unit: V */
//	double flashProgramVoltage;		/* The program voltage, Unit: V */
//	double flashEraseTime;			/* The flash erase time, Unit: s */
//	double flashProgramTime;		/* The SLC flash program time, Unit: s */
//	double gateCouplingRatio;		/* The ratio of control gate to total floating gate capacitance */
};


#endif /* CAM_CELL_H_ */
