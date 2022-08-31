#include "CAM_Cell.h"
#include "formula.h"
#include "global.h"
#include "macros.h"
#include <math.h>

CAM_MemCell::CAM_MemCell() {
	// TODO Auto-generated constructor stub
	memCellType         = PCRAM;
	area                = 0;
	aspectRatio         = 0;
	resistanceOn        = 0;
	resistanceOff       = 0;
	readMode            = true;
	readVoltage         = 0;
	readCurrent         = 0;
	readPower           = 0;
    wordlineBoostRatio  = 1.0;
	resetMode           = true;
	resetVoltage        = 0;
	resetCurrent        = 0;
	minSenseVoltage     = 0.08;
	resetPulse          = 0;
	resetEnergy         = 0;
	setMode             = true;
	setVoltage          = 0;
	setCurrent          = 0;
	setPulse            = 0;
	accessType          = CMOS_access;
	processNode         = 0;
	setEnergy           = 0;

	/* Optional */
	stitching         = 0;
	gateOxThicknessFactor = 2;
	widthSOIDevice = 0;
	widthAccessCMOS   = 0;
	voltageDropAccessDevice = 0;
	leakageCurrentAccessDevice = 0;
	capDRAMCell		  = 0;
	widthSRAMCellNMOS = 2.08;	/* Default NMOS width in SRAM cells is 2.08 (from CACTI) */
	widthSRAMCellPMOS = 1.23;	/* Default PMOS width in SRAM cells is 1.23 (from CACTI) */

	/*For memristors */
	readFloating = false;
	resistanceOnAtSetVoltage = 0;
	resistanceOffAtSetVoltage = 0;
	resistanceOnAtResetVoltage = 0;
	resistanceOffAtResetVoltage = 0;
	resistanceOnAtReadVoltage = 0;
	resistanceOffAtReadVoltage = 0;
	resistanceOnAtHalfReadVoltage = 0;
	resistanceOffAtHalfReadVoltage = 0;

	/* for CAM */
	camNumRow = 0;
	camNumCol = 0;
	camMLC = false;
	camWidthMatchTran = 0.0;

}

CAM_MemCell::~CAM_MemCell() {
	// TODO Auto-generated destructor stub
}

void CAM_MemCell::ReadCellFromFile(const string & inputFile)
{
	FILE *fp = fopen(inputFile.c_str(), "r");
	char line[5000];
	char tmp[5000];
	int index;

	if (!fp) {
		cout << inputFile << " cannot be found!\n";
		exit(-1);
	}

	while (fscanf(fp, "%[^\n]\n", line) != EOF) {
		if (!strncmp("-MemCellType", line, strlen("-MemCellType"))) {
			sscanf(line, "-MemCellType: %s", tmp);
			if (!strcmp(tmp, "SRAM"))
				memCellType = SRAM;
			else if (!strcmp(tmp, "DRAM"))
				memCellType = DRAM;
			else if (!strcmp(tmp, "eDRAM"))
				memCellType = eDRAM;
			else if (!strcmp(tmp, "MRAM"))
				memCellType = MRAM;
			else if (!strcmp(tmp, "PCRAM"))
				memCellType = PCRAM;
			else if (!strcmp(tmp, "FBRAM"))
				memCellType = FBRAM;
			else if (!strcmp(tmp, "memristor"))
				memCellType = memristor;
			else if (!strcmp(tmp, "SLCNAND"))
				memCellType = SLCNAND;
			else if (!strcmp(tmp, "MLCNAND"))
				memCellType = MLCNAND;
			else 
				memCellType = FEFETRAM;
			continue;
		}
		if (!strncmp("-ProcessNode", line, strlen("-ProcessNode"))) {
			sscanf(line, "-ProcessNode: %d", &processNode);
			continue;
		}
		if (!strncmp("-CellArea", line, strlen("-CellArea"))) {
			sscanf(line, "-CellArea (F^2): %lf", &area);
			continue;
		}
		if (!strncmp("-CellAspectRatio", line, strlen("-CellAspectRatio"))) {
			sscanf(line, "-CellAspectRatio: %lf", &aspectRatio);
			heightInFeatureSize = sqrt(area * aspectRatio);
			widthInFeatureSize = sqrt(area / aspectRatio);
			continue;
		}

		if (!strncmp("-ResistanceOnAtSetVoltage", line, strlen("-ResistanceOnAtSetVoltage"))) {
			sscanf(line, "-ResistanceOnAtSetVoltage (ohm): %lf", &resistanceOnAtSetVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOffAtSetVoltage", line, strlen("-ResistanceOffAtSetVoltage"))) {
			sscanf(line, "-ResistanceOffAtSetVoltage (ohm): %lf", &resistanceOffAtSetVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOnAtResetVoltage", line, strlen("-ResistanceOnAtResetVoltage"))) {
			sscanf(line, "-ResistanceOnAtResetVoltage (ohm): %lf", &resistanceOnAtResetVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOffAtResetVoltage", line, strlen("-ResistanceOffAtResetVoltage"))) {
			sscanf(line, "-ResistanceOffAtResetVoltage (ohm): %lf", &resistanceOffAtResetVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOnAtReadVoltage", line, strlen("-ResistanceOnAtReadVoltage"))) {
			sscanf(line, "-ResistanceOnAtReadVoltage (ohm): %lf", &resistanceOnAtReadVoltage);
			resistanceOn = resistanceOnAtReadVoltage;
			continue;
		}
		if (!strncmp("-ResistanceOffAtReadVoltage", line, strlen("-ResistanceOffAtReadVoltage"))) {
			sscanf(line, "-ResistanceOffAtReadVoltage (ohm): %lf", &resistanceOffAtReadVoltage);
			resistanceOff = resistanceOffAtReadVoltage;
			continue;
		}
		if (!strncmp("-ResistanceOnAtHalfReadVoltage", line, strlen("-ResistanceOnAtHalfReadVoltage"))) {
			sscanf(line, "-ResistanceOnAtHalfReadVoltage (ohm): %lf", &resistanceOnAtHalfReadVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOffAtHalfReadVoltage", line, strlen("-ResistanceOffAtHalfReadVoltage"))) {
			sscanf(line, "-ResistanceOffAtHalfReadVoltage (ohm): %lf", &resistanceOffAtHalfReadVoltage);
			continue;
		}
		if (!strncmp("-ResistanceOnAtHalfResetVoltage", line, strlen("-ResistanceOnAtHalfResetVoltage"))) {
			sscanf(line, "-ResistanceOnAtHalfResetVoltage (ohm): %lf", &resistanceOnAtHalfResetVoltage);
			continue;
		}

		if (!strncmp("-ResistanceOn", line, strlen("-ResistanceOn"))) {
			sscanf(line, "-ResistanceOn (ohm): %lf", &resistanceOn);
			continue;
		}
		if (!strncmp("-ResistanceOff", line, strlen("-ResistanceOff"))) {
			sscanf(line, "-ResistanceOff (ohm): %lf", &resistanceOff);
			continue;
		}
		if (!strncmp("-CapacitanceOn", line, strlen("-CapacitanceOn"))) {
			sscanf(line, "-CapacitanceOn (F): %lf", &capacitanceOn);
			continue;
		}
		if (!strncmp("-CapacitanceOff", line, strlen("-CapacitanceOff"))) {
			sscanf(line, "-CapacitanceOff (F): %lf", &capacitanceOff);
			continue;
		}

		if (!strncmp("-GateOxThicknessFactor", line, strlen("-GateOxThicknessFactor"))) {
			sscanf(line, "-GateOxThicknessFactor: %lf", &gateOxThicknessFactor);
			continue;
		}

		if (!strncmp("-SOIDeviceWidth (F)", line, strlen("-SOIDeviceWidth (F)"))) {
			sscanf(line, "-SOIDeviceWidth (F): %lf", &widthSOIDevice);
			continue;
		}

		if (!strncmp("-ReadMode", line, strlen("-ReadMode"))) {
			sscanf(line, "-ReadMode: %s", tmp);
			if (!strcmp(tmp, "voltage"))
				readMode = true;
			else
				readMode = false;
			continue;
		}
		if (!strncmp("-ReadVoltage", line, strlen("-ReadVoltage"))) {
			sscanf(line, "-ReadVoltage (V): %lf", &readVoltage);
			continue;
		}
		if (!strncmp("-ReadCurrent", line, strlen("-ReadCurrent"))) {
			sscanf(line, "-ReadCurrent (uA): %lf", &readCurrent);
			readCurrent /= 1e6;
			continue;
		}
		if (!strncmp("-ReadPower", line, strlen("-ReadPower"))) {
			sscanf(line, "-ReadPower (uW): %lf", &readPower);
			readPower /= 1e6;
			continue;
		}
		if (!strncmp("-WordlineBoostRatio", line, strlen("-WordlineBoostRatio"))) {
			sscanf(line, "-WordlineBoostRatio: %lf", &wordlineBoostRatio);
			continue;
		}
		if (!strncmp("-MinSenseVoltage", line, strlen("-MinSenseVoltage"))) {
			sscanf(line, "-MinSenseVoltage (mV): %lf", &minSenseVoltage);
			minSenseVoltage /= 1e3;
			continue;
		}


		if (!strncmp("-ResetMode", line, strlen("-ResetMode"))) {
			sscanf(line, "-ResetMode: %s", tmp);
			if (!strcmp(tmp, "voltage"))
				resetMode = true;
			else
				resetMode = false;
			continue;
		}
		if (!strncmp("-ResetVoltage", line, strlen("-ResetVoltage"))) {
			sscanf(line, "-ResetVoltage (V): %lf", &resetVoltage);
			continue;
		}
		if (!strncmp("-ResetCurrent", line, strlen("-ResetCurrent"))) {
			sscanf(line, "-ResetCurrent (uA): %lf", &resetCurrent);
			resetCurrent /= 1e6;
			continue;
		}
		if (!strncmp("-ResetVoltage", line, strlen("-ResetVoltage"))) {
			sscanf(line, "-ResetVoltage (V): %lf", &resetVoltage);
			continue;
		}
		if (!strncmp("-ResetPulse", line, strlen("-ResetPulse"))) {
			sscanf(line, "-ResetPulse (ns): %lf", &resetPulse);
			resetPulse /= 1e9;
			continue;
		}
		if (!strncmp("-ResetEnergy", line, strlen("-ResetEnergy"))) {
			sscanf(line, "-ResetEnergy (pJ): %lf", &resetEnergy);
			resetEnergy /= 1e12;
			continue;
		}

		if (!strncmp("-SetMode", line, strlen("-SetMode"))) {
			sscanf(line, "-SetMode: %s", tmp);
			if (!strcmp(tmp, "voltage"))
				setMode = true;
			else
				setMode = false;
			continue;
		}
		if (!strncmp("-SetVoltage", line, strlen("-SetVoltage"))) {
			sscanf(line, "-SetVoltage (V): %lf", &setVoltage);
			continue;
		}
		if (!strncmp("-SetCurrent", line, strlen("-SetCurrent"))) {
			sscanf(line, "-SetCurrent (uA): %lf", &setCurrent);
			setCurrent /= 1e6;
			continue;
		}
		if (!strncmp("-SetVoltage", line, strlen("-SetVoltage"))) {
			sscanf(line, "-SetVoltage (V): %lf", &setVoltage);
			continue;
		}
		if (!strncmp("-SetPulse", line, strlen("-SetPulse"))) {
			sscanf(line, "-SetPulse (ns): %lf", &setPulse);
			setPulse /= 1e9;
			continue;
		}
		if (!strncmp("-SetEnergy", line, strlen("-SetEnergy"))) {
			sscanf(line, "-SetEnergy (pJ): %lf", &setEnergy);
			setEnergy /= 1e12;
			continue;
		}

		if (!strncmp("-AccessType", line, strlen("-AccessType"))) {
			sscanf(line, "-AccessType: %s", tmp);
			if (!strcmp(tmp, "CMOS"))
				accessType = CMOS_access;
			else if (!strcmp(tmp, "BJT"))
				accessType = BJT_access;
			else if (!strcmp(tmp, "diode"))
				accessType = diode_access;
			else if (!strcmp(tmp, "none"))
				accessType = none_access; //2 FeFET TCAM uses this access type
			continue;
		}

		if (!strncmp("-AccessCMOSWidth", line, strlen("-AccessCMOSWidth"))) {
				sscanf(line, "-AccessCMOSWidth (F): %lf", &widthAccessCMOS);
			continue;
		}

		if (!strncmp("-VoltageDropAccessDevice", line, strlen("-VoltageDropAccessDevice"))) {
			sscanf(line, "-VoltageDropAccessDevice (V): %lf", &voltageDropAccessDevice);
			continue;
		}

		if (!strncmp("-LeakageCurrentAccessDevice", line, strlen("-LeakageCurrentAccessDevice"))) {
			sscanf(line, "-LeakageCurrentAccessDevice (uA): %lf", &leakageCurrentAccessDevice);
			leakageCurrentAccessDevice /= 1e6;
			continue;
		}

		if (!strncmp("-DRAMCellCapacitance", line, strlen("-DRAMCellCapacitance"))) {
			if (memCellType != DRAM && memCellType != eDRAM)
				cout << "Warning: The input of DRAM cell capacitance is ignored because the memory cell is not DRAM." << endl;
			else
				sscanf(line, "-DRAMCellCapacitance (F): %lf", &capDRAMCell);
			continue;
		}

		if (!strncmp("-SRAMCellNMOSWidth", line, strlen("-SRAMCellNMOSWidth"))) {
			if (memCellType != SRAM)
				cout << "Warning: The input of SRAM cell NMOS width is ignored because the memory cell is not SRAM." << endl;
			else
				sscanf(line, "-SRAMCellNMOSWidth (F): %lf", &widthSRAMCellNMOS);
			continue;
		}

		if (!strncmp("-SRAMCellPMOSWidth", line, strlen("-SRAMCellPMOSWidth"))) {
			if (memCellType != SRAM)
				cout << "Warning: The input of SRAM cell PMOS width is ignored because the memory cell is not SRAM." << endl;
			else
				sscanf(line, "-SRAMCellPMOSWidth (F): %lf", &widthSRAMCellPMOS);
			continue;
		}


		if (!strncmp("-ReadFloating", line, strlen("-ReadFloating"))) {
			sscanf(line, "-ReadFloating: %s", tmp);
			if (!strcmp(tmp, "true"))
				readFloating = true;
			else
				readFloating = false;
			continue;
		}

		if (!strncmp("-FlashEraseVoltage (V)", line, strlen("-FlashEraseVoltage (V)"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of programming/erase voltage is ignored because the memory cell is not flash." << endl;
			else
				sscanf(line, "-FlashEraseVoltage (V): %lf", &flashEraseVoltage);
			continue;
		}

		if (!strncmp("-FlashProgramVoltage (V)", line, strlen("-FlashProgramVoltage (V)"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of programming/program voltage is ignored because the memory cell is not flash." << endl;
			else
				sscanf(line, "-FlashProgramVoltage (V): %lf", &flashProgramVoltage);
			continue;
		}

		if (!strncmp("-FlashPassVoltage (V)", line, strlen("-FlashPassVoltage (V)"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of pass voltage is ignored because the memory cell is not flash." << endl;
			else
				sscanf(line, "-FlashPassVoltage (V): %lf", &flashPassVoltage);
			continue;
		}

		if (!strncmp("-FlashEraseTime", line, strlen("-FlashEraseTime"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of erase time is ignored because the memory cell is not flash." << endl;
			else {
				sscanf(line, "-FlashEraseTime (ms): %lf", &flashEraseTime);
				flashEraseTime /= 1e3;
			}
			continue;
		}

		if (!strncmp("-FlashProgramTime", line, strlen("-FlashProgramTime"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of erase time is ignored because the memory cell is not flash." << endl;
			else {
				sscanf(line, "-FlashProgramTime (us): %lf", &flashProgramTime);
				flashProgramTime /= 1e6;
			}
			continue;
		}

		if (!strncmp("-GateCouplingRatio", line, strlen("-GateCouplingRatio"))) {
			if (memCellType != SLCNAND && memCellType != MLCNAND)
				cout << "Warning: The input of gate coupling ratio (GCR) is ignored because the memory cell is not flash." << endl;
			else {
				sscanf(line, "-GateCouplingRatio: %lf", &gateCouplingRatio);
			}
			continue;
		}

		//////////////////////////////////////////////////////////////////////////////////////////////
		if (!strncmp("-MLC", line, strlen("MLC"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of MLC is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-MLC: %s", tmp);
				if (!strcmp(tmp, "true"))
					camMLC = true;
				else
					camMLC = false;
				continue;

			}
			continue;
		}
		if (!strncmp("-MatchCMOSWidth", line, strlen("-MatchCMOSWidth"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of MatchCMOSWidth is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-MatchCMOSWidth (F): %lf", &camWidthMatchTran);
			}
			continue;
		}
		if (!strncmp("-NumRowPort", line, strlen("-NumRowPort"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of NumRowPort is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-NumRowPort: %d", &camNumRow);
				for(int i=0;i<camNumRow;i++) {
					camPort[0][i].IsCol = false;
				}
			}
			continue;
		}
		if (!strncmp("-RowPort:PortType", line, strlen("-RowPort:PortType"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-RowPort:PortType: %d:%s", &index, tmp);
				if (index > camNumRow) {
					cout << "Error: The input of RowPort index error." << endl;
					exit(1);
				}
				if (!strcmp(tmp, "Wordline"))
					camPort[0][index].Type = Wordline;
				else if (!strcmp(tmp, "Sourceline"))
					camPort[0][index].Type = Sourceline;
				else if (!strcmp(tmp, "Searchline"))
					camPort[0][index].Type = Searchline;
				else if (!strcmp(tmp, "Bitline"))
					camPort[0][index].Type = Bitline;
				else if (!strcmp(tmp, "Matchline"))
					camPort[0][index].Type = Matchline;
				else if (!strcmp(tmp, "Matchline_Bitline"))
					camPort[0][index].Type = Matchline_Bitline;
				else if (!strcmp(tmp, "Searchline_Bitline"))
					camPort[0][index].Type = Searchline_Bitline;
				else {
					cout << "Error: The input of RowPort:Type error." << endl;
					exit(1);
				}
			}
			continue;
		}
		if (!strncmp("-RowPort:CmosRegion", line, strlen("-RowPort:CmosRegion"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:CmosRegion is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-RowPort:CmosRegion: %d:%s", &index, tmp);
				if (index > camNumRow) {
					cout << "Error: The input of RowPort index error." << endl;
					exit(1);
				}
				if (!strcmp(tmp, "gate"))
					camPort[0][index].ConnectedRegoin = gate;
				else if (!strcmp(tmp, "source"))
					camPort[0][index].ConnectedRegoin = source;
				else if (!strcmp(tmp, "drain"))
					camPort[0][index].ConnectedRegoin = drain;
				else if (!strcmp(tmp, "diode"))
					camPort[0][index].ConnectedRegoin = diode;
				else if (!strcmp(tmp, "none"))
					camPort[0][index].ConnectedRegoin = none;
				else {
					cout << "Error: The input of RowPort:CmosRegion error." << endl;
					exit(1);
				}
			}
			continue;
		}
		if (!strncmp("-RowPort:numCmos", line, strlen("-RowPort:numCmos"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:numCmos is ignored because the memory is not CAM." << endl;
			else {
				int temp;
				sscanf(line, "-RowPort:numCmos: %d:%d", &index, &temp);
				camPort[0][index].numCmos = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:widthCmos", line, strlen("-RowPort:widthCmos"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:widthCmos is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:widthCmos (F): %d:%lf", &index, &temp);
				camPort[0][index].widthCmos = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:isNMOS", line, strlen("-RowPort:isNMOS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:isNMOS is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-RowPort:isNMOS: %d:%s", &index, tmp);
				if (!strcmp(tmp, "true")) {
					camPort[0][index].isNMOS = true;
				} else {
					camPort[0][index].isNMOS = false;
				}
			}
			continue;
		}
		if (!strncmp("-RowPort:volSetLRS", line, strlen("-RowPort:volSetLRS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:volSetLRS is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:volSetLRS (V): %d:%lf", &index, &temp);
				camPort[0][index].volSetLRS = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:volReset", line, strlen("-RowPort:volReset"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:volReset is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:volReset (V): %d:%lf", &index, &temp);
				camPort[0][index].volReset = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:volSetMRS", line, strlen("-RowPort:volSetMRS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:volSetMRS is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:volSetMRS (V): %d:%lf", &index, &temp);
				camPort[0][index].volSetMRS = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:volSearch0", line, strlen("-RowPort:volSearch0"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:volSearch0 is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:volSearch0 (V): %d:%lf", &index, &temp);
				camPort[0][index].volSearch0 = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:volSearch1", line, strlen("-RowPort:volSearch1"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:volSearch1 is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:volSearch1 (V): %d:%lf", &index, &temp);
				camPort[0][index].volSearch1 = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:widthWire", line, strlen("-RowPort:widthWire"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of RowPort:widthWire is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-RowPort:widthWire: %d:%lf", &index, &temp);
				camPort[0][index].widthWire = temp;
			}
			continue;
		}
		if (!strncmp("-RowPort:leak", line, strlen("-RowPort:leak"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:widthWire is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-RowPort:leak: %d:%s", &index, tmp);
				if (!strcmp(tmp, "true")) {
					camPort[0][index].leak = true;
				} else {
					camPort[0][index].leak = false;
				}
			}
			continue;
		}
		if (!strncmp("-NumColPort", line, strlen("-NumColPort"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of NumColPort is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-NumColPort: %d", &camNumCol);
				for(int i=0;i<camNumCol;i++) {
					camPort[1][i].IsCol = true;
				}
			}
			continue;
		}
		if (!strncmp("-ColPort:PortType", line, strlen("-ColPort:PortType"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-ColPort:PortType: %d:%s", &index, tmp);
				if (index > camNumCol) {
					cout << "Error: The input of ColPort index error." << endl;
					exit(1);
				}
				if (!strcmp(tmp, "Wordline"))
					camPort[1][index].Type = Wordline;
				else if (!strcmp(tmp, "Sourceline"))
					camPort[1][index].Type = Sourceline;
				else if (!strcmp(tmp, "Searchline"))
					camPort[1][index].Type = Searchline;
				else if (!strcmp(tmp, "Bitline"))
					camPort[1][index].Type = Bitline;
				else if (!strcmp(tmp, "Matchline"))
					camPort[1][index].Type = Matchline;
				else if (!strcmp(tmp, "Matchline_Bitline"))
					camPort[1][index].Type = Matchline_Bitline;
				else if (!strcmp(tmp, "Searchline_Bitline"))
					camPort[1][index].Type = Searchline_Bitline;
				else {
					cout << "Error: The input of ColPort:Type error." << endl;
					exit(1);
				}
			}
			continue;
		}
		if (!strncmp("-ColPort:CmosRegion", line, strlen("-ColPort:CmosRegion"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:CmosRegion is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-ColPort:CmosRegion: %d:%s", &index, tmp);
				if (index > camNumCol) {
					cout << "Error: The input of ColPort index error." << endl;
					exit(1);
				}
				if (!strcmp(tmp, "gate"))
					camPort[1][index].ConnectedRegoin = gate;
				else if (!strcmp(tmp, "source"))
					camPort[1][index].ConnectedRegoin = source;
				else if (!strcmp(tmp, "drain"))
					camPort[1][index].ConnectedRegoin = drain;
				else if (!strcmp(tmp, "diode"))
					camPort[1][index].ConnectedRegoin = diode;
				else if (!strcmp(tmp, "none"))
					camPort[1][index].ConnectedRegoin = none;
				else {
					cout << "Error: The input of ColPort:CmosRegion error." << endl;
					exit(1);
				}
			}
			continue;
		}
		if (!strncmp("-ColPort:numCmos", line, strlen("-ColPort:numCmos"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:numCmos is ignored because the memory is not CAM." << endl;
			else {
				int temp;
				sscanf(line, "-ColPort:numCmos: %d:%d", &index, &temp);
				camPort[1][index].numCmos = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:widthCmos", line, strlen("-ColPort:widthCmos"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:widthCmos is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:widthCmos (F): %d:%lf", &index, &temp);
				camPort[1][index].widthCmos = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:isNMOS", line, strlen("-ColPort:isNMOS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:isNMOS is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-ColPort:isNMOS: %d:%s", &index, tmp);
				if (!strcmp(tmp, "true")) {
					camPort[1][index].isNMOS = true;
				} else {
					camPort[1][index].isNMOS = false;
				}
			}
			continue;
		}
		if (!strncmp("-ColPort:volSetLRS", line, strlen("-ColPort:volSetLRS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:volSetLRS is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:volSetLRS (V): %d:%lf", &index, &temp);
				camPort[1][index].volSetLRS = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:volReset", line, strlen("-ColPort:volReset"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:volReset is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:volReset (V): %d:%lf", &index, &temp);
				camPort[1][index].volReset = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:volSetMRS", line, strlen("-ColPort:volSetMRS"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:volSetMRS is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:volSetMRS (V): %d:%lf", &index, &temp);
				camPort[1][index].volSetMRS = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:volSearch0", line, strlen("-ColPort:volSearch0"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:volSearch0 is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:volSearch0 (V): %d:%lf", &index, &temp);
				camPort[1][index].volSearch0 = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:volSearch1", line, strlen("-ColPort:volSearch1"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:volSearch1 is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:volSearch1 (V): %d:%lf", &index, &temp);
				camPort[1][index].volSearch1 = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:widthWire", line, strlen("-ColPort:widthWire"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:widthWire is ignored because the memory is not CAM." << endl;
			else {
				double temp;
				sscanf(line, "-ColPort:widthWire: %d:%lf", &index, &temp);
				camPort[1][index].widthWire = temp;
			}
			continue;
		}
		if (!strncmp("-ColPort:leak", line, strlen("-ColPort:leak"))) {
			if (inputParameter->designTarget != CAM_chip)
				cout << "Warning: The input of ColPort:widthWire is ignored because the memory is not CAM." << endl;
			else {
				sscanf(line, "-ColPort:leak: %d:%s", &index, tmp);
				if (!strcmp(tmp, "true")) {
					camPort[1][index].leak = true;
				} else {
					camPort[1][index].leak = false;
				}
			}
			continue;
		}
	}

	fclose(fp);
}



void CAM_MemCell::PrintCell()
{
	char *type[6];
	type[0] = (char*)"Wordline";
	type[1] = (char*)"Searchline";
	type[2] = (char*)"Bitline";
	type[3] = (char*)"Sourceline";
	type[4] = (char*)"Matchline";
	type[5] = (char*)"Matchline_Bitline";
	type[6] = (char*)"Searchline_Bitline";

	char *region[5];
	region[0] = (char*)"gate";
	region[1] = (char*)"source";
	region[2] = (char*)"drain";
	region[3] = (char*)"diode";
	region[4] = (char*)"none";
	char *is[2];
	is[1] = (char*)"Yes";
	is[0] = (char*)"No";
	MemCell::PrintCell();
	cout << "===========   For CAM  ==============" << endl;
	cout << "MLC used             : " << camMLC << endl;
	cout << "Match CMOS Width (F) : " << camWidthMatchTran << endl;
	for(int i=0;i<camNumRow;i++) {
		cout << "**** Row Port " << i << " ******" << endl;
		cout << "  -PortType          : " << type[camPort[0][i].Type] << endl;
		cout << "  -CMOS Region       : " << region[camPort[0][i].ConnectedRegoin] << endl;
		cout << "  -CMOS Number       : " << camPort[0][i].numCmos << endl;
		cout << "  -CMOS Width        : " << camPort[0][i].widthCmos << endl;
		cout << "  -NMOS              : " << is[camPort[0][i].isNMOS] << endl;
		cout << "  -Set LRS Voltage   : " << camPort[0][i].volSetLRS << endl;
		cout << "  -Set MRS Voltage   : " << camPort[0][i].volSetMRS << endl;
		cout << "  -Reset Voltage     : " << camPort[0][i].volReset << endl;
		cout << "  -Search 0 Voltage  : " << camPort[0][i].volSearch0 << endl;
		cout << "  -Search 1 Voltage  : " << camPort[0][i].volSearch1 << endl;
		cout << "  -wire width        : " << camPort[0][i].widthWire << endl;
	}

	for(int i=0;i<camNumCol;i++) {
		cout << "**** Col Port " << i << " ******" << endl;
		cout << "  -PortType          : " << type[camPort[1][i].Type] << endl;
		cout << "  -CMOS Region       : " << region[camPort[1][i].ConnectedRegoin] << endl;
		cout << "  -CMOS Number       : " << camPort[1][i].numCmos << endl;
		cout << "  -CMOS Width        : " << camPort[1][i].widthCmos << endl;
		cout << "  -NMOS              : " << is[camPort[1][i].isNMOS] << endl;
		cout << "  -Set LRS Voltage   : " << camPort[1][i].volSetLRS << endl;
		cout << "  -Set MRS Voltage   : " << camPort[1][i].volSetMRS << endl;
		cout << "  -Reset Voltage     : " << camPort[1][i].volReset << endl;
		cout << "  -Search 0 Voltage  : " << camPort[1][i].volSearch0 << endl;
		cout << "  -Search 1 Voltage  : " << camPort[1][i].volSearch1 << endl;
		cout << "  -wire width        : " << camPort[1][i].widthWire << endl;
	}
}


