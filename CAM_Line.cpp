#include "CAM_Line.h"
#include "global.h"
#include "typedef.h"
#include "constant.h"
#include "CAM_Cell.h"
#include "formula.h"
#include "global.h"

CAM_Line::CAM_Line() {
	// TODO Auto-generated constructor stub
	initialized = false;
	invalid = false;
}

CAM_Line::~CAM_Line() {
	// TODO Auto-generated destructor stub
}

void CAM_Line::Initialize(bool _isRow, int _index, double _len, long long _numCell){
	if (initialized)
		cout << "[CAM_Line] Warning: Already initialized!" << endl;
	else {
		len = _len;
		isRow = _isRow;
		index = _index;
		CellPort.IsCol = CAM_cell->camPort[!isRow][index].IsCol;
		CellPort.ConnectedRegoin = CAM_cell->camPort[!isRow][index].ConnectedRegoin;
		CellPort.Type = CAM_cell->camPort[!isRow][index].Type;
		CellPort.isNMOS = CAM_cell->camPort[!isRow][index].isNMOS;
		CellPort.numCmos = CAM_cell->camPort[!isRow][index].numCmos;
		CellPort.volReset = CAM_cell->camPort[!isRow][index].volReset;
		CellPort.volSearch0 = CAM_cell->camPort[!isRow][index].volSearch0;
		CellPort.volSearch1 = CAM_cell->camPort[!isRow][index].volSearch1;
		CellPort.volSetLRS = CAM_cell->camPort[!isRow][index].volSetLRS;
		CellPort.volSetMRS = CAM_cell->camPort[!isRow][index].volSetMRS;
		CellPort.widthCmos = CAM_cell->camPort[!isRow][index].widthCmos;
		CellPort.widthWire = CAM_cell->camPort[!isRow][index].widthWire;
		numCell = _numCell;
		cap = len * localWire->capWirePerUnit;
		res = len * localWire->resWirePerUnit;
		if (CellPort.widthWire > 1.0) {
			cap = len * CalculateWireCapacitance(PERMITTIVITY, localWire->wireWidth*CellPort.widthWire,
					localWire->wireThickness, localWire->wireSpacing, localWire->ildThickness, 1.5,
					localWire->horizontalDielectric, 3.9, 1.15e-10);
			res = len * CalculateWireResistance(localWire->copper_resistivity, localWire->wireWidth*CellPort.widthWire,
					localWire->wireThickness, localWire->barrierThickness, 0, 1);
		}

		if (CellPort.ConnectedRegoin == gate) {
			if (CAM_cell->memCellType == FEFETRAM) {
				cap += CalculateGateCap(CellPort.widthCmos * FEFET_tech->featureSize, *FEFET_tech) * numCell * CellPort.numCmos;
			} else {
				cap += CalculateGateCap(CellPort.widthCmos * tech->featureSize, *tech) * numCell * CellPort.numCmos;
			}			
		} else if (CellPort.ConnectedRegoin == drain) {
			if (CAM_cell->memCellType == FEFETRAM){
				cap += CalculateDrainCap(CellPort.widthCmos * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech)
					* numCell * CellPort.numCmos;
			} else {
							cap += CalculateDrainCap(CellPort.widthCmos * tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech)
				 * numCell * CellPort.numCmos;
			}
				
		} else if (CellPort.ConnectedRegoin == diode) {
			if (CAM_cell->memCellType == FEFETRAM){
				cap += ( CalculateGateCap(CellPort.widthCmos * FEFET_tech->featureSize, *FEFET_tech) * numCell
						+ CalculateDrainCap(CellPort.widthCmos * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech) )
						* numCell * CellPort.numCmos;
			} else {
			cap += ( CalculateGateCap(CellPort.widthCmos * tech->featureSize, *tech) * numCell
					+ CalculateDrainCap(CellPort.widthCmos * tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech) )
					* numCell * CellPort.numCmos;
			}
		} else if (CellPort.ConnectedRegoin == source) { // it is source, which is the weird case in ISSCC'15 3t1r
			// TODO
			if (CAM_cell->memCellType == FEFETRAM){
				cap += CalculateDrainCap(CellPort.widthCmos * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech)
					* numCell * CellPort.numCmos;
			} else {
			cap += CalculateDrainCap(CellPort.widthCmos * tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech)
				 * numCell * CellPort.numCmos;
			}
		} else {
			cap += CalculateDrainCap(CellPort.widthCmos * tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech)
				 * numCell * CellPort.numCmos;
		}


		maxCurrent = 0;
		if (CellPort.Type == Wordline) {
			if (CellPort.ConnectedRegoin != gate) {
				// invalid = true;
				cout << "[CAM_Line] Warning: Impractical Wordline description!" << endl;
				return;
			}
		} else if (CellPort.Type == Bitline || CellPort.Type == Sourceline) {
			if (CellPort.ConnectedRegoin != drain && CellPort.ConnectedRegoin != none) {
				// invalid = true;
				cout << "[CAM_Line] Warning: Impractical Bitline/Sourceline description!" << endl;
				return;
			}
			if (CAM_cell->setMode) {
				maxCurrent = CAM_cell->setVoltage / CAM_cell->resistanceOff;
			} else {
				maxCurrent = CAM_cell->setCurrent;
			}
			if (CAM_cell->resetMode) {
				maxCurrent = MAX(maxCurrent, CAM_cell->resetVoltage / CAM_cell->resistanceOn);
			} else {
				maxCurrent = MAX(maxCurrent, CAM_cell->setCurrent);
			}
			maxCurrent += CAM_cell->leakageCurrentAccessDevice * (numCell - 1);
			if (CellPort.Type == Sourceline) { // the "column based resistor sourceline" in ISSCC'15 3t1r
				cout << "[CAM_Line] Warning: Make sure you are using Meng-Fan Chang's design" << endl;
				// it is special coded for Dr. Chang's design
				// for search operation: worst case, search all-0 or all-1
				// TODO: replace Vp to Vdd for coding convenience
				// TODO: the current is toooo large, have no idea, make it smaller
				double res = CalculateOnResistance(CAM_cell->widthAccessCMOS * tech->featureSize, NMOS, inputParameter->temperature, *tech)
						+ CAM_cell->resistanceOn;
				maxCurrent = MAX(maxCurrent, tech->vdd / res);
				maxCurrent = MAX(maxCurrent, MAX(CellPort.volSearch0,CellPort.volSearch1) / res);
			}
		} else if (CellPort.Type == Matchline || CellPort.Type == Matchline_Bitline) {
			// calc all miss
			if (CAM_cell->readMode && CAM_cell->readVoltage == 0) { // voltage sensing, current-in
				maxCurrent = CAM_cell->readCurrent;
			} else {
				// TODO
				// Note that no  such a large current, just large enough to recognize it is miss
				if (CAM_cell->accessType == CMOS_access) {
					maxCurrent = CellPort.widthCmos * tech->featureSize * (tech->currentOnNmos[inputParameter->temperature - 300]
					              + tech->currentOffNmos[inputParameter->temperature - 300] * (numCell - 1));
				} else if (CAM_cell->accessType == diode_access) {
					double res = CalculateOnResistance(CAM_cell->widthAccessCMOS * tech->featureSize, NMOS, inputParameter->temperature, *tech)
																+ CAM_cell->resistanceOn;
					maxCurrent = tech->vdd / res
					    + CellPort.widthCmos * tech->featureSize * tech->currentOffNmos[inputParameter->temperature - 300] * (numCell - 1);
				} else if (CAM_cell->accessType == none_access) {
					// TODO: too lazy to consider encoding here
				} else {
					double res = CalculateOnResistance(CAM_cell->widthAccessCMOS * tech->featureSize, NMOS, inputParameter->temperature, *tech)
											+ CAM_cell->resistanceOn;
					maxCurrent = tech->vdd / res * numCell;
						}
				}

		} else if (CAM_cell-> memCellType == FEFETRAM ||CellPort.Type == Searchline_Bitline){ // Design for 2FeFET TCAM
			//cout << "[CAM_Line] Warning: Make sure you are using 2FEFET TCAM design" << endl;
			if (CAM_cell->setMode) {
				maxCurrent = CAM_cell->setCurrent;
			}
			if (CAM_cell->resetMode) {
				maxCurrent = MAX(maxCurrent, CAM_cell->resetVoltage / CAM_cell->resistanceOn);
			} 
		}



			double maxCurrentBitline = 0;
			if (CellPort.Type == Matchline_Bitline) {
				// as bitline
				if (CAM_cell->setMode) {
					maxCurrentBitline = CAM_cell->setVoltage / CAM_cell->resistanceOff;
				} else {
					maxCurrentBitline = CAM_cell->setCurrent;
				}
				if (CAM_cell->resetMode) {
					maxCurrentBitline = MAX(maxCurrentBitline, CAM_cell->resetVoltage / CAM_cell->resistanceOn);
				} else {
					maxCurrentBitline = MAX(maxCurrentBitline, CAM_cell->setCurrent);
				}
				maxCurrentBitline += CAM_cell->leakageCurrentAccessDevice * (numCell - 1);
			}
			maxCurrent = MAX(maxCurrentBitline, maxCurrent);
		}

		if (isRow) {
			minMuxWidth = 0;
		} else {
			minMuxWidth = maxCurrent/ tech->currentOnNmos[inputParameter->temperature - 300];
		}
	

			initialized = true;
	}


void CAM_Line::Initialize(double _len, long long _numCell, double _MuxWidth){
	if (initialized)
		cout << "[CAM_Line] Warning: Already initialized!" << endl;
	else {
		len = _len;
		isRow = true;
		numCell = _numCell;
		cap = len * localWire->capWirePerUnit;
		res = len * localWire->resWirePerUnit;

		cap += CalculateGateCap(_MuxWidth * tech->featureSize, *tech) * numCell;
		maxCurrent = 1e11;
		minMuxWidth = 0;
		initialized = true;
	}
}


