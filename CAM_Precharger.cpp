#include "CAM_Precharger.h"
#include "formula.h"
#include "global.h"
#include "constant.h"

CAM_Precharger::CAM_Precharger() {
	// TODO Auto-generated constructor stub
	initialized = false;
	enableLatency = 0;
}

CAM_Precharger::~CAM_Precharger() {
	// TODO Auto-generated destructor stub
}

void CAM_Precharger::Initialize(double _voltagePrecharge, int _numColumn, double _capBitline, double _resBitline){
	if (initialized)
		cout << "[Precharger] Warning: Already initialized!" << endl;

	voltagePrecharge = _voltagePrecharge;
	numColumn  = _numColumn;
	capBitline = _capBitline;
	resBitline = _resBitline;
	capWireLoadPerColumn = cell->widthInFeatureSize * tech->featureSize * localWire->capWirePerUnit;
	resWireLoadPerColumn = cell->widthInFeatureSize * tech->featureSize * localWire->resWirePerUnit;
	widthInvNmos = MIN_NMOS_SIZE * tech->featureSize;
	widthInvPmos = widthInvNmos * tech->pnSizeRatio;
	widthPMOSBitlineEqual      = MIN_NMOS_SIZE * tech->featureSize;
	widthPMOSBitlinePrecharger = PRECHARGER_SIZE * tech->featureSize;
	capLoadInv  = CalculateGateCap(widthPMOSBitlineEqual, *tech) + 2 * CalculateGateCap(widthPMOSBitlinePrecharger, *tech)
			+ CalculateDrainCap(widthInvNmos, NMOS, tech->featureSize*40, *tech)
			+ CalculateDrainCap(widthInvPmos, PMOS, tech->featureSize*40, *tech);
	capOutputBitlinePrecharger = CalculateDrainCap(widthPMOSBitlinePrecharger, PMOS, tech->featureSize*40, *tech) + CalculateDrainCap(widthPMOSBitlineEqual, PMOS, tech->featureSize*40, *tech);
	double capInputInv         = CalculateGateCap(widthInvNmos, *tech) + CalculateGateCap(widthInvPmos, *tech);
	capLoadPerColumn           = capInputInv + capWireLoadPerColumn;
	double capLoadOutputDriver = numColumn * capLoadPerColumn;
	outputDriver.Initialize(1, capInputInv, capLoadOutputDriver, 0 /* TO-DO */, true, latency_first, 0);  /* Always Latency First */

	initialized = true;
}

void CAM_Precharger::CalculateArea() {
	if (!initialized) {
		cout << "[Precharger] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateArea();
		double hBitlinePrechareger, wBitlinePrechareger;
		double hBitlineEqual, wBitlineEqual;
		double hInverter, wInverter;
		CalculateGateArea(INV, 1, 0, widthPMOSBitlinePrecharger, tech->featureSize*40, *tech, &hBitlinePrechareger, &wBitlinePrechareger);
		CalculateGateArea(INV, 1, 0, widthPMOSBitlineEqual, tech->featureSize*40, *tech, &hBitlineEqual, &wBitlineEqual);
		CalculateGateArea(INV, 1, widthInvNmos, widthInvPmos, tech->featureSize*40, *tech, &hInverter, &wInverter);
		// begin_change
		//width = 2 * wBitlinePrechareger + wBitlineEqual;
		width = 2 * wBitlinePrechareger;
		// end_change
		width = MAX(width, wInverter);
		width *= numColumn;
		width = MAX(width, outputDriver.width);
		// begin_charge
		//height = MAX(hBitlinePrechareger, hBitlineEqual);
		height = hBitlinePrechareger;
		// end_change
		height += hInverter;
		height += outputDriver.height;
		area = height * width;
	}
}

CAM_Precharger & CAM_Precharger::operator=(const CAM_Precharger &rhs) {
	height = rhs.height;
	width = rhs.width;
	area = rhs.area;
	readLatency = rhs.readLatency;
	writeLatency = rhs.writeLatency;
	readDynamicEnergy = rhs.readDynamicEnergy;
	writeDynamicEnergy = rhs.writeDynamicEnergy;
	resetLatency = rhs.resetLatency;
	setLatency = rhs.setLatency;
	resetDynamicEnergy = rhs.resetDynamicEnergy;
	setDynamicEnergy = rhs.setDynamicEnergy;
	cellReadEnergy = rhs.cellReadEnergy;
	cellSetEnergy = rhs.cellSetEnergy;
	cellResetEnergy = rhs.cellResetEnergy;
	leakage = rhs.leakage;
	initialized = rhs.initialized;
	outputDriver = rhs.outputDriver;
	capBitline = rhs.capBitline;
	resBitline = rhs.resBitline;
	capLoadInv = rhs.capLoadInv;
	capOutputBitlinePrecharger = rhs.capOutputBitlinePrecharger;
	capWireLoadPerColumn = rhs.capWireLoadPerColumn;
	resWireLoadPerColumn = rhs.resWireLoadPerColumn;
	enableLatency = rhs.enableLatency;
	numColumn = rhs.numColumn;
	widthPMOSBitlinePrecharger = rhs.widthPMOSBitlinePrecharger;
	widthPMOSBitlineEqual = rhs.widthPMOSBitlineEqual;
	capLoadPerColumn = rhs.capLoadPerColumn;
	rampInput = rhs.rampInput;
	rampOutput = rhs.rampOutput;

	return *this;
}


