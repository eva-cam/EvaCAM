#include "CAM_MMR.h"
#include "global.h"
#include "formula.h"
#include <math.h>

CAM_MMR::CAM_MMR() {
	// TODO Auto-generated constructor stub
	initialized = false;
	capLoad = 0;
	resLoad = 0;
	numInputBits = 0;
	rampInput = rampOutput = 0;
	foldA = foldB = 0;
	LookAheadLatency = 0;
	numBasicMMR = 0;
	areaOptimizationLevel = latency_first;
}

CAM_MMR::~CAM_MMR() {
	// TODO Auto-generated destructor stub
}

void CAM_MMR::Initialize(int _numInputBits, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad){
	if (initialized)
		cout << "[CAM_MMR] Warning: Already initialized!" << endl;
	capLoad = _capLoad;
	resLoad = _resLoad;
	numInputBits = _numInputBits;
	areaOptimizationLevel = _areaOptimizationLevel;
	// 3-level folding partition
	// TODO: only 2-level folding is supported
	// TODO: only 8-to-3 basicMMR is supported
	numBasicMMR = (int)(numInputBits/8) + ( (numInputBits%8)>0 );
	foldA = (int)( (int)log2(numBasicMMR) - 3 )/2;
	foldB = (int)( (int)log2(numBasicMMR) - 3 - foldA)/2;
	foldA = (int)pow(2, foldA);
	foldB = (int)pow(2, foldB);

	CAM_BasicMMR tmp;
	tmp.Initialize(8, 0, 0, 0, 0);
	double logicEffort = 6 / (1+tech->pnSizeRatio);
	outputDriver.Initialize(logicEffort, tmp.capD[3], capLoad, resLoad, false, areaOptimizationLevel, 0);

	// 3-level folding, the LA is connected with four blocks
	// TODO: did not implement the NOR for 3 LA input (2-level folding) in the folding method
	BasicMMR.Initialize(8, outputDriver.capInput[0], 0, tmp.capInvIn*3 * 3, 0 /* TODO the gate resistance as zero*/);
	initialized = true;
}

void CAM_MMR::CalculateArea(){
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		area = 0;
		BasicMMR.CalculateArea();
		outputDriver.CalculateArea();
		area = BasicMMR.area * numBasicMMR + outputDriver.area;
		// TODO: a better layout may needed
		height = BasicMMR.height * numBasicMMR;
		width  = area / height;
	}
}

void CAM_MMR::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		BasicMMR.CalculateRC();
		outputDriver.CalculateRC();
	}
}

void CAM_MMR::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		rampInput = _rampInput;
		BasicMMR.CalculateLatency(rampInput);
		outputDriver.CalculateLatency(BasicMMR.rampOutput);
		// calculate longest path for the look ahead
		int longestLA = foldB-1 + foldA/foldB-1;
		double lastRamp = rampInput;
		LookAheadLatency = 0;
		for(int i=0;i<longestLA;i++){
			BasicMMR.CalculateLatency(lastRamp);
			lastRamp = BasicMMR.rampLAout;
			LookAheadLatency += BasicMMR.LookAheadLatency;
		}
		readLatency = LookAheadLatency + BasicMMR.readLatency;
		writeLatency = readLatency;
		rampOutput = outputDriver.rampOutput;
	}
}

void CAM_MMR::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		readDynamicEnergy = 0;
		BasicMMR.CalculatePower();
		outputDriver.CalculatePower();
		leakage = BasicMMR.leakage * numBasicMMR + outputDriver.leakage;
		readDynamicEnergy = BasicMMR.readDynamicEnergy * numBasicMMR + outputDriver.readDynamicEnergy;
		writeDynamicEnergy = readDynamicEnergy;
	}
}

void CAM_MMR::PrintProperty() {
	cout << "CAM_MMR Properties:" << endl;
	FunctionUnit::PrintProperty();
}

CAM_MMR & CAM_MMR::operator=(const CAM_MMR &rhs) {
	height = rhs.height;
	width = rhs.width;
	area = rhs.area;
	readLatency = rhs.readLatency;
	writeLatency = rhs.writeLatency;
	readDynamicEnergy = rhs.readDynamicEnergy;
	writeDynamicEnergy = rhs.writeDynamicEnergy;
	leakage = rhs.leakage;
	initialized = rhs.initialized;
	capLoad = rhs.capLoad;
	resLoad = rhs.resLoad;
	numInputBits = rhs.numInputBits;
	rampInput = rhs.rampInput;
	rampOutput = rhs.rampOutput;
	LookAheadLatency = rhs.LookAheadLatency;
	foldA = rhs.foldA;
	foldB = rhs.foldB;
	numBasicMMR = rhs.numBasicMMR;
	areaOptimizationLevel = rhs.areaOptimizationLevel;
	return *this;
}


