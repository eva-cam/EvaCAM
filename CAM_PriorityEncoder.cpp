/*
 * CAM_PriorityEncoder.cpp
 *
 */

#include "CAM_PriorityEncoder.h"
#include "global.h"
#include "formula.h"

CAM_PriorityEncoder::CAM_PriorityEncoder() {
	// TODO Auto-generated constructor stub
	initialized = false;
	capLoad = 0;
	resLoad = 0;
	numInputBits = 0;
	rampInput = rampOutput = 0;
	areaOptimizationLevel = latency_first;
	capNorInput = capNorOutput = 0;
	widthNorN = widthNorP = 0;
}

CAM_PriorityEncoder::~CAM_PriorityEncoder() {
	// TODO Auto-generated destructor stub
}

void CAM_PriorityEncoder::Initialize(int _numInputBits, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad){
	if (initialized)
		cout << "[CAM_MMR] Warning: Already initialized!" << endl;
	capLoad = _capLoad;
	resLoad = _resLoad;
	numInputBits = _numInputBits;
	areaOptimizationLevel = _areaOptimizationLevel;

	Encoder.Initialize(numInputBits, areaOptimizationLevel, capLoad, resLoad);
	widthNorN = MIN_NMOS_SIZE * tech->featureSize;
	widthNorP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
	CalculateGateCapacitance(NOR, 2, widthNorN, widthNorP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNorInput, &capNorOutput);
	MMR.Initialize(numInputBits, areaOptimizationLevel, capNorInput, 0 /*TODO gate resistance */);
	initialized = true;
}

void CAM_PriorityEncoder::CalculateArea(){
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		area = 0;
		Encoder.CalculateArea();
		MMR.CalculateArea();
		area = Encoder.area + MMR.area;
		// TODO: a better layout may needed
		height = MMR.height;
		width  = area / height;
	}
}

void CAM_PriorityEncoder::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		MMR.CalculateRC();
		Encoder.CalculateRC();
	}
}

void CAM_PriorityEncoder::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		rampInput = _rampInput;
		MMR.CalculateLatency(rampInput);
		Encoder.CalculateLatency(MMR.rampOutput);
		readLatency = MMR.readLatency + Encoder.readLatency;
		writeLatency = readLatency;
		rampOutput = Encoder.rampOutput;
	}
}

void CAM_PriorityEncoder::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_MMR] Error: Require initialization first!" << endl;
	} else {
		readDynamicEnergy = 0;
		MMR.CalculatePower();
		Encoder.CalculatePower();
		leakage = MMR.leakage + Encoder.leakage;
		readDynamicEnergy = MMR.readDynamicEnergy + Encoder.readDynamicEnergy;
		writeDynamicEnergy = readDynamicEnergy;
	}
}

void CAM_PriorityEncoder::PrintProperty() {
	cout << "CAM_MMR Properties:" << endl;
	FunctionUnit::PrintProperty();
}

CAM_PriorityEncoder & CAM_PriorityEncoder::operator=(const CAM_PriorityEncoder &rhs) {
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
	areaOptimizationLevel = rhs.areaOptimizationLevel;
	widthNorN = rhs.widthNorN;
	widthNorP = rhs.widthNorP;
	capNorInput = rhs.capNorInput;
	capNorOutput = rhs.capNorOutput;
	return *this;
}







