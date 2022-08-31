#include "CAM_Encoder.h"
#include "formula.h"
#include "global.h"

CAM_Encoder::CAM_Encoder() {
	initialized = false;
	capLoad = resLoad = 0;
	numInputBit = 0;
	widthNorN = widthNorP = 0;
	capNorInput = capNorOutput = 0;
	rampInput = rampOutput = 0;
	widthN = widthP = 0;
	capInvInput = capInvOutput = 0;
	numStage = numBasicEncoder = numAdr = 0;
	areaOptimizationLevel = latency_first;
}

CAM_Encoder::~CAM_Encoder() {
	// TODO Auto-generated destructor stub
}

void CAM_Encoder::Initialize(int _numInputBit, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad){
	if (initialized)
		cout << "[CAM_Encoder] Warning: Already initialized!" << endl;
	numInputBit = _numInputBit;
	capLoad = _capLoad;
	resLoad = _resLoad;
	areaOptimizationLevel = _areaOptimizationLevel;
	if (numInputBit > pow(2,27)) {
		cout << "[CAM_Encoder] Error: Invalid number of subarray bits" <<endl;
		exit(-1);
	}
	numBasicEncoder = (int)(numInputBit/8) + ( (numInputBit%8)>0 );
	numAdr = (int)log2(numInputBit);
	double tmp = numBasicEncoder;
	numStage = 0;
	while(tmp >= 1) {
		numStage++;
		tmp /= 8;
		numBasicEncoder += ( (int)tmp + ( (int)(tmp)%8)>0 );
	}
	widthN = MIN_NMOS_SIZE * tech->featureSize;
	widthP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
	CalculateGateCapacitance(INV, 2, widthN, widthP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capInvInput, &capInvOutput);
	double logicEffort = (2+tech->pnSizeRatio) / (1+tech->pnSizeRatio);
	outputDriver.Initialize(logicEffort, capInvOutput, capLoad, resLoad, false, areaOptimizationLevel, 0);

	widthNorN = MIN_NMOS_SIZE * tech->featureSize;
	widthNorP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
	CalculateGateCapacitance(NOR, 2, widthNorN, widthNorP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNorInput, &capNorOutput);
	BasicEncoder.Initialize(8, capNorInput, 0 /*TODO gate resistance */);
	initialized = true;
}

void CAM_Encoder::CalculateArea() {
	if (!initialized) {
		cout << "[CAM_Encoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateArea();
		BasicEncoder.CalculateArea();

		area = outputDriver.area * numAdr + BasicEncoder.area * numBasicEncoder;

		// TODO: a better layout
		height = BasicEncoder.height * (int)(numInputBit/8);
		width = area / height;
	}
}

void CAM_Encoder::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_Encoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateRC();
		BasicEncoder.CalculateRC();
	}
}

void CAM_Encoder::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_Encoder] Error: Require initialization first!" << endl;
	} else {
		rampInput = _rampInput;
		BasicEncoder.CalculateLatency(rampInput);
		outputDriver.CalculateLatency(rampInput /* TODO not exactly */);
		double lastRamp = rampInput;
		for(int i=0;i<numStage;i++){
			BasicEncoder.CalculateLatency(lastRamp);
			lastRamp = BasicEncoder.rampOutput;
			readLatency += BasicEncoder.readLatency;
		}
		writeLatency = readLatency;
		rampOutput = outputDriver.rampOutput;
	}
}

void CAM_Encoder::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_Encoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculatePower();
		BasicEncoder.CalculatePower();
		leakage = outputDriver.leakage * numAdr + BasicEncoder.leakage * numBasicEncoder;
		readDynamicEnergy = outputDriver.readDynamicEnergy * numAdr + BasicEncoder.readDynamicEnergy * numBasicEncoder;
		writeDynamicEnergy = readDynamicEnergy;
	}
}

void CAM_Encoder::PrintProperty() {
	cout << "8 to 3 CAM_Encoder Properties:" << endl;
	FunctionUnit::PrintProperty();
}



