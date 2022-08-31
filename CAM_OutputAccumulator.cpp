#include "CAM_OutputAccumulator.h"
#include "global.h"
#include "formula.h"

CAM_OutputAccumulator::CAM_OutputAccumulator() {
	// TODO Auto-generated constructor stub
	initialized = false;
	capLoad = 0;
	resLoad = 0;
	capNandIn = 0;
	capNandOut = 0;
	widthNandN = 0;
	widthNandP = 0;
	rampInput = 0;
	rampOutput = 0;
}

CAM_OutputAccumulator::~CAM_OutputAccumulator() {
	// TODO Auto-generated destructor stub
}

void CAM_OutputAccumulator::Initialize(double _capLoad, double _resLoad) {
	if (initialized)
		cout << "[CAM_OutputAccumulator] Warning: Already initialized!" << endl;
	/* structure: nand for out/out_last, nand for initial, FF (6 nand) */
	capLoad = _capLoad;
	resLoad = _resLoad;
	widthNandN = 2 * MIN_NMOS_SIZE * tech->featureSize;
	widthNandP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
	CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandIn, &capNandOut);

	/* logic effort: built up with 1+1+3 nand */
	double logicEffort = (2+tech->pnSizeRatio) / (1+tech->pnSizeRatio);
	logicEffort = pow(logicEffort,5.0);
	CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandIn, &capNandOut);
	outputDriver.Initialize(logicEffort, capNandOut, capLoad, resLoad, true, latency_first, 0);
	initialized = true;
}

void CAM_OutputAccumulator::CalculateArea(){
	if (!initialized) {
		cout << "[CAM_OutputAccumulator] Error: Require initialization first!" << endl;
	} else {
		// TODO: The exactly layout of accumulator is not described, just for magic folding
		outputDriver.CalculateArea();
		area = outputDriver.area;
		double hNand, wNand;
		CalculateGateArea(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &hNand, &wNand);
		area += (hNand * wNand * (1+1+6));
		height = hNand * 3;
		width = area / height;
	}
}

void CAM_OutputAccumulator::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_OutputAccumulator] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateRC();
		CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandIn, &capNandOut);
	}
}

void CAM_OutputAccumulator::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_OutputAccumulator] Error: Require initialization first!" << endl;
	} else {
		rampInput = _rampInput;
    	double resPullDown;
    	double cap;
    	double tr;		/* time constant */
    	double gm;		/* transconductance */
    	double beta;	/* for horowitz calculation */
    	double rampInternal;
    	/* nand and the driver */
    	resPullDown = CalculateOnResistance(widthNandN, NMOS, inputParameter->temperature, *tech);
    	gm = CalculateTransconductance(widthNandN, NMOS, *tech);
    	beta = 1 / (resPullDown * gm);
    	cap = capNandOut;
    	tr = resPullDown * cap;
    	readLatency = horowitz(tr, beta, rampInput, &rampInternal);
    	cap = capNandOut + capNandIn;
    	tr = resPullDown * cap;
    	readLatency += horowitz(tr, beta, rampInternal, &rampInternal);
    	cap = capNandOut + capNandIn;
    	tr = resPullDown * cap;
    	readLatency += horowitz(tr, beta, rampInternal, &rampInternal);
    	cap = capNandOut + capNandIn * 2;
    	tr = resPullDown * cap;
    	readLatency += horowitz(tr, beta, rampInternal, &rampInternal);
    	cap = capNandOut + capNandIn * 2;
    	tr = resPullDown * cap;
    	readLatency += horowitz(tr, beta, rampInternal, &rampInternal);
    	cap = capNandOut + capNandIn + outputDriver.capInput[0];
    	tr = resPullDown * cap;
    	readLatency += horowitz(tr, beta, rampInternal, &rampInternal);
    	outputDriver.CalculateLatency(rampInternal);
    	readLatency += outputDriver.readLatency;
    	rampOutput = outputDriver.rampOutput;
    	writeLatency = readLatency;
	}
}

void CAM_OutputAccumulator::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_OutputAccumulator] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculatePower();
		leakage = CalculateGateLeakage(NAND, 2, widthNandN, widthNandP, inputParameter->temperature, *tech) * tech->vdd * (1+1+6);
		leakage += outputDriver.leakage;

		/* nand and the driver */
		double cap;
		cap = outputDriver.capInput[0] + capNandOut * 7 + capNandIn * 11;
		readDynamicEnergy = cap * tech->vdd * tech->vdd;
		readDynamicEnergy += outputDriver.readDynamicEnergy;
		writeDynamicEnergy = readDynamicEnergy;
	}
}

void CAM_OutputAccumulator::PrintProperty() {
	cout << "CAM_OutputAccumulator Properties:" << endl;
	FunctionUnit::PrintProperty();
}

CAM_OutputAccumulator & CAM_OutputAccumulator::operator=(const CAM_OutputAccumulator &rhs) {
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
	rampInput = rhs.rampInput;
	rampOutput = rhs.rampOutput;
	resLoad = rhs.resLoad;
	capNandIn = rhs.capNandIn;
	capNandOut = rhs.capNandOut;
	widthNandN = rhs.widthNandN;
	widthNandP = rhs.widthNandP;
	rampInput = rhs.rampInput;
	rampOutput = rhs.rampOutput;
	return *this;
}



