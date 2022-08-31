#include "CAM_InputEncoder.h"
#include "global.h"
#include "formula.h"
#include "typedef.h"

CAM_InputEncoder::CAM_InputEncoder() {
	// TODO Auto-generated constructor stub
	initialized = false;
	capLoad = 0;
	resLoad = 0;
	typeEncoder = encoding_two_bit;
	isCustom = false;
	numInputBits = 2;
	capNandIn = 0;
	capNandOut = 0;
	widthNandN = 0;
	widthNandP = 0;
	rampInput = 0;
	rampOutput = 0;
}

CAM_InputEncoder::~CAM_InputEncoder() {
	// TODO Auto-generated destructor stub
}

void CAM_InputEncoder::Initialize(TypeOfInputEncoder _typeEncoder, bool _isCustom, double _capLoad, double _resLoad){
	if (initialized)
		cout << "[CAM_InputEncoder] Warning: Already initialized!" << endl;
	capLoad = _capLoad;
	resLoad = _resLoad;
	typeEncoder = _typeEncoder;
	isCustom = _isCustom;
	if(isCustom) {
		// TODO Customizable Input Encoder
		cout<<"[CAM_InputEncoder] Error: Customized input encoder is under development."<<endl;
		return;
	}
	else if(typeEncoder == encoding_two_bit) {
		numInputBits = 2;
		/* gate sizing: built up with 2-input nand */
		widthNandN = 2 * MIN_NMOS_SIZE * tech->featureSize;
		widthNandP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
		double logicEffort = (2+tech->pnSizeRatio) / (1+tech->pnSizeRatio);
		CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandIn, &capNandOut);
		outputDriver.Initialize(logicEffort, capNandOut, capLoad, resLoad, true, latency_first, 0);
		initialized = true;
	}
	else {
		// TODO Encoding scheme look up table
		cout<<"[CAM_InputEncoder] Error: Two-bit encoder in JSSC11 is the only scheme supported in this version."<<endl;
		return;
	}
}

void CAM_InputEncoder::CalculateArea(){
	if (!initialized) {
		cout << "[CAM_InputEncoder] Error: Require initialization first!" << endl;
	} else {
		if(isCustom) {
			// TODO Customizable Input Encoder
			cout<<"[CAM_InputEncoder] Error: Customized input encoder is under development."<<endl;
			return;
		}
		else if(typeEncoder == encoding_two_bit) {
			outputDriver.CalculateArea();
			height = outputDriver.height * numInputBits * 2;
			width = outputDriver.width;
			double hNand, wNand;
			CalculateGateArea(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &hNand, &wNand);
			width += wNand;
			height = MAX(height, hNand * numInputBits * 2);
			area = height * width;
		}
		else {
			// TODO Encoding scheme look up table
			cout<<"[CAM_InputEncoder] Error: Two-bit encoder in JSSC11 is the only scheme supported in this version."<<endl;
			return;
		}
	}
}

void CAM_InputEncoder::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_InputEncoder] Error: Require initialization first!" << endl;
	} else {
		if(isCustom) {
			// TODO Customizable Input Encoder
			cout<<"[CAM_InputEncoder] Error: Customized input encoder is under development."<<endl;
			return;
		}
		else if(typeEncoder == encoding_two_bit) {
			outputDriver.CalculateRC();
			CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandIn, &capNandOut);
		}
		else {
			// TODO Encoding scheme look up table
			cout<<"[CAM_InputEncoder] Error: Two-bit encoder in JSSC11 is the only scheme supported in this version."<<endl;
			return;
		}
	}
}

void CAM_InputEncoder::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_InputEncoder] Error: Require initialization first!" << endl;
	} else {

		if(isCustom) {
			// TODO Customizable Input Encoder
			cout<<"[CAM_InputEncoder] Error: Customized input encoder is under development."<<endl;
			return;
		}
		else if(typeEncoder == encoding_two_bit) {
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
	    	cap = capNandOut + outputDriver.capInput[0];
	    	tr = resPullDown * cap;
	    	readLatency = horowitz(tr, beta, rampInput, &rampInternal);
	    	outputDriver.CalculateLatency(rampInternal);
	    	readLatency += outputDriver.readLatency;
	    	writeLatency = readLatency;
	    	rampOutput = outputDriver.rampOutput;
		}
		else {
			// TODO Encoding scheme look up table
			cout<<"[CAM_InputEncoder] Error: Two-bit encoder in JSSC11 is the only scheme supported in this version."<<endl;
			return;
		}
	}
}

void CAM_InputEncoder::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_InputEncoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculatePower();
		if(isCustom) {
			// TODO Customizable Input Encoder
			cout<<"[CAM_InputEncoder] Error: Customized input encoder is under development."<<endl;
			return;
		}
		else if(typeEncoder == encoding_two_bit) {
			leakage = CalculateGateLeakage(NAND, 2, widthNandN, widthNandP, inputParameter->temperature, *tech) * tech->vdd * numInputBits * 2;
			leakage += outputDriver.leakage * numInputBits * 2;

			/* nand and the driver */
			double cap;
			cap = outputDriver.capInput[0] + capNandOut;
			readDynamicEnergy = numInputBits * 2 * cap * tech->vdd * tech->vdd;
			readDynamicEnergy += (numInputBits * 2 * outputDriver.readDynamicEnergy);
			writeDynamicEnergy = readDynamicEnergy;
		}
		else {
			// TODO Encoding scheme look up table
			cout<<"[CAM_InputEncoder] Error: Two-bit encoder in JSSC11 is the only scheme supported in this version."<<endl;
			return;
		}
	}
}

void CAM_InputEncoder::PrintProperty() {
	cout << "CAM_InputEncoder Properties:" << endl;
	FunctionUnit::PrintProperty();
}

CAM_InputEncoder & CAM_InputEncoder::operator=(const CAM_InputEncoder &rhs) {
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
	isCustom = rhs.isCustom;
	typeEncoder = rhs.typeEncoder;
	numInputBits = rhs.numInputBits;
	return *this;
}


