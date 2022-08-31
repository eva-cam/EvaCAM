#include "CAM_BasicEncoder.h"
#include "formula.h"
#include "global.h"

CAM_BasicEncoder::CAM_BasicEncoder() {
	initialized = false;
	capLoad = resLoad = 0;
	numInputBit = numNorInput = numNorGate = 0;
	widthNorN = widthNorP = 0;
	capNorInput = capNorOutput = 0;
	rampInput = rampOutput = 0;
	widthN = widthP = 0;
	widthNandN = widthNandP = 0;
	capDyn = 0;
	capNandInput = capNandOutput = 0;
	capInvInput = capInvOutput = 0;
}

CAM_BasicEncoder::~CAM_BasicEncoder() {
	// TODO Auto-generated destructor stub
}

void CAM_BasicEncoder::Initialize(int _numInputBit, double _capLoad, double _resLoad){
	if (initialized)
		cout << "[CAM_BasicEncoder] Warning: Already initialized!" << endl;
	numInputBit = _numInputBit;
	capLoad = _capLoad;
	resLoad = _resLoad;
	if (numInputBit == 8) {
		//TODO: Assuming we only have drivers at carry-in, z-output has no drivers
		widthNorN = MIN_NMOS_SIZE * tech->featureSize;
		widthNorP = 2 * tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
		widthNandN = 2 * MIN_NMOS_SIZE * tech->featureSize;
		widthNandP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
		widthN = MIN_NMOS_SIZE * tech->featureSize;
		widthP = tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize;
		double logicEffortCarry = 2 / (1+tech->pnSizeRatio);
		double tmp;
		CalculateGateCapacitance(NOR, 8, widthN*2, widthP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &tmp, &capDyn);
		outputDriver.Initialize(logicEffortCarry, capDyn, capLoad, resLoad, true, latency_first, 0);
	}
	else {
		// TODO 4-to-2 and 2-to-1 encoder
		cout << "[CAM_BasicEncoder] Error: Only support 8-to-3 Encoder block by now!" << endl;
		return;
	}
	initialized = true;
}

void CAM_BasicEncoder::CalculateArea() {
	if (!initialized) {
		cout << "[CAM_BasicEncoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateArea();
		if (numInputBit == 8) {
			outputDriver.CalculateArea();
			// 4-input OR is 2 NOR2 + 1 NAND2
			// and the tre-state output by trans-gate
			double hNOR, wNOR, hNAND, wNAND, hTRI, wTRI;
			CalculateGateArea(NOR, 2, widthNorN, widthNorP, tech->featureSize*40, *tech, &hNOR, &wNOR);
			// assuming the second stage NAND is twice as large as the first stage NOR
			CalculateGateArea(NAND, 2, widthNandN*2, widthNandP*2, tech->featureSize*40, *tech, &hNAND, &wNAND);
			CalculateGateArea(INV, 1, widthN, widthP, tech->featureSize*40, *tech, &hTRI, &wTRI);
			// TODO: a better layout
			width = MAX(MAX(wNOR, wNAND), wTRI);
			height = hTRI + hNAND + hNOR * 2;
			height *= 3;
			area = height * width;

			// dynamic circuit for carry in
			double hPullDown, wPullDown, hCLK, wCLK;
			// the clock part
			CalculateGateArea(INV, 1, widthN * 2, widthP, tech->featureSize*40, *tech, &hCLK, &wCLK);
			// the pull down NMOS part
			CalculateGateArea(INV, 8, widthN * 2, 0, tech->featureSize*40, *tech, &hPullDown, &wPullDown);
			// TODO: a better layout
			area += ( hCLK*wCLK + hPullDown*wPullDown );
			height = area / width;
		} else {
			// TODO 4-to-2 and 2-to-1 encoder
			cout << "[CAM_BasicEncoder] Error: Only support 8-to-3 Encoder block by now!" << endl;
			return;
		}
	}
}

void CAM_BasicEncoder::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_BasicEncoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculateRC();
		CalculateGateCapacitance(NOR, 2, widthNorN, widthNorP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNorInput, &capNorOutput);
		CalculateGateCapacitance(NAND, 2, widthNandN, widthNandP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandInput, &capNandOutput);
		CalculateGateCapacitance(INV, 2, widthN, widthP, tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capInvInput, &capInvOutput);
	}
}

void CAM_BasicEncoder::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_BasicEncoder] Error: Require initialization first!" << endl;
	} else {
		rampInput = _rampInput;
		if (numInputBit == 8) {
			// TODO: the output data latency is not considered, since the carry in signal is much slower when array size larger than 8
        	double resPullDown;
        	double capLoad;
        	double tr;	/* time constant */
        	double gm;	/* transconductance */
        	double beta;	/* for horowitz calculation */
        	double rampInputForDriver;

        	resPullDown = CalculateOnResistance(widthN*2, NMOS, inputParameter->temperature, *tech);
        	// carry in also gives to the tri-state gate
        	capLoad = capDyn + outputDriver.capInput[0] + capInvInput;
        	tr = resPullDown * capLoad;
        	gm = CalculateTransconductance(widthNandN, NMOS, *tech);
        	beta = 1 / (resPullDown * gm);
        	readLatency = horowitz(tr, beta, rampInput, &rampInputForDriver);
        	outputDriver.CalculateLatency(rampInputForDriver);
        	readLatency += outputDriver.readLatency;
        	writeLatency = readLatency;
        	rampOutput = outputDriver.rampOutput;
		} else {
			// TODO 4-to-2 and 2-to-1 encoder
			cout << "[CAM_BasicEncoder] Error: Only support 8-to-3 Encoder block by now!" << endl;
			return;
		}
	}
}

void CAM_BasicEncoder::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_BasicEncoder] Error: Require initialization first!" << endl;
	} else {
		outputDriver.CalculatePower();
		if (numInputBit == 8) {
			double cap;
			leakage = outputDriver.leakage;
			readDynamicEnergy = 0;
			// leakage for OR gates and tri-state output
			leakage += ( CalculateGateLeakage(NOR, 2, widthNorN, widthNorP, inputParameter->temperature, *tech) * tech->vdd *2*3);
			leakage += ( CalculateGateLeakage(NAND, 2, widthNandN, widthNandP, inputParameter->temperature, *tech) * tech->vdd *3);
			leakage += ( CalculateGateLeakage(INV, 1, widthN, widthP, inputParameter->temperature, *tech) * tech->vdd *3);
			// leakage for the dynamic logic
			int tempIndex = (int)inputParameter->temperature - 300;
			if ((tempIndex > 100) || (tempIndex < 0)) {
				cout<<"Error: Temperature is out of range"<<endl;
				exit(-1);
			}
			double *leakP = tech->currentOffPmos;
			double leakageP = widthP * leakP[tempIndex];
			leakage += ( leakageP * tech->vdd );

			// dynamic for the dynamic logic
			cap = outputDriver.capInput[0] + capDyn + capInvInput;
			readDynamicEnergy += (cap * tech->vdd * tech->vdd);
			// dynamic power for the OR gates and the tri-state output

			cap = capNorOutput *2 + capNandInput + capInvInput;
			readDynamicEnergy += (cap * tech->vdd * tech->vdd * 3);
			writeDynamicEnergy = readDynamicEnergy;
		}  else {
			// TODO 4-to-2 and 2-to-1 encoder
			cout << "[CAM_BasicEncoder] Error: Only support 8-to-3 Encoder block by now!" << endl;
			return;
		}
	}
}

void CAM_BasicEncoder::PrintProperty() {
	cout << "8 to 3 CAM_BasicEncoder Properties:" << endl;
	FunctionUnit::PrintProperty();
}


