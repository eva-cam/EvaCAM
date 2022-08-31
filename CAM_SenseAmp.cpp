#include "SenseAmp.h"
#include "CAM_SenseAmp.h"
#include "CAM_CustomSA.h"
#include "formula.h"
#include "global.h"
#include "typedef.h"

CAM_SenseAmp::CAM_SenseAmp() {
	// TODO Auto-generated constructor stub
	initialized = false;
	invalid = false;
	typeSA = nvsim_voltage_sense;
	isCustom = false;
	senseVoltage = 0;
	capLoad = 0;
	pitchSenseAmp = 0;
}

CAM_SenseAmp::~CAM_SenseAmp() {
	// TODO Auto-generated destructor stub
}

void CAM_SenseAmp::Initialize(long long _numColumn, TypeOfSenseAmp _typeSA, bool _isCustom, double _senseVoltage, double _pitchSenseAmp) {
	if (initialized)
		cout << "[CAM_SenseAmp] Warning: Already initialized!" << endl;
	typeSA = _typeSA;
	isCustom = _isCustom;
	senseVoltage = _senseVoltage;
	pitchSenseAmp = _pitchSenseAmp;
	numColumn = _numColumn;

	if (pitchSenseAmp <= tech->featureSize * 2) {
		/* too small, cannot do the layout */
		cout << "[CAM_SenseAmp] Warning: Pitch size too small, cannot do the layout!" << endl;
		invalid = true;
	}
	normalSenseAmp.Initialize(numColumn, typeSA == nvsim_current_sense, senseVoltage, pitchSenseAmp);
	if(isCustom == false && (typeSA != nvsim_voltage_sense && typeSA != nvsim_current_sense && typeSA != discharge ) ){
		normalSenseAmp.invalid = true;
		cout << "[CAM_Sense Amp] Error: input type not supported yet!" << endl;
		exit(-1);
	}
	if(isCustom) {
		FILE *fp = fopen(inputParameter->fileCustomSA.c_str(), "r");
		char line[5000];

		if (!fp) {
			cout << inputParameter->fileCustomSA << " custom SA file cannot be found!\n";
			exit(-1);
		}

		while (fscanf(fp, "%[^\n]\n", line) != EOF) {
			double tmp;
			if (!strncmp("-Height", line, strlen("-Height"))) {
				sscanf(line, "-Height (F): %lf", &tmp);
				customSA.height = tmp * tech->featureSize;
				continue;
			}
			if (!strncmp("-Width", line, strlen("-Width"))) {
				sscanf(line, "-Width (F): %lf", &tmp);
				customSA.width = tmp * tech->featureSize;
				continue;
			}
			if (!strncmp("-Area", line, strlen("-Area"))) {
				sscanf(line, "-Area (um): %lf", &tmp);
				customSA.area = tmp / 1e12;
				continue;
			}
			if (!strncmp("-Latency", line, strlen("-Latency"))) {
				sscanf(line, "-Latency (ps): %lf", &tmp);
				customSA.readLatency = tmp / 1e12;
				continue;
			}
			if (!strncmp("-Energy", line, strlen("-Energy"))) {
				sscanf(line, "-Energy (pJ): %lf", &tmp);
				customSA.readDynamicEnergy = tmp / 1e12;
				continue;
			}
			if (!strncmp("-Leakage", line, strlen("-Leakage"))) {
				sscanf(line, "-Leakage (pW): %lf", &tmp);
				customSA.leakage = tmp / 1e12;
				continue;
			}
			if (!strncmp("-CapLoad", line, strlen("-CapLoad"))) {
				sscanf(line, "-CapLoad (fF): %lf", &tmp);
				customSA.setLatency = tmp / 1e15;
				continue;
			}
		}
		if (customSA.area == 0) {
			customSA.area = customSA.height * customSA.width;
		}

	}
	initialized = true;
}

void CAM_SenseAmp::CalculateArea() {
	if (!initialized) {
		cout << "[CAM_SenseAmp] Error: Require initialization first!" << endl;
	} else if (invalid) {
		height = width = area = 1e41;
	} else {
		if (isCustom && customSA.area > 0) {
			if (customSA.height * customSA.width > 0) {
				height = customSA.height;
				width = customSA.width * numColumn;
			} else {
				height = sqrt(customSA.area);
				width = height * numColumn;
			}
			area = width * height;
		}
		else if (typeSA == nvsim_voltage_sense || typeSA == nvsim_current_sense || typeSA == discharge) {
			normalSenseAmp.CalculateArea();
			width = normalSenseAmp.width;
			height = normalSenseAmp.height;
			area = width * height;
		}
		else if (typeSA == self_clock_sense) {
			// TODO: self clock sense
			height = width = area = 1e41;
			cout << "[CAM_SenseAmp] Warning: Self clocked sensing is under development!" << endl;
		}
		else if (typeSA == dual_threshold_sense) {
			// TODO: self clock sense
			height = width = area = 1e41;
			cout << "[CAM_SenseAmp] Warning: Dual threshold sensing is under development!" << endl;
		}
		else {
			height = width = area = 1e41;
			cout << "[CAM_SenseAmp] Error: Sensing type entered is not supported yet!" << endl;
			exit(1);
		}
	}
}

void CAM_SenseAmp::CalculateRC() {
	if (!initialized) {
		cout << "[CAM_SenseAmp] Error: Require initialization first!" << endl;
	} else if (invalid) {
		readLatency = writeLatency = 1e41;
	} else {
		if (isCustom && customSA.setLatency > 0) {
			// custom design
			capLoad = customSA.setLatency;
		}
		else if (typeSA == nvsim_voltage_sense || typeSA == nvsim_current_sense || typeSA == discharge) {
			normalSenseAmp.CalculateRC();
			capLoad = normalSenseAmp.capLoad;
		}
		else if (typeSA == self_clock_sense) {
			// TODO: self clock sense
			capLoad = 1e41;
			cout << "[CAM_SenseAmp] Warning: Self clocked sensing is under development!" << endl;
		}
		else if (typeSA == dual_threshold_sense) {
			// TODO: self clock sense
			capLoad = 1e41;
			cout << "[CAM_SenseAmp] Warning: Dual threshold sensing is under development!" << endl;
		}
		else {
			capLoad = 1e41;
			cout << "[CAM_SenseAmp] Error: Sensing type entered is not supported yet!" << endl;
			exit(1);
		}
	}
}

void CAM_SenseAmp::CalculateLatency(double _rampInput) {	/* _rampInput is actually no use in SenseAmp */
	if (!initialized) {
		cout << "[Sense Amp] Error: Require initialization first!" << endl;
	} else {
		readLatency = writeLatency = 0;
		if (isCustom && customSA.readLatency > 0) {
			readLatency = customSA.readLatency;
			writeLatency = readLatency;
		}
		else if (typeSA == nvsim_voltage_sense || typeSA == nvsim_current_sense || typeSA == discharge) {
			normalSenseAmp.CalculateLatency(_rampInput);
			readLatency = normalSenseAmp.readLatency;
			writeLatency = normalSenseAmp.writeLatency;
		}
		else if (typeSA == self_clock_sense) {
			// TODO: self clock sense
			readLatency = writeLatency = 0;
			cout << "[CAM_SenseAmp] Warning: Self clocked sensing is under development!" << endl;
		}
		else if (typeSA == dual_threshold_sense) {
			// TODO: self clock sense
			readLatency = writeLatency = 0;
			cout << "[CAM_SenseAmp] Warning: Dual threshold sensing is under development!" << endl;
		}
		else {
			readLatency = writeLatency = 0;
			cout << "[CAM_SenseAmp] Error: Sensing type entered is not supported yet!" << endl;
			exit(1);
		}
	}
}

void CAM_SenseAmp::CalculatePower() {
	if (!initialized) {
		cout << "[Sense Amp] Error: Require initialization first!" << endl;
	} else if (invalid) {
		readDynamicEnergy = writeDynamicEnergy = leakage = 1e41;
	} else {
		readDynamicEnergy = writeDynamicEnergy = 0;
		leakage = 0;
		readLatency = writeLatency = 0;
		if (isCustom && customSA.readDynamicEnergy > 0) {
			normalSenseAmp.CalculatePower();
			readDynamicEnergy = customSA.readDynamicEnergy * numColumn;
			writeDynamicEnergy = readDynamicEnergy;
			if (customSA.leakage > 0) {
				leakage = customSA.leakage * numColumn;
			} else {
				leakage = normalSenseAmp.leakage;
			}
		}
		else if (typeSA == nvsim_voltage_sense || typeSA == nvsim_current_sense || typeSA == discharge) {
			normalSenseAmp.CalculatePower();
			readDynamicEnergy = normalSenseAmp.readDynamicEnergy;
			writeDynamicEnergy = normalSenseAmp.writeDynamicEnergy;
			leakage = normalSenseAmp.leakage;
		}
		else if (typeSA == self_clock_sense) {
			// TODO: self clock sense
			readDynamicEnergy = writeDynamicEnergy = leakage = 0;
			cout << "[CAM_SenseAmp] Warning: Self clocked sensing is under development!" << endl;
		}
		else if (typeSA == dual_threshold_sense) {
			// TODO: self clock sense
			readDynamicEnergy = writeDynamicEnergy = leakage = 0;
			cout << "[CAM_SenseAmp] Warning: Dual threshold sensing is under development!" << endl;
		}
		else {
			readDynamicEnergy = writeDynamicEnergy = leakage = 0;
			cout << "[CAM_SenseAmp] Error: Sensing type entered is not supported yet!" << endl;
			exit(1);
		}
	}
}

void CAM_SenseAmp::PrintProperty() {
	cout << "Sense Amplifier Properties:" << endl;
	FunctionUnit::PrintProperty();
}

CAM_SenseAmp & CAM_SenseAmp::operator=(const CAM_SenseAmp &rhs) {
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
	invalid = rhs.invalid;
	senseVoltage = rhs.senseVoltage;
	capLoad = rhs.capLoad;
	pitchSenseAmp = rhs.pitchSenseAmp;
	typeSA = rhs.typeSA;
	isCustom = rhs.isCustom;

	return *this;
}



