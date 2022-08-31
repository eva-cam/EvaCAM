#include "FunctionUnit.h"
#include "constant.h"

FunctionUnit::FunctionUnit() {
	height = width = 0;
	area = 0;
	readLatency = writeLatency = 0;
	readDynamicEnergy = writeDynamicEnergy = 0;
	leakage = 0;

	resetLatency = setLatency = 0;
	resetDynamicEnergy = setDynamicEnergy = 0;
	cellReadEnergy = 0;
	cellSetEnergy = cellResetEnergy = 0;

}

FunctionUnit::~FunctionUnit() {
	// TODO Auto-generated destructor stub
}

void FunctionUnit::PrintProperty() {
	cout << "Area = " << height*1e6 << "um x " << width*1e6 << "um = " << area*1e6 << "mm^2" << endl;
	cout << "Timing:" << endl;
	cout << " -  Read Latency = " << readLatency*1e9 << "ns" << endl;
	cout << " - Write Latency = " << writeLatency*1e9 << "ns" << endl;
	cout << "Power:" << endl;
	cout << " -  Read Dynamic Energy = " << readDynamicEnergy*1e12 << "pJ" << endl;
	cout << " - Write Dynamic Energy = " << writeDynamicEnergy*1e12 << "pJ" << endl;
	cout << " - Leakage Power = " << leakage*1e3 << "mW" << endl;
}
