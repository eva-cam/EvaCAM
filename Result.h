#ifndef RESULT_H_
#define RESULT_H_

#include "BankWithHtree.h"
#include "BankWithoutHtree.h"
#include "Wire.h"

class Result {
public:
	Result();
	virtual ~Result();

	/* Functions */
	void print();
	void printAsCache(Result &tagBank, CacheAccessMode cacheAccessMode);
	void reset();
	void printToCsvFile(ofstream &outputFile);
	void printAsCacheToCsvFile(Result &tagBank, CacheAccessMode cacheAccessMode, ofstream &outputFile);
	void compareAndUpdate(Result &newResult);

	OptimizationTarget optimizationTarget;	/* Exploration should not be assigned here */

	Bank * bank;
	Wire * localWire;		/* TO-DO: this one has the same name as one of the global variables */
	Wire * globalWire;

	double limitReadLatency;			/* The maximum allowable read latency, Unit: s */
	double limitWriteLatency;			/* The maximum allowable write latency, Unit: s */
	double limitReadDynamicEnergy;		/* The maximum allowable read dynamic energy, Unit: J */
	double limitWriteDynamicEnergy;		/* The maximum allowable write dynamic energy, Unit: J */
	double limitReadEdp;				/* The maximum allowable read EDP, Unit: s-J */
	double limitWriteEdp;				/* The maximum allowable write EDP, Unit: s-J */
	double limitArea;					/* The maximum allowable area, Unit: m^2 */
	double limitLeakage;				/* The maximum allowable leakage power, Unit: W */
};

#endif /* RESULT_H_ */
