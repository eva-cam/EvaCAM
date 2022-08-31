#ifndef BANK_H_
#define BANK_H_

#include "FunctionUnit.h"
#include "Mat.h"
#include "typedef.h"

class Bank: public FunctionUnit {
public:
	Bank();
	virtual ~Bank();

	/* Functions */
	void printbreakdown();
	void PrintProperty();
	//NCsim-CAM
	void debug();
	//End NVsim-CAM
	virtual void Initialize(int _numRowMat, int _numColumnMat, long long _capacity,
			long _blockSize, int _associativity, int _numRowPerSet, int _numActiveMatPerRow,
			int _numActiveMatPerColumn, int _muxSenseAmp, bool _internalSenseAmp, int _muxOutputLev1, int _muxOutputLev2,
			int _numRowSubarray, int _numColumnSubarray,
			int _numActiveSubarrayPerRow, int _numActiveSubarrayPerColumn,
			BufferDesignTarget _areaOptimizationLevel, MemoryType _memoryType, CAMType _camType, SearchFunction _searchFunction) = 0;
	virtual void CalculateArea() = 0;
	virtual void CalculateRC() = 0;
	virtual void CalculateLatencyAndPower() = 0;
	virtual Bank & operator=(const Bank &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	bool invalid;		/* Indicate that the current configuration is not valid, pass down to all the sub-components */
	bool internalSenseAmp;
	int numRowMat;		/* Number of mat rows in a bank */
	int numColumnMat;	/* Number of mat columns in a bank */
	long long capacity;		/* The capacity of this bank, Unit: bit */
	long blockSize;		/* The basic block size in this bank, Unit: bit */
	int associativity;	/* Associativity, for cache design only */
	int numRowPerSet;		/* For cache design, the number of wordlines which a set is partitioned into */
	int numActiveMatPerRow;	/* For different access types */
	int numActiveMatPerColumn;	/* For different access types */
	int muxSenseAmp;	/* How many bitlines connect to one sense amplifier */
	int muxOutputLev1;	/* How many sense amplifiers connect to one output bit, level-1 */
	int muxOutputLev2;	/* How many sense amplifiers connect to one output bit, level-2 */
	int numRowSubarray;		/* Number of subarray rows in a subarray  */
	int numColumnSubarray;	/* Number of subarray columns in a subarray  */
	int numActiveSubarrayPerRow;	/* For different access types */
	int numActiveSubarrayPerColumn;	/* For different access types */
	BufferDesignTarget areaOptimizationLevel;
	MemoryType memoryType;
	CAMType camType;
	SearchFunction searchFunction;

	double searchLatency;
	double searchDynamicEnergy;

	Mat mat;
	int numBitSerial;
};

#endif /* BANK_H_ */
