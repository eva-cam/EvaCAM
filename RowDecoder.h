#ifndef ROWDECODER_H_
#define ROWDECODER_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"
#include "typedef.h"

class RowDecoder: public FunctionUnit {
public:
	RowDecoder();
	virtual ~RowDecoder();

	/* Functions */
	void PrintProperty();
	void Initialize(int _numRow, double _capLoad, double _resLoad,
			bool _multipleRowPerSet, BufferDesignTarget _areaOptimizationLevel, double _minDriverCurrent);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	RowDecoder & operator=(const RowDecoder &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	bool invalid;      /*Invalidatio flag */
	OutputDriver outputDriver;
	int numRow;			/* Number of rows */
	bool multipleRowPerSet;		/* For cache design, whether a set is partitioned into multiple wordlines */
	int numNandInput;	/* Type of NAND, NAND2 or NAND3 */
	double capLoad;		/* Load capacitance, i.e. wordline capacitance, Unit: F */
	double resLoad;		/* Load resistance, Unit: ohm */
	BufferDesignTarget areaOptimizationLevel; /* 0 for latency, 2 for area */
	double minDriverCurrent; /* Minimum driving current should be provided */

	double widthNandN, widthNandP;
	double capNandInput, capNandOutput;
	double rampInput, rampOutput;
};

#endif /* ROWDECODER_H_ */
