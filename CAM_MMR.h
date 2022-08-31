/*
 * CAM_MMR.h
 *
 *  n-input 2-level folded look ahead MMR built upon 8-to-3 basic block
 */

#ifndef CAM_MMR_H_
#define CAM_MMR_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"
#include "typedef.h"
#include "CAM_BasicMMR.h"

class CAM_MMR: public FunctionUnit {
public:
	CAM_MMR();
	virtual ~CAM_MMR();

	/* Functions */
	void PrintProperty();
	void Initialize(int _numInputBits, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	CAM_MMR & operator=(const CAM_MMR &);

	/* Properties */
	bool initialized;			/* Initialization flag */
	int numInputBits;  			/* Number of input bits */
	BufferDesignTarget areaOptimizationLevel;
	int numBasicMMR;
	double capLoad;		/* Load capacitance, Unit: F */
	double resLoad;		/* Load resistance, Unit: ohm */
	int foldA, foldB;
	double LookAheadLatency;
	double rampInput, rampOutput;

	OutputDriver outputDriver;
	CAM_BasicMMR BasicMMR;
};


#endif /* CAM_MMR_H_ */
