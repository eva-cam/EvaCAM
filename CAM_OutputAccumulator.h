#ifndef CAM_OUTPUTACCUMULATOR_H_
#define CAM_OUTPUTACCUMULATOR_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"

class CAM_OutputAccumulator: public FunctionUnit {
public:
	CAM_OutputAccumulator();
	virtual ~CAM_OutputAccumulator();

	/* Functions */
	void PrintProperty();
	void Initialize(double _capLoad, double _resLoad);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	/* Note that this is a single accumulator, not yet multiplied by number of inputs */
	CAM_OutputAccumulator & operator=(const CAM_OutputAccumulator &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	double capLoad;		/* Load capacitance, Unit: F */
	double resLoad;		/* Load resistance, Unit: ohm */

	double capNandIn, capNandOut;
	double widthNandN, widthNandP;
	OutputDriver outputDriver;
	double rampInput, rampOutput;
};

#endif /* CAM_OUTPUTACCUMULATOR_H_ */
