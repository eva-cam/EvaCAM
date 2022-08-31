/*
 * CAM_BasicMMR.h
 *  The multiple match resolver (MMR) for the priority encoder
 *  Adopting JSSC02, folding with Lookahead
 *  This block is just for 8 MMR block, with intra-LA
 */

#ifndef CAM_BASICMMR_H_
#define CAM_BASICMMR_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"
#include "typedef.h"

class CAM_BasicMMR: public FunctionUnit {
public:
	CAM_BasicMMR();
	virtual ~CAM_BasicMMR();

	/* Functions */
	void PrintProperty();
	void Initialize(int _numInputBits, double _capLoad, double _resLoad, double _capLaLoad, double _resLaLoad);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	/* Note that this is a single encoder, not yet multiplied by number of inputs */
	CAM_BasicMMR & operator=(const CAM_BasicMMR &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	int numInputBits;   /* Number of input bits */
	double capLAintra, capLAout, capD[4];
	double capInvIn, capInvOut;
	double capIn;
	double widthN, widthP;
	double capLoad, capLaLoad;		/* Load capacitance, Unit: F */
	double resLoad, resLaLoad;		/* Load resistance, Unit: ohm */
	double LookAheadLatency;
	OutputDriver LookAheadDriver;
	double rampInput, rampOutput;
	double rampLAout;
};



#endif /* CAM_MMR_H_ */
