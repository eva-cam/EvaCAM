/*
 * CAM_PriorityEncoder.h
 * combine the MMR and the encoder as the priority encoder
 */

#ifndef CAM_PRIORITYENCODER_H_
#define CAM_PRIORITYENCODER_H_

#include "FunctionUnit.h"
#include "CAM_MMR.h"
#include "CAM_Encoder.h"

class CAM_PriorityEncoder: public FunctionUnit {
public:
	CAM_PriorityEncoder();
	virtual ~CAM_PriorityEncoder();

	/* Functions */
	void PrintProperty();
	void Initialize(int _numInputBits, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	CAM_PriorityEncoder & operator=(const CAM_PriorityEncoder &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	int numInputBits;   /* Number of input bits */
	BufferDesignTarget areaOptimizationLevel;

	double widthNorN, widthNorP;
	double capNorInput, capNorOutput;

	double capLoad;		/* Load capacitance, Unit: F */
	double resLoad;		/* Load resistance, Unit: ohm */
	double rampInput, rampOutput;

	CAM_MMR MMR;
	CAM_Encoder Encoder;
};



#endif /* CAM_PRIORITYENCODER_H_ */
