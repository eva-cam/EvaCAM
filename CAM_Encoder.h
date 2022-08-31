#ifndef CAM_ENCODER_H_
#define CAM_ENCODER_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"
#include "CAM_BasicEncoder.h"

class CAM_Encoder: public FunctionUnit {
public:
	CAM_Encoder();
	virtual ~CAM_Encoder();

	/* Functions */
	void PrintProperty();
	void Initialize(int _numInputBit, BufferDesignTarget _areaOptimizationLevel, double _capLoad, double _resLoad);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();

	/* Properties */
	bool initialized;	/* Initialization flag */
	double capLoad;		/* Load capacitance, Unit: F */
	double resLoad;		/* Load resistance, Unit: ohm */
	int numInputBit;
	double rampInput, rampOutput;

	int numStage;
	int numBasicEncoder;
	int numAdr;


	double widthNorN, widthNorP;
	double capNorInput, capNorOutput;
	double capInvInput, capInvOutput;

	double widthN, widthP;
	BufferDesignTarget areaOptimizationLevel;

	CAM_BasicEncoder BasicEncoder;
	OutputDriver outputDriver;

};


#endif /* CAM_ENCODER_H_ */
