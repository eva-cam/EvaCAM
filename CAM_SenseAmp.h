/*
 * Totally brand new SA circuit. LUT based for existing design, while providing customizing interface
 */
#ifndef CAM_SENSEAMP_H_
#define CAM_SENSEAMP_H_

#include "FunctionUnit.h"
#include "SenseAmp.h"
#include "typedef.h"

class CAM_SenseAmp: public FunctionUnit {
public:
	CAM_SenseAmp();
	virtual ~CAM_SenseAmp();

	/* Functions */
	void PrintProperty();
	void Initialize(long long _numColumn, TypeOfSenseAmp _typeSA, bool _isCustom, double _senseVoltage /* Unit: V */, double _pitchSenseAmp);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	/* Note that this is a single SA, not yet multiplied by number of columns */
	CAM_SenseAmp & operator=(const CAM_SenseAmp &);
	FunctionUnit customSA;

	/* Properties */
	bool initialized;	/* Initialization flag */
	bool invalid;		/* Indicate that the current configuration is not valid */
	long long numColumn;
	TypeOfSenseAmp typeSA;			/* SA type loop up table index */
	bool isCustom;		/* Indicate whether the design is customized or not */
	SenseAmp normalSenseAmp;
	double senseVoltage;	/* Minimum sensible voltage */
	double capLoad;			/* Load capacitance of sense amplifier */
	double pitchSenseAmp;	/* The maximum width allowed for one sense amplifier layout */
};


#endif /* CAM_SENSEAMP_H_ */
