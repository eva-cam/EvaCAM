/*
 * Inherit Precharger.cpp in NVsim_origin
 * Modification: get rid of equalization circuit
 */
#ifndef CAM_PRECHARGER_H_
#define CAM_PRECHARGER_H_

#include "Precharger.h"

class CAM_Precharger: public Precharger {
public:
	CAM_Precharger();
	virtual ~CAM_Precharger();

	/* Functions */
	void Initialize(double _voltagePrecharge, int _numColumn, double _capBitline, double _resBitline);
	void CalculateArea();
	CAM_Precharger & operator=(const CAM_Precharger &);
};


#endif /* CAM_PRECHARGER_H_ */
