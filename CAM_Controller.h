#ifndef CAM_CONTROLLER_H_
#define CAM_CONTROLLER_H_

#include "FunctionUnit.h"
#include "OutputDriver.h"

class CAM_Controller: public FunctionUnit {
public:
	CAM_Controller();
	virtual ~CAM_Controller();

	/* Functions */
	void PrintProperty();
	void Initialize(int _typeCtrl, bool _typeCustom);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	CAM_Controller & operator=(const CAM_Controller &);
	/* Note that this is a single write controller, not yet multiplied by number of columns */

	/* Properties */
	bool initialized;	/* Initialization flag */
	bool invalid;		/* Indicate that the current configuration is not valid */
	int typeCtrl;		/* Controller type loop up table index */
	bool typeCustom;	/* Indicate whether the design is customized or not */
};

#endif /* CAM_CONTROLLER_H_ */
