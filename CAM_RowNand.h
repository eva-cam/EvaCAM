/*
 * Used for last few level decoding, which are NAND gates
 * Inherit RowDecoder.cpp from NVsim_origin
 * Modification: control driver INV or Non-INV so that multi-stage decoder is possible, without NOR gates
 */
#ifndef CAM_ROWNAND_H_
#define CAM_ROWNAND_H_

#include "RowDecoder.h"


class CAM_RowNand: public RowDecoder {
public:
	CAM_RowNand();
	virtual ~CAM_RowNand();

	/* Functions */
	void Initialize(int _numRow, double _capLoad, double _resLoad,
			bool _multipleRowPerSet, bool _inv, BufferDesignTarget _areaOptimizationLevel, double _minDriverCurrent);
	CAM_RowNand & operator=(const CAM_RowNand &);

	/* Properties */
	bool driverInv;
};


#endif /* CAM_ROWNAND_H_ */
