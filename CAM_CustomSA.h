/*
 * CAM_CustomSAArea.h
 *  custom SA design interface
 */

#ifndef CAM_CUSTOMSAAREA_H_
#define CAM_CUSTOMSAAREA_H_

#include "formula.h"
#include "global.h"
#include "constant.h"

extern void CalcAreaForCostomSA(int designNum, double widthTransistorRegion, Technology tech, double *width, double *height);
extern void CalcCapForCostomSA(int designNum, double widthTransistorRegion, Technology tech, double *CapLoad);

#endif /* CAM_CUSTOMSAAREA_H_ */
