#include "InputParameter.h"
#include "Technology.h"
#include "MemCell.h"
#include "CAM_Cell.h"
#include "Wire.h"

extern InputParameter *inputParameter;
extern Technology *tech;
extern Technology *FEFET_tech;
extern MemCell *cell;
extern Wire *localWire;		/* The wire type of local interconnects (for example, wire in mat) */
extern Wire *globalWire;	/* The wire type of global interconnects (for example, the ones that connect mats) */
extern CAM_MemCell *CAM_cell;
extern CAM_Opt CAM_opt;

