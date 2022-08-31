#ifndef TECHNOLOGY_H_
#define TECHNOLOGY_H_

#include <iostream>
#include "typedef.h"

using namespace std;

class Technology {
public:
	Technology();
	virtual ~Technology();

	/* Functions */
	void PrintProperty();
	void Initialize(int _featureSizeInNano, DeviceRoadmap _deviceRoadmap);
	void InterpolateWith(Technology rhs, double _alpha);

	/* Properties */
	bool initialized;	/* Initialization flag */
	int featureSizeInNano; /*Process feature size, Unit: nm */
	double featureSize;	/* Process feature size, Unit: m */
	DeviceRoadmap deviceRoadmap;	/* HP, LSTP, or LOP */
	//MemCellType memCellType; /* support different parameters for FeFET */ 
	double vdd;			/* Supply voltage, Unit: V */
	double vth;				/* Threshold voltage, Unit: V */
	double vdsatNmos;		/* Velocity saturation voltage, Unit: V */
	double vdsatPmos;		/* Velocity saturation voltage, Unit: V */
	double phyGateLength;	/* Physical gate length, Unit: m */
	double capIdealGate;	/* Ideal gate capacitance, Unit: F/m */
	double capFringe;		/* Fringe capacitance, Unit: F/m */
	double capJunction;		/* Junction bottom capacitance, Cj0, Unit: F/m^2 */
	double capOverlap;		/* Overlap capacitance, Cover in MASTAR, Unit: F/m */
	double capSidewall;		/* Junction sidewall capacitance, Cjsw, Unit: F/m */
	double capDrainToChannel;	/* Junction drain to channel capacitance, Cjswg, Unit: F/m */
	double capOx;			/* Cox_elec in MASTAR, Unit: F/m^2 */
	double buildInPotential; /* Bottom junction built-in potential(PB in BSIM4 model), Unit: V */
	double effectiveElectronMobility;		 /* ueff for NMOS in MASTAR, Unit: m^2/V/s */
	double effectiveHoleMobility;            /* ueff for PMOS in MASTAR, Unit: m^2/V/s */
	double pnSizeRatio;		/* PMOS to NMOS size ratio */
	double effectiveResistanceMultiplier;	/* Extra resistance due to vdsat */
	double currentOnNmos[101];	/* NMOS saturation current, Unit: A/m */
	double currentOnPmos[101];	/* PMOS saturation current, Unit: A/m */
	double currentOffNmos[101];	/* NMOS off current (from 300K to 400K), Unit: A/m */
	double currentOffPmos[101]; /* PMOS off current (from 300K to 400K), Unit: A/m */

	double capPolywire;	/* Poly wire capacitance, Unit: F/m */


	double currentGmNmos;
	double currentGmPmos;
	double heightFin;
	double widthFin;
	double PitchFin;

};

#endif /* TECHNOLOGY_H_ */
