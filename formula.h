#ifndef FORMULA_H_
#define FORMULA_H_

#include "Technology.h"
#include "constant.h"
#include <math.h>

#define MAX(a,b) (((a)> (b))?(a):(b))
#define MIN(a,b) (((a)< (b))?(a):(b))

bool isPow2(int n);

/* calculate the gate capacitance */
double CalculateGateCap(double width, Technology tech);

double CalculateGateArea(
		int gateType, int numInput,
		double widthNMOS, double widthPMOS,
		double heightTransistorRegion, Technology tech,
		double *height, double *width);

/* calculate the capacitance of a gate */
void CalculateGateCapacitance(
		int gateType, int numInput,
		double widthNMOS, double widthPMOS,
		double heightTransistorRegion, Technology tech,
		double *capInput, double *capOutput);

double CalculateDrainCap(
		double width, int type,
		double heightTransistorRegion, Technology tech);

double CAM_CalculateSourceCap(
		double width, int type,
		double heightTransistorRegion, Technology tech);

/* calculate the capacitance of a FBRAM */
double CalculateFBRAMGateCap(double width, double thicknessFactor, Technology tech);

double CalculateFBRAMDrainCap(double width, Technology tech);

double CalculateGateLeakage(
		int gateType, int numInput,
		double widthNMOS, double widthPMOS,
		double temperature, Technology tech);

double CalculateOnResistance(double width, int type, double temperature, Technology tech);

double CalculateTransconductance(double width, int type, Technology tech);

double horowitz(double tr, double beta, double rampInput, double *rampOutput);

double CalculateWireResistance(
		double resistivity, double wireWidth, double wireThickness,
		double barrierThickness, double dishingThickness, double alphaScatter);

double CalculateWireCapacitance(
		double permittivity, double wireWidth, double wireThickness, double wireSpacing,
		double ildThickness, double millarValue, double horizontalDielectric,
		double verticalDielectic, double fringeCap);


#endif /* FORMULA_H_ */
