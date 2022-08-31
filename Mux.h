#ifndef MUX_H_
#define MUX_H_

#include "FunctionUnit.h"
#include "constant.h"

class Mux: public FunctionUnit {
public:
	Mux();
	virtual ~Mux();
	/* Functions */
	void PrintProperty();
	void Initialize(int _numInput, long long _numMux, double _capLoad, double _capInputNextStage, double _minDriverCurrent);
	void CalculateArea();
	void CalculateRC();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	Mux & operator=(const Mux &);

	/* Properties */
	bool initialized;	/* Initialization flag */
	int numInput;
	long long numMux;		/* Number of muxs in each row */
	double capLoad;
	double capInputNextStage;
	double minDriverCurrent;
    double capOutput;
	double widthNMOSPassTransistor;
	double resNMOSPassTransistor;
	double capNMOSPassTransistor;
	double capForPreviousDelayCalculation;
	double capForPreviousPowerCalculation;

	double rampInput, rampOutput;
};

#endif /* MUX_H_ */
