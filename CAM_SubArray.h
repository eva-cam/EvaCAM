/*
 * Inherit SubArray.cpp from NVsim_origin
 * Modification: (1) different components (2) RC calculation on ML (3) provide sensing constraints
 * (4) result based on input and miss-rate patterns
 */

#ifndef CAM_SUBARRAY_H_
#define CAM_SUBARRAY_H_

#include "FunctionUnit.h"
#include "CAM_RowNand.h"
#include "CAM_Precharger.h"
#include "CAM_SenseAmp.h"
#include "CAM_DataBuffer.h"
#include "CAM_InputEncoder.h"
#include "CAM_OutputAccumulator.h"
#include "CAM_PriorityEncoder.h"
//#include "CAM_RefCircuit.h"
#include "CAM_Controller.h"
#include "RowDecoder.h"
#include "Mux.h"
#include "typedef.h"
#include "CAM_Line.h"

class CAM_SubArray: public FunctionUnit {
public:
	CAM_SubArray();
	virtual ~CAM_SubArray();

	/* Functions */
	void PrintProperty();
	void Initialize(long long _numRow, long long _numColumn, bool _multipleRowPerSet, bool _split,
			int _muxSenseAmp, bool _internalSenseAmp, int _muxOutputLev1, int _muxOutputLev2,
			BufferDesignTarget _RowDecMergeOptLevel, BufferDesignTarget _RowDriverOptLevel,
			bool _withInputEnc, TypeOfInputEncoder _typeInputEnc, bool _customInputEnc,
			TypeOfSenseAmp _typeSenseAmp, bool _customSenseAmp, bool _withWriteDriver,
			bool _withOutputAcc, bool _withPriorityEnc, BufferDesignTarget _PriorityOptLevel,
			bool _withInputBuf, bool _withOutputBuf, CAMType _camType, SearchFunction _searchFunction
	);
	void ReadCustomDesign(char* _fileInputEnc, char* _fileSenseAmp);
	void CalculateArea();
	void CalculateLatency(double _rampInput);
	void CalculatePower();
	CAM_SubArray & operator=(const CAM_SubArray &);

	/* Properties */
	CAM_DataBuffer inputBuf;
	CAM_DataBuffer outputBuf;

	CAM_InputEncoder inputEnc;
	CAM_RowNand	RowDecMergeNand;		/* Last level nand & driver for wordline predecoder */
	CAM_RowNand RowDriver[MAX_PORT];				/* nand & driver for wl/sl/etc, there could be multiple, e.g., ISSCC15 3T1R */

	CAM_Precharger	precharger;

	RowDecoder ColDecMergeNand;
	RowDecoder WriteDriver[MAX_PORT];
	Mux ColMux[MAX_PORT];				/*  mux for ml/bl, there are could be multiple or single, e.g., JSSC 2T2R */

	CAM_SenseAmp	senseAmp;

	RowDecoder	senseAmpMuxLev1Nand;
	Mux			senseAmpMuxLev1;
	RowDecoder	senseAmpMuxLev2Nand;
	Mux			senseAmpMuxLev2;

	CAM_OutputAccumulator outputAcc;
	CAM_PriorityEncoder priorityEnc;

	CAMType camType; /* For CAM type specification */
	SearchFunction searchFunction; /* For search function specifcation */


	bool initialized;	/* Initialization flag */
	bool invalid;		/* Indicate that the current configuration is not valid, pass down to all the sub-components */

	// structure
	long long numRow;			/* Number of rows, number of words*/
	long long numColumn;		/* Number of columns, word size */
	int muxSenseAmp;		/* How many bitlines connect to one sense amplifier */
	int muxOutputLev1;		/* How many sense amplifiers connect to one output bit, level-1 */
	int muxOutputLev2;		/* How many sense amplifiers connect to one output bit, level-2 */
	long long numSenseAmp;	/* Number of sense amplifiers */
	long long numInputEnc;

	double lenRow;
	double lenCol;
	int indexMatchline;
	int indexBitline;
	int numBitline;
	CAM_Line Row[MAX_PORT];
	CAM_Line Col[MAX_PORT];


	// for input and output buffer
	bool withInputBuf;
	bool withOutputBuf;


	// for write driver
	bool withWriteDriver;
	double WriteDriverArea;
	double WriteDriverLatency;
	double WriteDriverDyn;
	double WriteDriverLeakage;

	// for input encoder
	bool withInputEnc;
	TypeOfInputEncoder typeInputEnc;
	bool customInputEnc;


	// for row drivers
	BufferDesignTarget DecMergeOptLevel;
	bool RowDecMergeInv;
	BufferDesignTarget DriverOptLevel;


	// for precharger
	double voltagePrecharge;

	// for sense amplifier
	bool internalSenseAmp; 	/* Indicate whether sense amp is within subarray */
	bool voltageSense;		/* Whether the sense amplifier is voltage-sensing */
	double senseVoltage;	/* Minimum sensible voltage */
	TypeOfSenseAmp typeSenseAmp;
	bool customSenseAmp;
	double senseMargin;
	double referDelay;
	double volMatchDrop;
	double volAllMissDrop;

	double columnDecoderLatency;	/* The worst-case mux latency, Unit: s */
	double bitlineDelayOn;  /* Bitline delay of LRS, Unit: s */
	double bitlineDelayOff; /* Bitline delay of HRS, Unit: s */
	double resEquivalentOn;          /* resInSerialForSenseAmp in parallel with resMemCellOn, Unit: ohm */
	double resEquivalentOff;          /* resInSerialForSenseAmp in parallel with resMemCellOn, Unit: ohm */
	double resInSerialForSenseAmp; /* Serial resistance of voltage-in voltage sensing as a voltage divider, Unit: ohm */
	double bitlineDelay;	/* Bitline delay, Unit: s */

	// for accumulator
	bool withOutputAcc;

	// for priority encoder
	bool withPriorityEnc;
	BufferDesignTarget PriorityOptLevel;

	// for cell  ??
	double resMemCellOff;  /* HRS resistance, Unit: ohm */
	double resMemCellOn;   /* LRS resistance, Unit: ohm */
	double voltageMemCellOff; /* Voltage drop on HRS during read operation, Unit: V */
	double voltageMemCellOn;   /* Voltage drop on LRS druing read operation, Unit: V */
	double resCellAccess; /* Resistance of access device, Unit: ohm */
	double capCellAccess; /* Capacitance of access device, Unit: ohm */
	double resMatchTran;
	double capMatchTran;

	double decoderLatency;
	double bitlineRamp;
	double chargeLatency;
	double searchLatency;
	double searchDynamicEnergy;
	double energyDriveSearch0;
	double energyDriveSearch1;
	double rampOutput;
	double resetEnergyPerBit;
	double setEnergyPerBit;
	double senseAmpLatency;

	double searchAverage;

	//Array size vs. Accuracy tradeoff
	double SenseTime;
	double Basetau;
	double ArrayWidth;
	double MLWindow;
	double resTotalCell;
	double BaseResTotalCell;
	double BaseSenseTime;
	double tau;
	double MatchlineSenseMargin;
	double temptau;
	double resmiss;
	double totalcapcell;

	// useless items
	bool multipleRowPerSet;		/* For cache design, whether a set is partitioned into multiple wordlines */
	bool split;			/* NOT USED YET for NVSIM: Whether the row decoder is at the middle of subarrays */
	double lenWordline;	/* Length of wordlines, Unit: m */
	double lenBitline;	/* Length of bitlines, Unit: m */
	double capWordline;	/* Wordline capacitance, Unit: F */
	double capBitline;	/* Bitline capacitance, Unit: F */
	double resWordline;	/* Wordline resistance, Unit: ohm */
	double resBitline;	/* Bitline resistance, Unit: ohm */
	double maxCurrentBL;
	RowDecoder	rowDecoder;
	RowDecoder	bitlineMuxDecoder;
	Mux			bitlineMux;
	RowDecoder	senseAmpMuxLev1Decoder;
	RowDecoder	senseAmpMuxLev2Decoder;
};


#endif /* CAM_SUBARRAY_H_ */
