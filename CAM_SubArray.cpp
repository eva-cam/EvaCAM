/*
 * CAM_SubArray.cpp
 *
 */
#include "CAM_SubArray.h"
#include "formula.h"
#include "global.h"
#include "constant.h"
#include "CAM_Line.h"
#include "CAM_Cell.h"
#include <math.h>

CAM_SubArray::CAM_SubArray() {
	// TODO Auto-generated constructor stub
	initialized = false;
	invalid = false;
}

CAM_SubArray::~CAM_SubArray() {
	// TODO Auto-generated destructor stub
}

void CAM_SubArray::Initialize(long long _numRow, long long _numColumn, bool _multipleRowPerSet, bool _split,
		int _muxSenseAmp, bool _internalSenseAmp, int _muxOutputLev1, int _muxOutputLev2,
		BufferDesignTarget _DecMergeOptLevel, BufferDesignTarget _DriverOptLevel,
		bool _withInputEnc, TypeOfInputEncoder _typeInputEnc, bool _customInputEnc,
		TypeOfSenseAmp _typeSenseAmp, bool _customSenseAmp, bool _withWriteDriver,
		bool _withOutputAcc, bool _withPriorityEnc, BufferDesignTarget _PriorityOptLevel,
		bool _withInputBuf, bool _withOutputBuf, CAMType _camType, SearchFunction _searchFunction) {
	if (initialized)
		cout << "[CAM_SubArray] Warning: Already initialized!" << endl;
	numRow = _numRow;
	numColumn = _numColumn;
	multipleRowPerSet = _multipleRowPerSet;
	split = _split;
	muxSenseAmp = _muxSenseAmp;
	muxOutputLev1 = _muxOutputLev1;
	muxOutputLev2 = _muxOutputLev2;
	internalSenseAmp = _internalSenseAmp;
	DecMergeOptLevel = _DecMergeOptLevel;
	DriverOptLevel = _DriverOptLevel;
	withInputEnc = _withInputEnc;
	typeInputEnc = _typeInputEnc;
	customInputEnc = _customInputEnc;
	withWriteDriver = _withWriteDriver;
	typeSenseAmp = _typeSenseAmp;
	customSenseAmp = _customSenseAmp;
	withOutputAcc = _withOutputAcc;
	withPriorityEnc = _withPriorityEnc;
	PriorityOptLevel = _PriorityOptLevel;
	withInputBuf = _withInputBuf;
	withOutputBuf = _withOutputBuf;
	camType = _camType;
	searchFunction = _searchFunction;

	//////////////////////////////////////////////////////////////////////////////////
	//		   						 input check: 									//
	//////////////////////////////////////////////////////////////////////////////////

	if (inputParameter->realCapacity != inputParameter->capacity && inputParameter->realCapacity != 0) {
		// deal with the ASP-DAC12 fucking 72-bit word
		numRow = inputParameter->realCapacity / inputParameter->minNumRowSubarray / inputParameter->minNumColumnSubarray
				/  inputParameter->minNumActiveMatPerRow / inputParameter->minNumActiveMatPerColumn / numColumn;
	}
	if(CAM_cell->memCellType != FEFETRAM && inputParameter->searchFunction != EX){
		invalid = true;
		cout <<" [Search function Error]: Other search functions are under developmemt. " <<endl;
		return;
	}
	if(CAM_cell->memCellType != FEFETRAM && inputParameter->camType != TCAM){
		invalid = true;
		cout <<" [CAM type Error]: Other CAM types are under developmemt. " <<endl;
		return;
	}

	/*if (inputParameter->designTarget != CAM_chip || CAM_cell->memCellType > memristor ||  CAM_cell->memCellType == DRAM || CAM_cell->memCellType == eDRAM) {
		invalid = true;
		cout <<"[CAM_SubArray] Error: type input error" <<endl;
		return;
	}*/
	
	if (CAM_cell->memCellType == SRAM) {
		if (CAM_cell->camNumCol != 5 || CAM_cell->camNumRow != 3) {
			// TODO: other types of SRAM based TCAM are under development
			invalid = true;
			cout <<"[CAM_SubArray] Error: other types of SRAM based TCAM are under development" <<endl;
			return;
		}
	}
	if (CAM_cell->camNumRow < 1 || CAM_cell->camNumCol < 1) {
		invalid = true;
		cout <<"[CAM_SubArray] Error: cell configuration error" <<endl;
		return;
	}

	//////////////////////////////////////////////////////////////////////////////////
	//		   						 defination: 									//
	//////////////////////////////////////////////////////////////////////////////////


	/* Derived parameters */
	numSenseAmp = numColumn / muxSenseAmp;
	lenRow = (double)numColumn * CAM_cell->widthInFeatureSize * tech->featureSize;
	lenCol = (double)numRow * cell->heightInFeatureSize * tech->featureSize;
	/* Add stitching overhead if necessary */
	if (CAM_cell->stitching) {
		lenRow += ((numColumn - 1) / CAM_cell->stitching + 1) * STITCHING_OVERHEAD * tech->featureSize;
	}

	//////////////////////////////////////////////////////////////////////////////////
	//		   			   calculation for SA: 										//
	//////////////////////////////////////////////////////////////////////////////////

	// 1. setting SA sense mode
	// TODO: note that cmos-based/doide-based has to be current-in voltage sensing
	// 5. calc precharge voltage
	// 6. calc for sensing: resMemCellOff, voltageMemCellOff

	voltageSense = bool(CAM_cell->readMode > 0);
	senseVoltage = CAM_cell->minSenseVoltage;


	if (!internalSenseAmp && CAM_cell->memCellType != SRAM) {
		invalid = true;
		cout << "[CAM_SubArray] Error: nvTCAM does not support external sense amplifiers!" << endl;
		return;
	}

	// calc voltage precharger
	// TODO: for simple, all of the design are precharged to vdd
	/* In fact, diode just need to charge to Vmatch+vth
	 * Cmos could be half swing
	 * none could be n*volatgeMemOff
	 */
	voltagePrecharge = tech->vdd;
	resCellAccess = CalculateOnResistance(CAM_cell->widthAccessCMOS*tech->featureSize, NMOS, inputParameter->temperature, *tech);
	capCellAccess = CalculateDrainCap(CAM_cell->widthAccessCMOS*tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech);
	resMatchTran = CalculateOnResistance(CAM_cell->camWidthMatchTran * tech->featureSize, NMOS, inputParameter->temperature, *tech);
	capMatchTran = CalculateDrainCap(CAM_cell->camWidthMatchTran * tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * tech->featureSize, *tech);
	
	if (CAM_cell->accessType == CMOS_access) {
		resMemCellOff = tech->vdd / tech->currentOffNmos[inputParameter->temperature - 300]
		                / tech->featureSize / CAM_cell->camWidthMatchTran;
		resMemCellOn = resMatchTran;
		if (CAM_cell->memCellType == SRAM)
			resMemCellOn *=2;
	} else if (CAM_cell->accessType == diode_access) {
		resMemCellOff = tech->vdd / tech->currentOffNmos[inputParameter->temperature - 300]
		                        / tech->featureSize / CAM_cell->camWidthMatchTran;
		resMemCellOn = resMatchTran + resCellAccess + CAM_cell->resistanceOn;
	} else if (CAM_cell->accessType == none_access) {
		resMemCellOff = resMatchTran + CAM_cell->resistanceOff;
		resMemCellOn = resMatchTran+ CAM_cell->resistanceOn;
		if (CAM_cell-> memCellType == FEFETRAM){ //2FEFET TCAM has no access transistor, and the Ron/off should be measured by HSPICE model
		// resCellAccess = 0;
		// capCellAccess = 0;
		// resMatchTran = 0;
		// capMatchTran = 0;
		resCellAccess = CalculateOnResistance(CAM_cell->widthAccessCMOS * FEFET_tech->featureSize, NMOS, inputParameter->temperature, *FEFET_tech);
		capCellAccess = CalculateDrainCap(CAM_cell->widthAccessCMOS * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech);
		resMatchTran = CalculateOnResistance(CAM_cell->camWidthMatchTran * FEFET_tech->featureSize, NMOS, inputParameter->temperature, *tech);
		capMatchTran = CalculateDrainCap(CAM_cell->camWidthMatchTran * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech);
				
		resMemCellOn = CAM_cell->resistanceOn;
		resMemCellOff = CAM_cell->resistanceOff;
		}
	} else {
		// this will not happen
		invalid = true;
		cout << "[CAM_SubArray] Error: access type error!" << endl;
		return;
	}



	// TODO: go back when design latency
	if (CAM_cell->memCellType != SRAM) {
		/* MRAM, PCRAM, and memristor have three types of access devices: CMOS, BJT, and diode */
		if (voltageSense) {						/* voltage-sensing */
			if (CAM_cell->readVoltage == 0) {  /* Current-in voltage sensing */
				voltageMemCellOff = CAM_cell->readCurrent * resMemCellOff;
				voltageMemCellOn = CAM_cell->readCurrent * resMemCellOn;
				//voltagePrecharge = (voltageMemCellOff + voltageMemCellOn) / 2;
				//voltagePrecharge = MIN(tech->vdd, voltagePrecharge);  /* TO-DO: we can have charge bump to increase SA working point */
				if ((voltagePrecharge - voltageMemCellOn) <= senseVoltage) {
					cout <<"[CAM_SubArray] Error: Read current too large or too small that no reasonable precharge voltage existing" <<endl;
					invalid = true;
					return;
				}
			} else {   /*Voltage-in voltage sensing */
				resInSerialForSenseAmp = sqrt(resMemCellOn * resMemCellOff);
				resEquivalentOn = resMemCellOn * resInSerialForSenseAmp / (resMemCellOn + resInSerialForSenseAmp);
				resEquivalentOff = resMemCellOff * resInSerialForSenseAmp / (resMemCellOff + resInSerialForSenseAmp);
				voltageMemCellOff = CAM_cell->readVoltage * resMemCellOff / (resMemCellOff + resInSerialForSenseAmp);
				voltageMemCellOn = CAM_cell->readVoltage * resMemCellOn / (resMemCellOn + resInSerialForSenseAmp);
				//voltagePrecharge = (voltageMemCellOff + voltageMemCellOn) / 2;
				//voltagePrecharge = MIN(tech->vdd, voltagePrecharge);  /* TO-DO: we can have charge bump to increase SA working point */
				if ((voltagePrecharge - voltageMemCellOn) <= senseVoltage) {
					// cout <<"[CAM_SubArray] Error: Read Voltage too large or too small that no reasonable precharge voltage existing" <<endl;
					// invalid = true;
					// return;
				}
			}
		}

		if(CAM_cell->accessType == CMOS_access) {
			voltageMemCellOn = 0;
			voltageMemCellOff = voltagePrecharge;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////
	//		   			   calculation for driver: 									//
	//////////////////////////////////////////////////////////////////////////////////

	// 2. line resistance caclualtion
	// 3. Mux load calculation: extended from signel bl to cols
	// 4. transistor's impact on res and cap of the lines


	for(int i=0;i<CAM_cell->camNumRow;i++){
		Row[i].Initialize(true, i, lenRow, numColumn);
	}
	for(int i=0;i<CAM_cell->camNumCol;i++){
		Col[i].Initialize(false, i, lenCol, numRow);
		if (Col[i].minMuxWidth > inputParameter->maxNmosSize * tech->featureSize) {
			invalid = true;
			cout <<"[CAM_SubArray] Error: Mux width too large" <<endl;
			return;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////
	//		   					 validation check: 									//
	//////////////////////////////////////////////////////////////////////////////////

	// 2. ML length: match (all-Ih) v.s. 1-miss (Il+all-Ih) for search

	if (CAM_cell->accessType == CMOS_access || CAM_cell->accessType == diode_access){
		// ML is connected with the drain of NMOS, like ISSCC15-3t1r
		// ML is connected with the diode (both gate and drain of nmos), like VSLIT12-4t2r
		// the leakage cannot be too large
		if(numRow * tech->currentOffNmos[inputParameter->temperature - 300] / BITLINE_LEAKAGE_TOLERANCE >
			tech->currentOnNmos[inputParameter->temperature - 300]) {
			invalid = true;
			cout <<"[CAM_SubArray] Error: bitline too long" <<endl;
			return;
		}
	} else if (CAM_cell->accessType == none_access) {
		// ML is connected with cell, like JSSC11-2t2r and also for 2FEFET TCAM design
		// TODO sth for 2FEFET TCAM
		cout << "numRow: " << numRow;
		if( inputParameter->withOutputAcc == false ) {
			double Rh = numRow * resMemCellOff;
			double Rl = resMemCellOn;
			senseMargin = 2 * senseVoltage * (Rh - Rl) / (Rh + Rl);
			if ( senseMargin < MIN_SENSE_MARGIN ) {
				// TODO: the minimal sense margin is defined in the constant
				invalid = true;
				cout <<"[CAM_SubArray] Error: too many rows for sense" <<endl;
				return;
			}
		}
	} else {
		invalid = true;
		cout <<"[CAM_SubArray] Error: access type input error" <<endl;
		return;
	}

	//////////////////////////////////////////////////////////////////////////////////
	//		   						 Initialize										//
	//////////////////////////////////////////////////////////////////////////////////

	double capNandInput, tmp;

	if(withInputBuf) {
		CalculateGateCapacitance(NAND, 2, 2 * MIN_NMOS_SIZE * tech->featureSize, tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize,
					tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandInput, &tmp);
		inputBuf.Initialize(true /*TODO*/, capNandInput, 0);
	}

	if(withInputEnc) {
		CalculateGateCapacitance(NAND, 2, 2 * MIN_NMOS_SIZE * tech->featureSize, tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize,
					tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandInput, &tmp);
		inputEnc.Initialize(encoding_two_bit, false, capNandInput, 0 /* TODO*/);
		inputEnc.CalculateRC();
	}


	// this NAND merges pre-decoder's result, output the WL activation signal
	CalculateGateCapacitance(NAND, 2, 2 * MIN_NMOS_SIZE * tech->featureSize, tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize,
				tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandInput, &tmp);
	if(!CAM_cell->camMLC && CAM_cell->memCellType != SRAM) {
		RowDecMergeNand.Initialize(numRow * 2, capNandInput, 0, false/*TODO*/, true, DecMergeOptLevel, 0 /*TODO*/);
	} else {
		RowDecMergeNand.Initialize(numRow, capNandInput, 0, false/*TODO*/, true, DecMergeOptLevel, 0 /*TODO*/);
	}
	RowDecMergeNand.CalculateRC();
	// those NAND merges the WL and SL signal
	//RowDriver = new CAM_RowNand [CAM_cell->camNumRow];
	for(int i=0;i<CAM_cell->camNumRow;i++){
		RowDriver[i].Initialize(numRow, Row[i].cap*1.6, Row[i].res, false/*TODO*/, false, DriverOptLevel, Row[i].maxCurrent);
		RowDriver[i].CalculateRC();
	}
	// if(CAM_cell->memCellType == FEFETRAM){
	// 	for(int i=0;i<CAM_cell->camNumRow;i++){
	// 	RowDriver[i].Initialize(numRow, Row[i].cap /* * 10 */, Row[i].res, false/*TODO*/, false, DriverOptLevel, Row[i].maxCurrent);
	// 	RowDriver[i].CalculateRC();
	// 	}
	// }

	// precharge
	indexMatchline = -1;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		if (Col[i].CellPort.Type == Matchline || Col[i].CellPort.Type == Matchline_Bitline) {
			indexMatchline = i;
			break;
		}
	}
	if(indexMatchline == -1) {
		invalid = true;
		cout <<"[CAM_SubArray] Error: no matchline found" <<endl;
		return;
	}
	precharger.Initialize(voltagePrecharge, numColumn, Col[indexMatchline].cap, Col[indexMatchline].res);
	precharger.CalculateRC();


	// colmn decoder signal merge
	// TODO: I am too lazy to calculate very one, just use the index zero
	Row[CAM_cell->camNumRow].Initialize(lenRow, numColumn, Col[0].minMuxWidth);
	ColDecMergeNand.Initialize(CAM_cell->camNumCol * muxSenseAmp, Row[CAM_cell->camNumRow].cap, Row[CAM_cell->camNumRow].res, false, DecMergeOptLevel, 0);
	ColDecMergeNand.CalculateRC();

	senseAmpMuxLev1Nand.Initialize(muxOutputLev1, Row[CAM_cell->camNumRow].cap, Row[CAM_cell->camNumRow].res, false, DecMergeOptLevel, 0);
	senseAmpMuxLev1Nand.CalculateRC();
	senseAmpMuxLev2Nand.Initialize(muxOutputLev2, Row[CAM_cell->camNumRow].cap, Row[CAM_cell->camNumRow].res, false, DecMergeOptLevel, 0);
	senseAmpMuxLev2Nand.CalculateRC();

	// MUX
	senseAmpMuxLev2.Initialize(muxOutputLev2, numColumn / muxSenseAmp / muxOutputLev1 / muxOutputLev2,
			0, 0, Col[indexMatchline].maxCurrent);
	senseAmpMuxLev2.CalculateRC();
	senseAmpMuxLev1.Initialize(muxOutputLev1, numColumn / muxSenseAmp / muxOutputLev1,
			senseAmpMuxLev2.capForPreviousDelayCalculation, senseAmpMuxLev2.capForPreviousPowerCalculation, Col[indexMatchline].maxCurrent);
	senseAmpMuxLev1.CalculateRC();

	if (internalSenseAmp) {
		senseAmp.Initialize(numColumn / muxSenseAmp, typeSenseAmp, customSenseAmp, senseVoltage, lenRow / numColumn * muxSenseAmp);
		senseAmp.CalculateRC();
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMux[i].Initialize(muxSenseAmp, numColumn / muxSenseAmp, senseAmp.capLoad, senseAmp.capLoad, Col[i].maxCurrent);
			ColMux[i].CalculateRC();
		}
	} else {
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMux[i].Initialize(muxSenseAmp, numColumn / muxSenseAmp, senseAmpMuxLev1.capForPreviousDelayCalculation,
					senseAmpMuxLev1.capForPreviousPowerCalculation, Col[i].maxCurrent);
			ColMux[i].CalculateRC();
		}
	}

	if (withWriteDriver) {
		for(int i=0;i<CAM_cell->camNumCol;i++) {
			if(CAM_cell->camPort[1][i].Type == Matchline) {
				WriteDriver[i].initialized = false;
			} else {
				WriteDriver[i].Initialize(numColumn / muxSenseAmp, Col[i].cap, Col[i].res, false, DriverOptLevel, Col[i].maxCurrent);
				WriteDriver[i].CalculateRC();
			}
		}
	}

	if (withPriorityEnc) {
		priorityEnc.Initialize(numColumn, PriorityOptLevel, 0, 0 /*TODO: no output driver*/);
	}

	if (withOutputAcc) {
		if (withPriorityEnc)
			outputAcc.Initialize(priorityEnc.MMR.BasicMMR.capIn, 0);
		else
			outputAcc.Initialize(0, 0 /*TODO: no output driver*/);
		outputAcc.CalculateRC();
	}

	if(withOutputBuf) {
		CalculateGateCapacitance(NAND, 2, 2 * MIN_NMOS_SIZE * tech->featureSize, tech->pnSizeRatio * MIN_NMOS_SIZE * tech->featureSize,
					tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capNandInput, &tmp);
		outputBuf.Initialize(true /*TODO*/, capNandInput, 0);
	}

	initialized = true;
}

void CAM_SubArray::CalculateArea() {
	if (!initialized) {
		cout << "[CAM_SubArray] Error: Require initialization first!" << endl;
	} else if (invalid) {
		cout << "[CAM_SubArray] Error: invalid!" << endl;
		height = width = area = 1e41;
	} else {
		double addWidthArea = 0, addHeightArea = 0;


		width = lenRow;
		height = lenCol;
	//	cout << "width: height: " << width * 1e6 << ": " << height*1e6 << endl;
		area = height * width;


		if(withInputBuf) {
			inputBuf.CalculateArea();
			area += (inputBuf.area * numRow);
			addWidthArea += (inputBuf.area * numRow);
		}

		if(withInputEnc) {
			inputEnc.CalculateArea();
			area += (inputEnc.area * numRow);
			addWidthArea += (inputEnc.area * numRow);
		}

		RowDecMergeNand.CalculateArea();
		area += (RowDecMergeNand.area);
		addWidthArea += (RowDecMergeNand.area);


		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowDriver[i].CalculateArea();
			area += (RowDriver[i].area);
			addWidthArea += (RowDriver[i].area);
		}

		// colmn decoder signal merge
		ColDecMergeNand.CalculateArea();
		area += (ColDecMergeNand.area);
		addWidthArea += (ColDecMergeNand.area);

		senseAmpMuxLev1Nand.CalculateArea();
		area += (senseAmpMuxLev1Nand.area);
		addWidthArea += (senseAmpMuxLev1Nand.area);
		senseAmpMuxLev2Nand.CalculateArea();
		area += (senseAmpMuxLev2Nand.area);
		addWidthArea += (senseAmpMuxLev2Nand.area);

		precharger.CalculateArea();
		area += (precharger.area);
		addHeightArea += (precharger.area);

		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMux[i].CalculateArea();
			area += (ColMux[i].area);
			addHeightArea += (ColMux[i].area);
		}

		// MUX
		senseAmpMuxLev2.CalculateArea();
		area += (senseAmpMuxLev2.area);
		addHeightArea += (senseAmpMuxLev2.area);
		senseAmpMuxLev1.CalculateArea();
		area += (senseAmpMuxLev1.area);
		addHeightArea += (senseAmpMuxLev1.area);

		if (internalSenseAmp) {
			senseAmp.CalculateArea();
			area += (senseAmp.area);
			addHeightArea += (senseAmp.area);
		}

		WriteDriverArea = 0;
		if (withWriteDriver) {
			for(int i=0;i<CAM_cell->camNumCol;i++){
				if (WriteDriver[i].initialized) {
					WriteDriver[i].CalculateArea();
					if (i > 0 && Col[i].CellPort.Type == Bitline && Col[i-1].CellPort.Type == Bitline) {
						//WriteDriverArea += (WriteDriver[i].outputDriver.area);
						WriteDriverArea += (WriteDriver[i].area);
					} else {
						WriteDriverArea += (WriteDriver[i].area);
					}
				}
			}
		}
		area += WriteDriverArea;
		addHeightArea += WriteDriverArea;

		if (withOutputAcc) {
			outputAcc.CalculateArea();
			area += (outputAcc.area * numColumn / muxSenseAmp);
			addHeightArea += (outputAcc.area * numColumn / muxSenseAmp);
		}

		if (withPriorityEnc) {
			priorityEnc.CalculateArea();
			area += (priorityEnc.area);
			addHeightArea += (priorityEnc.area);
		}

		if(withOutputBuf) {
			outputBuf.CalculateArea();
			area += (outputBuf.area * numColumn / muxSenseAmp);
			addWidthArea += (outputBuf.area * numColumn / muxSenseAmp);
		}

		// TODO: a prefect layout
		width = addWidthArea / lenCol + lenCol;
		height = area / width;
//		cout << "width: height: 22 " << width * 1e6 << ": " << height*1e6 << endl;
	}
}

void CAM_SubArray::CalculateLatency(double _rampInput) {
	if (!initialized) {
		cout << "[CAM_SubArray] Error: Require initialization first!" << endl;
	} else if (invalid) {
		cout << "[CAM_SubArray] Error: invalid!" << endl;
		searchLatency = readLatency = writeLatency = 1e41;
	} else {

		if(withInputBuf) {
			inputBuf.CalculateLatency(_rampInput);
		} else {
			inputBuf.readLatency = 0;
			inputBuf.rampOutput = _rampInput;
		}

		if(withInputEnc) {
			inputEnc.CalculateLatency(_rampInput);
		} else {
			inputEnc.readLatency = 0;
			inputEnc.rampOutput = _rampInput;
		}

		// this NAND merges pre-decoder's result, output the WL activation signal
		RowDecMergeNand.CalculateLatency(_rampInput);
		// those NAND merges the WL and SL signal
		double maxRowDriver = 0;
		int indexMaxRowDriver = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowDriver[i].CalculateLatency(MAX(inputEnc.rampOutput, RowDecMergeNand.rampOutput));
			if (RowDriver[i].readLatency > maxRowDriver) {
				maxRowDriver = RowDriver[i].readLatency;
				indexMaxRowDriver = i;
			}
		}

		// precharge
		precharger.CalculateLatency(_rampInput);

		// colmn decoder signal merge
		ColDecMergeNand.CalculateLatency(_rampInput);

		senseAmpMuxLev1Nand.CalculateLatency(_rampInput);
		senseAmpMuxLev2Nand.CalculateLatency(_rampInput);

		columnDecoderLatency = MAX(MAX(ColDecMergeNand.readLatency, senseAmpMuxLev1Nand.readLatency), senseAmpMuxLev2Nand.readLatency);
		decoderLatency = MAX(RowDecMergeNand.readLatency + maxRowDriver, columnDecoderLatency);

		//////////////////////////////////////////////////////////////////////////////////
		//		   					calc matchline & bitline							//
		//////////////////////////////////////////////////////////////////////////////////
		double tau, gm, beta;

		if (typeSenseAmp == discharge) {
			if (CAM_cell->memCellType == SRAM || CAM_cell->accessType == CMOS_access) {
				double resTotalCell = 0;
				// 1-miss situation
				resTotalCell = resMemCellOn; // + resMemCellOff / (numRow-1);
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				referDelay = tau * log((voltagePrecharge) / (CAM_cell->readVoltage));

				// all-match situation
				resTotalCell = resMemCellOff / CAM_opt.BitSerialWidth;//  /numRow;
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				volMatchDrop = voltagePrecharge - voltagePrecharge * exp(-referDelay * tau);

				// all-miss situation
				resTotalCell = resMemCellOn  / CAM_opt.BitSerialWidth; // / numRow;
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				volAllMissDrop = voltagePrecharge - voltagePrecharge * exp(-referDelay * tau);

				senseMargin = voltagePrecharge - volMatchDrop - CAM_cell->readVoltage;
				if (senseMargin < senseVoltage) {
					cout << "[CAM_Subarray] Error: bitline too long to be sensed!" << endl;
					invalid = true;
					searchLatency = readLatency = writeLatency = 1e41;
					return;
				}

				bitlineDelayOn = bitlineDelayOff = bitlineDelay = referDelay;
			} else if (CAM_cell->accessType == diode_access) {
				double resTotalCell = 0;

				// 1-miss situation
				resTotalCell = resMemCellOn; // + resMemCellOff / (numRow-1);
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				referDelay = tau * log((voltagePrecharge - tech->vth) / (CAM_cell->readVoltage));
				// all-match situation
				resTotalCell = resMemCellOff  / CAM_opt.BitSerialWidth; // / numRow;
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				volMatchDrop = (voltagePrecharge - tech->vth) - (voltagePrecharge - tech->vth) * exp(-referDelay / tau);

				// all-miss situation
				resTotalCell = resMemCellOn  / CAM_opt.BitSerialWidth; // / numRow;
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				volAllMissDrop = (voltagePrecharge - tech->vth) - (voltagePrecharge - tech->vth) * exp(-referDelay / tau);

				senseMargin = voltagePrecharge - volMatchDrop - CAM_cell->readVoltage;
				if (senseMargin < senseVoltage) {
					cout << "[CAM_Subarray] Error: bitline too long to be sensed!" << endl;
					invalid = true;
					searchLatency = readLatency = writeLatency = 1e41;
					return;
				}
				// pre-threshold drop
				resTotalCell = resMemCellOn  / CAM_opt.BitSerialWidth; // / numRow;
				tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				referDelay += tau * log((voltagePrecharge) / (voltagePrecharge - tech->vth));
				bitlineDelayOn = bitlineDelayOff = bitlineDelay = referDelay;
			} else if (CAM_cell->accessType == none_access) {
				// double resTotalCell = 0;
				// resTotalCell = resMemCellOn + resMemCellOff/(CAM_opt.BitSerialWidth-1);
				// 1-miss situation may exist problems for PCM 2T2R design
				// if (CAM_opt.BitSerialWidth == 1) {
				// 	resTotalCell = resMemCellOn;
				// } else {
				// 	resTotalCell = resMemCellOn * resMemCellOff  / (CAM_opt.BitSerialWidth - 1)
				// 			/ (resMemCellOn + resMemCellOff  / (CAM_opt.BitSerialWidth - 1)); // / (numRow-1);
				// }
				// tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
				// 		+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				// referDelay = tau * log((voltagePrecharge) / (CAM_cell->readVoltage));

				// // all-match situation
				// resTotalCell = resMemCellOff  / CAM_opt.BitSerialWidth; // / numRow;
				// tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
				// 		+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				// volMatchDrop = (voltagePrecharge) - (voltagePrecharge) * exp(-referDelay / tau);

				// // all-miss situation
				// resTotalCell = resMemCellOn  / CAM_opt.BitSerialWidth; // / numRow;
				// tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
				// 		+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				// volAllMissDrop = voltagePrecharge - voltagePrecharge * exp(-referDelay * tau);

				// senseMargin = voltagePrecharge - volMatchDrop - CAM_cell->readVoltage;
				// if (senseMargin < senseVoltage) {
				// 	cout << "[CAM_Subarray] Error: bitline too long to be sensed!" << endl;
				// 	invalid = true;
				// 	searchLatency = readLatency = writeLatency = 1e41;
				// 	return;
				// }
				// bitlineDelayOn = bitlineDelayOff = bitlineDelay = referDelay;
				if (CAM_cell -> memCellType == FEFETRAM){

					
						double resTotalCell = 0;
						double ArrayWidth = 0;
						double BaseResTotalCell = 0;


						resTotalCell = resMemCellOn + resMemCellOff/(CAM_opt.BitSerialWidth-1);
						BaseResTotalCell = resMemCellOff/CAM_opt.BitSerialWidth;
						// }
						tau = resTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
						Basetau = BaseResTotalCell * (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation) 
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
						SenseTime = log(2)*(tau-Basetau);
						//BaseSenseTime = log(2)*Basetau;
				
					}
			} else {
				// should not happen
				cout << "[CAM_subarray]: Error input access tpye!" << endl;
				exit(-1);
			}
		} else {
			if (CAM_cell->memCellType == SRAM) {
				/* Codes below calculate the bitline latency */
				double resPullDown = CalculateOnResistance(CAM_cell->widthSRAMCellNMOS * tech->featureSize, NMOS,
						inputParameter->temperature, *tech);
				tau = (resMatchTran + resPullDown) * (capMatchTran + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
				tau *= log(voltagePrecharge / (voltagePrecharge - senseVoltage));
				gm = CalculateTransconductance(CAM_cell->camWidthMatchTran * tech->featureSize, NMOS, *tech);
				beta = 1 / (resPullDown * gm);
				bitlineDelay = horowitz(tau, beta, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
			} else if (CAM_cell->memCellType == FEFETRAM) {
				if (CAM_cell->accessType == none_access){
					// 1-miss situation
					resTotalCell = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-1)+resMemCellOff*1);



					tau = resTotalCell*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation +  inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Basetau = BaseResTotalCell * (capCellAccess*CAM_opt.BitSerialWidth  + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (capCellAccess*CAM_opt.BitSerialWidth  + ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
					totalcapcell = Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation*1e9;


					bitlineDelay = horowitz(tau, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
				
					// //////////////////////MCAM//////////////////////////////////////////
					double S0, S1, S2, S3, S4, S5, S6, S7;
					S0 = 1.616E+06;
					S1 = 8.547E+05;
					S2 = 2.833E+05;
					S3 = 1.083E+05;
					S4 = 5.291E+04;
					S5 = 3.300E+04;
					S6 = 2.439E+04;
					S7 = 2.004E+04;

					//CAM_opt.BitSerialWidth = 32;

					double resM0, resM1, resM2, resM3, resM4, resM5, resM6, resM7;
					double Stau0, Stau1, Stau2, Stau3, Stau4, Stau5, Stau6, Stau7;
					resM0 = S0/CAM_opt.BitSerialWidth;
					resM1 = S1*S0/(S1*(CAM_opt.BitSerialWidth-1)+S0);
					resM2 = S2*S0/(S2*(CAM_opt.BitSerialWidth-1)+S0);
					resM3 = S3*S0/(S3*(CAM_opt.BitSerialWidth-1)+S0);
					resM4 = S4*S0/(S4*(CAM_opt.BitSerialWidth-1)+S0);
					resM5 = S5*S0/(S5*(CAM_opt.BitSerialWidth-1)+S0);
					resM6 = S6*S0/(S6*(CAM_opt.BitSerialWidth-1)+S0);
					resM7 = S7*S0/(S7*(CAM_opt.BitSerialWidth-1)+S0);
					

					Stau0 = resM0*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau1 = resM1*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau2 = resM2*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau3 = resM3*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau4 = resM4*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau5 = resM5*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau6 = resM6*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);
					Stau7 = resM7*(capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad + inputParameter->AddCapOnML) 
							+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + capCellAccess*CAM_opt.BitSerialWidth + Col[indexMatchline].cap / 2);


					
					double MbitlineDelay0 = horowitz(Stau0, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay1 = horowitz(Stau1, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay2 = horowitz(Stau2, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay3 = horowitz(Stau3, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay4 = horowitz(Stau4, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay5 = horowitz(Stau5, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay6 = horowitz(Stau6, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double MbitlineDelay7 = horowitz(Stau7, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					////////////////////////////////////////////////////////////////////
					// cout << "-1/tau0, -1/tau1, -1/tau2, -1/tau3, -1/tau4, -1/tau5 = " << -1/tau0 <<": " << -1/tau1 << ": " << -1/tau2  << ": " << -1/tau3 << ": " << -1/tau4 << ": " << -1/tau5 << endl;
				} else if(CAM_cell->accessType == CMOS_access){
					double resOn = CalculateOnResistance( tech->featureSize, NMOS,inputParameter->temperature, *tech);
					double tauM1 = (resOn) * (capMatchTran + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
						+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
					double TotalCap = capMatchTran;
					//cout << "total cap" << TotalCap *1e15<< endl;
				
				gm = CalculateTransconductance(	tech->featureSize, NMOS, *tech);
				beta = 1 / (resOn * gm);
				bitlineDelay = horowitz(tauM1, beta, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);			
				}
			} else if (CAM_cell->memCellType == MRAM || CAM_cell->memCellType == PCRAM || CAM_cell->memCellType == memristor || CAM_cell->memCellType == FBRAM /*|| CAM_cell->memCellType == FEFETRAM*/) {
				if (CAM_cell->readMode == false) {	/* current-sensing */
					/* Use ICCAD 2009 model */
					////////////////////////Approximate Match ////////////////////////
					double res1 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-1)+resMemCellOff*1);
					double res2 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-2)+resMemCellOff*2);
					double res3 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-3)+resMemCellOff*3);
					double res4 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-4)+resMemCellOff*4);
					double res5 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-5)+resMemCellOff*5);

					double tau1 = Col[indexMatchline].res * (Col[indexMatchline].cap+ 0.6e-13* CAM_opt.BitSerialWidth)/
							2 * (res1 + Col[indexMatchline].res / 3) / (res1 + Col[indexMatchline].res);
					double tau2 = Col[indexMatchline].res * (Col[indexMatchline].cap+ 0.6e-13* CAM_opt.BitSerialWidth)/
							2 * (res2 + Col[indexMatchline].res / 3) / (res1 + Col[indexMatchline].res);
					double tau3 = Col[indexMatchline].res * (Col[indexMatchline].cap+ 0.6e-13* CAM_opt.BitSerialWidth)/
							2 * (res3 + Col[indexMatchline].res / 3) / (res1 + Col[indexMatchline].res);
					double tau4 = Col[indexMatchline].res * (Col[indexMatchline].cap+ 0.6e-13* CAM_opt.BitSerialWidth)/
							2 * (res4 + Col[indexMatchline].res / 3) / (res1 + Col[indexMatchline].res);
					double tau5 = Col[indexMatchline].res * (Col[indexMatchline].cap+ 0.6e-13* CAM_opt.BitSerialWidth)/
							2 * (res5 + Col[indexMatchline].res / 3) / (res1 + Col[indexMatchline].res);					

					double BL1 = horowitz(tau1, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);	
					double BL2 = horowitz(tau2, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double BL3 = horowitz(tau3, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double BL4 = horowitz(tau4, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
					double BL5 = horowitz(tau5, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);

					// cout << "Bitline (current): " << BL1 *1e12 << ": " << BL2 *1e12 << ": " << BL3 *1e12 << ": " << BL4 *1e12 << ": " << BL5 *1e12 << endl;
					/////////////////////////////////////////////////////////////////
					tau = Col[indexMatchline].res * (Col[indexMatchline].cap+ CAM_opt.BitSerialWidth)/
							2 * (resMemCellOff + Col[indexMatchline].res / 3) / (resMemCellOff + Col[indexMatchline].res);
					bitlineDelay = horowitz(tau, 0, RowDriver[indexMaxRowDriver].rampOutput, &bitlineRamp);
				} else {						/* voltage-sensing */
					if (CAM_cell->readVoltage == 0) {  /* Current-in voltage sensing */
						tau = resMemCellOn * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						/////// Approximate Match /////////////////////
					double res1 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-1)+resMemCellOff*1);
					double res2 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-2)+resMemCellOff*2);
					double res3 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-3)+resMemCellOff*3);
					double res4 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-4)+resMemCellOff*4);
					double res5 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-5)+resMemCellOff*5);

						double tau1 = res1 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau2 = res2 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau3 = res3 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau4 = res4 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);								
						double tau5 = res5 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double bitlineOn1 = tau1 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn2 = tau2 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn3 = tau3 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn4 = tau4 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn5 = tau5 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));

						double bitlineOff1 = tau1 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff2 = tau2 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff3 = tau3 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff4 = tau4 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff5 = tau5 * log((voltagePrecharge) / (voltageMemCellOff));

						///////////////////////////////////////////////
						bitlineDelayOn = tau * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						// TODO
						//bitlineDelayOn = tau * log((voltagePrecharge) / (voltageMemCellOn));
						tau = resMemCellOff * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
						bitlineDelayOff = tau * log((voltagePrecharge) / (voltageMemCellOff));
						bitlineDelay = MAX(bitlineDelayOn, bitlineDelayOff);
						//////////////Approximate match////////////////////////////
						
						// Used for test ////////////////////////////
					} else {   /*Voltage-in voltage sensing */
						/////////////////////////////////test//////////////////////////
						resTotalCell = resMemCellOn*resMemCellOff/(resMemCellOn+resMemCellOff/(CAM_opt.BitSerialWidth-1));
						tau = resEquivalentOn * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
						bitlineDelayOn = tau * log((voltagePrecharge - voltageMemCellOn)/(voltagePrecharge - voltageMemCellOn - senseVoltage));
						tau = resEquivalentOff * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation + precharger.capOutputBitlinePrecharger + senseAmp.capLoad)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation + Col[indexMatchline].cap / 2);
						bitlineDelayOff = tau * log((voltageMemCellOff - voltagePrecharge)/(voltageMemCellOff - voltagePrecharge - senseVoltage));
						bitlineDelay = MAX(bitlineDelayOn, bitlineDelayOff);
	
							/////// Approximate Match /////////////////////
					double res1 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-1)+resMemCellOff*1);
					double res2 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-2)+resMemCellOff*2);
					double res3 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-3)+resMemCellOff*3);
					double res4 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-4)+resMemCellOff*4);
					double res5 = resMemCellOn*resMemCellOff/(resMemCellOn*(CAM_opt.BitSerialWidth-5)+resMemCellOff*5);

						double tau1 = res1 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau2 = res2 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau3 = res3 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double tau4 = res4 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);								
						double tau5 = res5 * (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousDelayCalculation)
								+ Col[indexMatchline].res * (ColMux[indexMatchline].capForPreviousDelayCalculation
								+ Col[indexMatchline].cap / 2);
						double bitlineOn1 = tau1 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn2 = tau2 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn3 = tau3 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn4 = tau4 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));
						double bitlineOn5 = tau5 * log((voltagePrecharge - voltageMemCellOn) / (voltagePrecharge - voltageMemCellOn - senseVoltage));

						double bitlineOff1 = tau1 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff2 = tau2 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff3 = tau3 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff4 = tau4 * log((voltagePrecharge) / (voltageMemCellOff));
						double bitlineOff5 = tau5 * log((voltagePrecharge) / (voltageMemCellOff));
					}
				}
			}
		}

		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMux[i].CalculateLatency(bitlineRamp);
		}
		if (internalSenseAmp) {
			senseAmp.CalculateLatency(ColMux[indexMatchline].rampOutput);
			senseAmpMuxLev1.CalculateLatency(1e20);
			senseAmpMuxLev2.CalculateLatency(senseAmpMuxLev1.rampOutput);
		} else {
			senseAmpMuxLev1.CalculateLatency(ColMux[indexMatchline].rampOutput);
			senseAmpMuxLev2.CalculateLatency(senseAmpMuxLev1.rampOutput);
		}

		if (withOutputAcc) {
			outputAcc.CalculateLatency(1e20);
		} else {
			outputAcc.readLatency = 0;
			outputAcc.rampOutput = 1e20;
		}

		if (withPriorityEnc) {
			priorityEnc.CalculateLatency(outputAcc.rampOutput);
			rampOutput = priorityEnc.rampOutput;
		} else {
			priorityEnc.readLatency = 0;
			priorityEnc.rampOutput = outputAcc.rampOutput;
			rampOutput = priorityEnc.rampOutput;
		}

		if(withOutputBuf) {
			outputBuf.CalculateLatency(outputAcc.rampOutput);
		} else {
			outputBuf.readLatency = 0;
		}

		searchLatency = inputBuf.readLatency + precharger.readLatency + maxRowDriver + inputEnc.readLatency + bitlineDelay
				+ ColMux[indexMatchline].readLatency + senseAmp.readLatency + outputAcc.readLatency + priorityEnc.readLatency
				+ outputBuf.readLatency;
		senseAmpLatency = senseAmp.readLatency;
		readLatency = inputBuf.readLatency + MAX(precharger.readLatency, decoderLatency + inputEnc.readLatency) + bitlineDelay
				+ ColMux[indexMatchline].readLatency + senseAmp.readLatency + senseAmpMuxLev1.readLatency + senseAmpMuxLev2.readLatency
				+ outputAcc.readLatency + priorityEnc.readLatency + outputBuf.readLatency;

		// for write
		double capPassTransistor = ColMux[indexMatchline].capNMOSPassTransistor +
				senseAmpMuxLev1.capNMOSPassTransistor + senseAmpMuxLev2.capNMOSPassTransistor;
		double resPassTransistor = ColMux[indexMatchline].resNMOSPassTransistor +
				senseAmpMuxLev1.resNMOSPassTransistor + senseAmpMuxLev2.resNMOSPassTransistor;
		double tauChargeLatency = resPassTransistor * (capPassTransistor + Col[indexMatchline].cap) +
				Col[indexMatchline].res * Col[indexMatchline].cap / 2;
		chargeLatency = horowitz(tauChargeLatency, 0, 1e20, NULL);
		WriteDriverLatency = 0;
		if (inputParameter->writeScheme == write_and_verify) {
			/*TODO: write and verify programming */
		} else {
			if (withWriteDriver) {
				for(int i=0;i<CAM_cell->camNumCol;i++){
					if (WriteDriver[i].initialized) {
						WriteDriver[i].CalculateLatency(1e20);
						WriteDriverLatency = MAX(WriteDriver[i].writeLatency, WriteDriverLatency);
					}
				}
			}
			writeLatency = MAX(RowDecMergeNand.readLatency + maxRowDriver, columnDecoderLatency + WriteDriverLatency + chargeLatency);
			resetLatency = writeLatency + CAM_cell->resetPulse;
			setLatency = writeLatency + CAM_cell->setPulse;
			writeLatency += MAX(CAM_cell->resetPulse, CAM_cell->setPulse);
		}
	}
}

void CAM_SubArray::CalculatePower() {
	if (!initialized) {
		cout << "[CAM_SubArray] Error: Require initialization first!" << endl;
	} else if (invalid) {
		cout << "[CAM_SubArray] Error: invalid!" << endl;
		readDynamicEnergy = writeDynamicEnergy = leakage = 1e41;
	} else {

		//////////////////////////////////////////////////////////////////////////////////
		//		   					calc components										//
		//////////////////////////////////////////////////////////////////////////////////

		readDynamicEnergy = writeDynamicEnergy = leakage = 0;

		if(withInputBuf) {
			inputBuf.CalculatePower();
		} else {
			inputBuf.readDynamicEnergy = 0;
			inputBuf.leakage = 0;
		}

		if(withInputEnc) {
			inputEnc.CalculatePower();
		} else {
			inputEnc.readDynamicEnergy = 0;
			inputEnc.leakage = 0;
		}

		// this NAND merges pre-decoder's result, output the WL activation signal
		RowDecMergeNand.CalculatePower();
		// those NAND merges the WL and SL signal
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowDriver[i].CalculatePower();
		}

		// precharge
		precharger.CalculatePower();

		// colmn decoder signal merge
		ColDecMergeNand.CalculatePower();

		senseAmpMuxLev1Nand.CalculatePower();
		senseAmpMuxLev2Nand.CalculatePower();

		if (internalSenseAmp) {
			senseAmp.CalculatePower();
		}
		senseAmpMuxLev1.CalculatePower();
		senseAmpMuxLev2.CalculatePower();

		if (withOutputAcc) {
			outputAcc.CalculatePower();
		}

		if (withPriorityEnc) {
			priorityEnc.CalculatePower();
		}

		if(withOutputBuf) {
			outputBuf.CalculatePower();
		} else {
			outputBuf.readDynamicEnergy = 0;
			outputBuf.leakage = 0;
		}

		//////////////////////////////////////////////////////////////////////////////////
		//		   					calc read and search								//
		//////////////////////////////////////////////////////////////////////////////////
		if (typeSenseAmp == discharge) {
			searchDynamicEnergy = (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousPowerCalculation)
					* (voltagePrecharge * voltagePrecharge - CAM_cell->readVoltage * CAM_cell->readVoltage)* numColumn;
		} else {
			if (CAM_cell->memCellType == SRAM) {
				/* Codes below calculate the SRAM bitline power */
				searchDynamicEnergy = (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousPowerCalculation)
						* voltagePrecharge * voltagePrecharge * numColumn;
			
			} else if (CAM_cell->memCellType == FEFETRAM){
				double FEFETCap = CalculateDrainCap(CAM_cell->widthAccessCMOS * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech);
				//if (CAM_cell->accessType == CMOS_access){
					searchDynamicEnergy = (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousPowerCalculation+ FEFETCap)
						* voltagePrecharge * voltagePrecharge * numColumn/ muxSenseAmp;
				//} else if (CAM_cell->accessType == none_access)


			} else if (CAM_cell->memCellType == MRAM || CAM_cell->memCellType == PCRAM || CAM_cell->memCellType == memristor) {
				if (CAM_cell->readMode == false) {	/* current-sensing */
					/* Use ICCAD 2009 model */
					double resBitlineMux = ColMux[indexMatchline].resNMOSPassTransistor;
					double vpreMin = CAM_cell->readVoltage * resBitlineMux / (resBitlineMux + Col[indexMatchline].res + resMemCellOn);
					double vpreMax = CAM_cell->readVoltage * (resBitlineMux + Col[indexMatchline].res) /
							(resBitlineMux + Col[indexMatchline].res + resMemCellOn);
					searchDynamicEnergy = capCellAccess * vpreMax * vpreMax + ColMux[indexMatchline].capForPreviousPowerCalculation
							* vpreMin * vpreMin + Col[indexMatchline].cap * (vpreMax * vpreMax + vpreMin * vpreMin + vpreMax * vpreMin) / 3;
					searchDynamicEnergy *= numColumn / muxSenseAmp;

				} else {				/* voltage-sensing */
					searchDynamicEnergy = (capCellAccess + Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousPowerCalculation) *
							(voltagePrecharge * voltagePrecharge - voltageMemCellOn * voltageMemCellOn ) * numColumn / muxSenseAmp;
				}
			// } else if (CAM_cell->memCellType ==FEFETRAM){
			// 	double FEFETCap = CalculateDrainCap(CAM_cell->widthAccessCMOS * FEFET_tech->featureSize, NMOS, CAM_cell->widthInFeatureSize * FEFET_tech->featureSize, *FEFET_tech);
			// 	searchDynamicEnergy = (Col[indexMatchline].cap + ColMux[indexMatchline].capForPreviousPowerCalculation + FEFETCap) *
			// 			(voltagePrecharge * voltagePrecharge) * numColumn / muxSenseAmp;
			}// }
		}
		if (CAM_cell->readPower == 0)
			cellReadEnergy = 2 * CAM_cell->CalculateReadPower() * (senseAmp.readLatency + bitlineDelay); /* x2 is because of the reference cell */
		else
			cellReadEnergy = 2 * CAM_cell->readPower * (senseAmp.readLatency + bitlineDelay);
		cellReadEnergy *= numColumn / muxSenseAmp;
		// 	double Singlecellenrgy = 65e-15;
		// cellReadEnergy = Singlecellenrgy*numColumn*numRow;

		energyDriveSearch0 = 0;
		energyDriveSearch1 = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			energyDriveSearch0 += RowDriver[i].readDynamicEnergy / tech->vdd / tech->vdd
					* Row[i].CellPort.volSearch0 * Row[i].CellPort.volSearch0;
			energyDriveSearch1 += RowDriver[i].readDynamicEnergy / tech->vdd / tech->vdd
					* Row[i].CellPort.volSearch1 * Row[i].CellPort.volSearch1;
		}
		searchDynamicEnergy += (energyDriveSearch0 + energyDriveSearch1)/2;
		readDynamicEnergy = searchDynamicEnergy + inputBuf.readDynamicEnergy * numRow + inputEnc.readDynamicEnergy * numRow
				+ cellReadEnergy + ColDecMergeNand.readDynamicEnergy + precharger.readDynamicEnergy
				+ senseAmpMuxLev1Nand.readDynamicEnergy + senseAmpMuxLev2Nand.readDynamicEnergy+ ColMux[indexMatchline].readDynamicEnergy
				+ senseAmp.readDynamicEnergy + senseAmpMuxLev1.readDynamicEnergy + senseAmpMuxLev2.readDynamicEnergy
				+ outputAcc.readDynamicEnergy + priorityEnc.readDynamicEnergy + outputBuf.readDynamicEnergy * numColumn;
		searchDynamicEnergy +=  inputBuf.readDynamicEnergy * numRow + inputEnc.readDynamicEnergy * numRow
				+ cellReadEnergy + ColDecMergeNand.readDynamicEnergy + precharger.readDynamicEnergy
				+ ColMux[indexMatchline].readDynamicEnergy
				+ senseAmp.readDynamicEnergy
				+ outputAcc.readDynamicEnergy + priorityEnc.readDynamicEnergy + outputBuf.readDynamicEnergy * numColumn;
		//////////////////////////////////////////////////////////////////////////////////
		//		   							calc write									//
		//////////////////////////////////////////////////////////////////////////////////
		numBitline = 0;
		indexBitline = 0;
		// default status is using ML as the BL, as it is in the case of JSSC 2t2r
		for(int i=0;i<CAM_cell->camNumCol;i++){
			if (Col[i].CellPort.Type == Bitline) {
				indexBitline = i;
				numBitline++;
			}
		}
		if (CAM_cell->memCellType == SRAM) {
			// since the SRAM cell is not flexible, we can make the coding simpler
			double capSRAMin;
			double capSRAMout;
			CalculateGateCapacitance(INV, 1, CAM_cell->widthSRAMCellNMOS, CAM_cell->widthSRAMCellPMOS,
					tech->featureSize*MAX_TRANSISTOR_HEIGHT, *tech, &capSRAMin, &capSRAMout);
			cellResetEnergy = (capSRAMin + capSRAMout) * tech->vdd * tech->vdd;
			cellResetEnergy += (Col[indexBitline].cap + ColMux[indexBitline].capForPreviousPowerCalculation)
					* tech->vdd  * tech->vdd;
			cellSetEnergy = cellResetEnergy;
		} else if (CAM_cell->memCellType == MRAM || CAM_cell->memCellType == PCRAM || CAM_cell->memCellType == memristor || CAM_cell->memCellType == FEFETRAM) {
			/* Ignore the dynamic transition during the SET/RESET operation */
			/* Assume that the cell resistance keeps high for worst-case power estimation */
			CAM_cell->CalculateWriteEnergy();

			// TODO: MLC MRS set
			resetEnergyPerBit = CAM_cell->resetEnergy;
			setEnergyPerBit = CAM_cell->setEnergy;
			for(int i=0;i<CAM_cell->camNumCol;i++){
				// since each line has a description of the set/reset voltage already, we do not need setMode and resetMode in original nvsim anymore
				// for current set/reset mode, it has to be converted to the description of voltage set/reset in the cell configuration file
				// for example, the matchline voltage will be zero when writing in ISSCC'15 3t1r
				setEnergyPerBit += (capCellAccess + Col[indexBitline].cap + ColMux[i].capForPreviousPowerCalculation)
										* Col[i].CellPort.volSetLRS * Col[i].CellPort.volSetLRS;
				resetEnergyPerBit += (capCellAccess + Col[indexBitline].cap + ColMux[i].capForPreviousPowerCalculation)
										* Col[i].CellPort.volReset * Col[i].CellPort.volReset;

			}
			

			if (CAM_cell->memCellType == PCRAM) { //PCRAM write energy
				if (inputParameter->writeScheme == write_and_verify) {
					/*TO-DO: write and verify programming */
				} else {
					cellResetEnergy = resetEnergyPerBit / SHAPER_EFFICIENCY_CONSERVATIVE;
					cellSetEnergy = setEnergyPerBit / SHAPER_EFFICIENCY_CONSERVATIVE;  /* Due to the shaper inefficiency */
				}
			} else { //MRAM and memristor + FEFET write energy

				cellResetEnergy = resetEnergyPerBit / SHAPER_EFFICIENCY_AGGRESSIVE;
				cellSetEnergy = setEnergyPerBit / SHAPER_EFFICIENCY_AGGRESSIVE;  /* Due to the shaper inefficiency */
			}
			leakage = 0;                       //TO-DO: cell leaks during read/write operation
		}
		writeDynamicEnergy = MAX(cellResetEnergy, cellSetEnergy);
		cellResetEnergy *= numColumn / muxSenseAmp / muxOutputLev1 / muxOutputLev2;
		cellSetEnergy *= numColumn / muxSenseAmp / muxOutputLev1 / muxOutputLev2;
		writeDynamicEnergy *= numColumn / muxSenseAmp / muxOutputLev1 / muxOutputLev2;

		// TODO: does not calculate MLC case
		setDynamicEnergy = resetDynamicEnergy = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			setDynamicEnergy += RowDriver[i].writeDynamicEnergy / tech->vdd / tech->vdd
					* Row[i].CellPort.volSetLRS * Row[i].CellPort.volSetLRS;
			resetDynamicEnergy += RowDriver[i].writeDynamicEnergy / tech->vdd / tech->vdd
					* Row[i].CellPort.volReset * Row[i].CellPort.volReset;
			writeDynamicEnergy += RowDriver[i].writeDynamicEnergy /*/ tech->vdd / tech->vdd
				* (Row[i].CellPort.volReset + Row[i].CellPort.volSetLRS) * (Row[i].CellPort.volSetLRS + Row[i].CellPort.volReset)*/;
			
		}
		for(int i=0;i<CAM_cell->camNumCol;i++){
			if(indexBitline == i && i > 0) {
				// this is skipping the matchline that is not used in writing
				continue;
			}
			setDynamicEnergy += ColMux[i].writeDynamicEnergy;
			resetDynamicEnergy += ColMux[i].writeDynamicEnergy;
			writeDynamicEnergy += ColMux[i].writeDynamicEnergy;
		}
		if (indexBitline == 0) {
			// ML is also used in the writing operations
			setDynamicEnergy += senseAmp.writeDynamicEnergy;
			resetDynamicEnergy += senseAmp.writeDynamicEnergy;
			writeDynamicEnergy += senseAmp.writeDynamicEnergy;
		}

		WriteDriverDyn = 0;
		WriteDriverLeakage = 0;
		if (withWriteDriver) {
			for(int i=0;i<CAM_cell->camNumCol;i++){
				if (WriteDriver[i].initialized) {
					WriteDriver[i].CalculatePower();
					WriteDriverDyn += WriteDriver[i].writeDynamicEnergy;
				}
			}
		}

		writeDynamicEnergy += ColDecMergeNand.writeDynamicEnergy + WriteDriverDyn
				+ senseAmpMuxLev1Nand.writeDynamicEnergy + senseAmpMuxLev2Nand.writeDynamicEnergy
				+ senseAmpMuxLev1.writeDynamicEnergy + senseAmpMuxLev2.writeDynamicEnergy;
		// cout << "Write Dynamic Energy in Subarray: " << writeDynamicEnergy << endl;
		/* for assymetric RESET and SET latency calculation only */
		setDynamicEnergy += cellSetEnergy + ColDecMergeNand.writeDynamicEnergy
				+ senseAmpMuxLev1Nand.writeDynamicEnergy + senseAmpMuxLev2Nand.writeDynamicEnergy
				+ senseAmpMuxLev1.writeDynamicEnergy + senseAmpMuxLev2.writeDynamicEnergy;
		resetDynamicEnergy += setDynamicEnergy + ColDecMergeNand.writeDynamicEnergy
				+ senseAmpMuxLev1Nand.writeDynamicEnergy + senseAmpMuxLev2Nand.writeDynamicEnergy
				+ senseAmpMuxLev1.writeDynamicEnergy + senseAmpMuxLev2.writeDynamicEnergy;

		
		//////////////////////////////////////////////////////////////////////////////////
		//		   							calc leakage								//
		//////////////////////////////////////////////////////////////////////////////////

		// leakage inside the cell
		if (CAM_cell->memCellType == SRAM) {
			leakage = CalculateGateLeakage(INV, 1, CAM_cell->widthSRAMCellNMOS * tech->featureSize,
					CAM_cell->widthSRAMCellPMOS * tech->featureSize, inputParameter->temperature, *tech)
							* tech->vdd * 2;	/* two inverters per SRAM cell */
			leakage += CalculateGateLeakage(INV, 1, CAM_cell->widthAccessCMOS * tech->featureSize, 0,
							inputParameter->temperature, *tech) * tech->vdd;	/* two accesses NMOS, but combined as one with vdd crossed */
			leakage += CalculateGateLeakage(INV, 1, CAM_cell->widthAccessCMOS * tech->featureSize, 0,
										inputParameter->temperature, *tech) * tech->vdd;	/* two accesses NMOS, but combined as one with vdd crossed */
			leakage += CalculateGateLeakage(INV, 1, CAM_cell->camWidthMatchTran * tech->featureSize, 0,
										inputParameter->temperature, *tech) * tech->vdd;	/* two accesses NMOS, but combined as one with vdd crossed */
			leakage *= 2;
		} else if (CAM_cell->memCellType == MRAM || CAM_cell->memCellType == PCRAM || CAM_cell->memCellType == memristor || CAM_cell->memCellType ==FEFETRAM) {
			// TODO: i am here
			// basically count the transistors in the cell
			// the trick here is that every thransistor in the cell should be connected by some line to the gate to control it
			// the exception is the matchline transistor in cmos-access, like ISSCC'15 3t1r
			leakage = 0;
			for(int i=0;i<CAM_cell->camNumRow;i++){
				if (Row[i].CellPort.ConnectedRegoin == gate && Row[i].CellPort.leak) {
					if (Row[i].CellPort.isNMOS)
						leakage += CalculateGateLeakage(INV, 1, Row[i].CellPort.widthCmos * tech->featureSize, 0,
							inputParameter->temperature, *tech) * tech->vdd;
					else
						leakage += CalculateGateLeakage(INV, 1, 0, Row[i].CellPort.widthCmos * tech->featureSize,
													inputParameter->temperature, *tech) * tech->vdd;
				}
			}
			for(int i=0;i<CAM_cell->camNumCol;i++){
				if (Col[i].CellPort.ConnectedRegoin == gate && Col[i].CellPort.leak) {
					if (Col[i].CellPort.isNMOS)
						leakage += CalculateGateLeakage(INV, 1, Col[i].CellPort.widthCmos * tech->featureSize, 0,
							inputParameter->temperature, *tech) * tech->vdd;
					else
						leakage += CalculateGateLeakage(INV, 1, 0, Col[i].CellPort.widthCmos * tech->featureSize,
													inputParameter->temperature, *tech) * tech->vdd;
				}
			}
			if (CAM_cell->accessType == CMOS_access) {
				leakage += CalculateGateLeakage(INV, 1, CAM_cell->camWidthMatchTran * tech->featureSize,  0,
						inputParameter->temperature, *tech) * tech->vdd;
			}
		} else {
			invalid = true;
			cout <<"[CAM_SubArray] Error: cell type input error" <<endl;
			return;
		}
		leakage *= numRow * numColumn;

		//////////////////////////
		double leak = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			leakage += RowDriver[i].leakage;
			leak += RowDriver[i].leakage;
		}
		for(int i=0;i<CAM_cell->camNumCol;i++){
			leakage += ColMux[i].leakage;
			leak += ColMux[i].leakage;
		}

		leakage += inputBuf.leakage * numColumn + inputEnc.leakage + precharger.leakage + senseAmpMuxLev1Nand.leakage
				+ senseAmpMuxLev2Nand.leakage + ColDecMergeNand.leakage + WriteDriverLeakage + senseAmp.leakage
				+ senseAmpMuxLev1.leakage + senseAmpMuxLev2.leakage + outputAcc.leakage + priorityEnc.leakage
				+ outputBuf.leakage * numColumn;
		leak += inputBuf.leakage * numColumn + inputEnc.leakage + precharger.leakage + senseAmpMuxLev1Nand.leakage
				+ senseAmpMuxLev2Nand.leakage + ColDecMergeNand.leakage + WriteDriverLeakage + senseAmp.leakage
				+ senseAmpMuxLev1.leakage + senseAmpMuxLev2.leakage + outputAcc.leakage + priorityEnc.leakage
				+ outputBuf.leakage * numColumn;
		//cout << "leakage energy of peripherals: " << leak * 1e9 << endl;
	}

	

}

void CAM_SubArray::PrintProperty() {
	cout << "CAMSubarray Properties:" << endl;
	//FunctionUnit::PrintProperty();
	cout << "numRow:" << numRow << " numColumn:" << numColumn << endl;
	// area wise
	cout << "Input Encoder Area:" << inputEnc.area*1e12 << " um^2 (" << inputEnc.area/area*100 << "%)" << endl;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		cout << "Row Driver " << i << " Area:" << RowDriver[i].area*1e12 << " um^2 (" << RowDriver[i].area/area*100 << "%)" <<endl;
	}
	cout << "lenWordline * lenBitline = " << lenRow*1e6 << " um * " << lenCol*1e6 << " um = " << lenRow * lenCol * 1e12
			<< " um^2 (" << lenRow * lenCol/area*100 << "%)" << endl;
	cout << "MergeDecoderNand Area:" << RowDecMergeNand.area*1e12 << " um^2 (" << RowDecMergeNand.area/area*100 << "%)" << endl;
	for(int i=0;i<CAM_cell->camNumCol;i++){
		cout << "Col Mux " << i << " Area:" << ColMux[i].area*1e12 << " um^2 (" << ColMux[i].area/area*100 << "%)" << endl;
	}
	cout << "Write Driver Area:" << WriteDriverArea*1e12 << " um^2 (" << WriteDriverArea/area*100 << "%)" << endl;
	cout << "Mux Area:" << inputEnc.area*1e12 << " um^2" << endl;
	cout << "Sense Amplifier Area:" << senseAmp.area*1e12 << " um^2 (" << senseAmp.area/area*100 << "%)" << endl;
	cout << "Output Acc Area:" << outputAcc.area*1e12 << " um^2 (" << outputAcc.area/area*100 << "%)" << endl;
	cout << "Priority Encoder Area:" << priorityEnc.area*1e12 << " um^2 (" << priorityEnc.area/area*100 << "%)" << endl;

	// TODO: not done with debug interface yet
	cout << "bitlineDelay: " << bitlineDelay*1e12 << "ps" << endl;
	cout << "chargeLatency: " << chargeLatency*1e12 << "ps" << endl;
	cout << "columnDecoderLatency: " << columnDecoderLatency*1e12 << "ps" << endl;
	cout << "errors exist here!" << endl;
}

CAM_SubArray & CAM_SubArray::operator=(const CAM_SubArray &rhs) {
	// TODO: did not go through clearly
	height = rhs.height;
	width = rhs.width;
	area = rhs.area;
	readLatency = rhs.readLatency;
	writeLatency = rhs.writeLatency;
	readDynamicEnergy = rhs.readDynamicEnergy;
	writeDynamicEnergy = rhs.writeDynamicEnergy;
	resetLatency = rhs.resetLatency;
	setLatency = rhs.setLatency;
	resetDynamicEnergy = rhs.resetDynamicEnergy;
	setDynamicEnergy = rhs.setDynamicEnergy;
	cellReadEnergy = rhs.cellReadEnergy;
	cellResetEnergy = rhs.cellResetEnergy;
	cellSetEnergy = rhs.cellSetEnergy;
	leakage = rhs.leakage;
	initialized = rhs.initialized;
	numRow = rhs.numRow;
	numColumn = rhs.numColumn;
	multipleRowPerSet = rhs.multipleRowPerSet;
	split = rhs.split;
	muxSenseAmp = rhs.muxSenseAmp;
	internalSenseAmp = rhs.internalSenseAmp;
	muxOutputLev1 = rhs.muxOutputLev1;
	muxOutputLev2 = rhs.muxOutputLev2;

	voltageSense = rhs.voltageSense;
	senseVoltage = rhs.senseVoltage;
	numSenseAmp = rhs.numSenseAmp;
	resCellAccess = rhs.resCellAccess;
	capCellAccess = rhs.capCellAccess;
	bitlineDelay = rhs.bitlineDelay;
	chargeLatency = rhs.chargeLatency;
	columnDecoderLatency = rhs.columnDecoderLatency;
	bitlineDelayOn = rhs.bitlineDelayOn;
	bitlineDelayOff = rhs.bitlineDelayOff;
	/////////////////////////////
	tau = rhs.tau;
	Basetau = rhs.Basetau;
	SenseTime = rhs.SenseTime;
	MatchlineSenseMargin = rhs.MatchlineSenseMargin;
	//////////////////////////////
	resInSerialForSenseAmp = rhs.resInSerialForSenseAmp;
	resEquivalentOn = rhs.resEquivalentOn;
	resEquivalentOff = rhs.resEquivalentOff;
	resMemCellOff = rhs.resMemCellOff;
	resMemCellOn = rhs.resMemCellOn;

	senseAmpMuxLev1 = rhs.senseAmpMuxLev1;
	senseAmpMuxLev2 = rhs.senseAmpMuxLev2;
	precharger = rhs.precharger;
	senseAmp = rhs.senseAmp;

	inputEnc = rhs.inputEnc;
	RowDecMergeNand = rhs.RowDecMergeNand;
	for(int i=0;i<MAX_PORT;i++){
		RowDriver[i] = rhs.RowDriver[i];
	}
	precharger = rhs.precharger;
	ColDecMergeNand = rhs.ColDecMergeNand;
	for(int i=0;i<MAX_PORT;i++){
		ColMux[i] = rhs.ColMux[i];
	}
	senseAmpMuxLev1Nand = rhs.senseAmpMuxLev1Nand;
	senseAmpMuxLev2Nand = rhs.senseAmpMuxLev2Nand;
	outputAcc = rhs.outputAcc;
	priorityEnc = rhs.priorityEnc;

	numRow = rhs.numRow;
	numColumn = rhs.numColumn;
	lenRow = rhs.lenRow;
	lenCol = rhs.lenCol;
	indexMatchline = rhs.indexMatchline;
	indexBitline = rhs.indexBitline;
	numBitline = rhs.numBitline;
	for(int i=0;i<MAX_PORT;i++){
		Row[i] = rhs.Row[i];
		Col[i] = rhs.Col[i];
	}
	withInputEnc = rhs.withInputEnc;
	typeInputEnc = rhs.typeInputEnc;
	customInputEnc = rhs.customInputEnc;
	DecMergeOptLevel = rhs.DecMergeOptLevel;
	RowDecMergeInv = rhs.RowDecMergeInv;
	DriverOptLevel = rhs.DriverOptLevel;
	voltagePrecharge = rhs.voltagePrecharge;
	typeSenseAmp = rhs.typeSenseAmp;
	customSenseAmp = rhs.customSenseAmp;
	senseMargin = rhs.senseMargin;
	withOutputAcc = rhs.withOutputAcc;
	withPriorityEnc = rhs.withPriorityEnc;
	PriorityOptLevel = rhs.PriorityOptLevel;
	withWriteDriver = rhs.withWriteDriver;
	WriteDriverArea = rhs.WriteDriverArea;
	WriteDriverLatency = rhs.WriteDriverLatency;
	WriteDriverDyn = rhs.WriteDriverDyn;
	WriteDriverLeakage = rhs.WriteDriverLeakage;
	withInputBuf = rhs.withInputBuf;
	withOutputBuf = rhs.withOutputBuf;
	senseAmpLatency = rhs.senseAmpLatency;
	referDelay = rhs.referDelay;
	volMatchDrop = rhs.volMatchDrop;
	volAllMissDrop = rhs.volAllMissDrop;

	decoderLatency = rhs.decoderLatency;
	bitlineRamp = rhs.bitlineRamp;
	chargeLatency = rhs.chargeLatency;
	searchLatency = rhs.searchLatency;
	searchDynamicEnergy = rhs.searchDynamicEnergy;
	energyDriveSearch0 = rhs.energyDriveSearch0;
	energyDriveSearch1 = rhs.energyDriveSearch1;
	rampOutput = rhs.rampOutput;
	resetEnergyPerBit = rhs.resetEnergyPerBit;
	setEnergyPerBit = rhs.setEnergyPerBit;
	return *this;
}


