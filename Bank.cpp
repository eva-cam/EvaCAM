#include "Bank.h"
#include "CAM_SubArray.h"
#include "Mat.h"
#include "formula.h"
#include "global.h"
#include "macros.h"

Bank::Bank() {
	// TODO Auto-generated constructor stub
	initialized = false;
	invalid = false;
}

Bank::~Bank() {
	// TODO Auto-generated destructor stub
}

void Bank::PrintProperty() {
	cout << "Bank Properties:" << endl;
	FunctionUnit::PrintProperty();
}

void Bank::debug() {
	cout << & mat << endl;
	cout << & numActiveSubarrayPerRow << endl;
}

 void Bank::printbreakdown() {
	double latency = 0;
	if (inputParameter->NoPrechargeInc) {
		latency = mat.subarray.bitlineDelay + mat.subarray.ColMux[mat.subarray.indexMatchline].readLatency
				+ mat.subarray.senseAmpLatency + mat.subarray.outputAcc.readLatency;
	} else {
		latency = mat.subarray.searchLatency;
	}

	double energy = mat.subarray.searchDynamicEnergy;
	if (inputParameter->IncludeLeakge) {
		energy += mat.leakage * latency;
	}
	if (inputParameter->scaledVoltage > 0) {
		energy = energy / tech->vdd / tech->vdd * inputParameter->scaledVoltage * inputParameter->scaledVoltage;
	}




		// ///////////////       AREA              //////////////

		// cout << endl;
		cout << "========= Subarray Area Breakdown =========" << endl;
		cout << " |--- Total Cell Area         = " << TO_SQM(mat.subarray.lenRow * mat.subarray.lenCol) << endl;
		
		cout << " |--- Input Buffer Area       = " << TO_SQM(mat.subarray.inputBuf.area * mat.subarray.numRow) << endl;
		cout << " |--- Input Encoder Area      = " << TO_SQM(mat.subarray.inputEnc.area) << endl;
		cout << " |--- Row Decoder Area        = " << TO_SQM(mat.subarray.RowDecMergeNand.area) << endl;
		double RowDriverArea = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowDriverArea += mat.subarray.RowDriver[i].area;
		}
		cout << " |--- Row Driver Area         = " << TO_SQM(RowDriverArea) << endl;
		cout << " |--- Precharger Area         = " << TO_SQM(mat.subarray.precharger.area) << endl;
		if(inputParameter->withWriteDriver == true){
		cout << " |--- Write Driver Area       = " << TO_SQM(mat.subarray.WriteDriverArea) << endl;
		}

		double ColMuxArea = 0;
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMuxArea += mat.subarray.ColMux[i].area;
		}
		cout << " |--- Column Mux Area         = " << TO_SQM(ColMuxArea)<< endl;
		cout << " |--- Sense Amplifier Area    = " << TO_SQM(mat.subarray.senseAmp.area) << endl;
		cout << " |--- Mux of SA               = " << TO_SQM(mat.subarray.senseAmpMuxLev1.area + mat.subarray.senseAmpMuxLev2.area
				+ mat.subarray.senseAmpMuxLev1Nand.area + mat.subarray.senseAmpMuxLev2Nand.area) << endl;
		cout << " |--- Output Accumulator Area = " << TO_SQM(mat.subarray.outputAcc.area * mat.subarray.numColumn / mat.subarray.muxSenseAmp) << endl;
		if(inputParameter->withPriorityEnc == true){
			cout << " |--- Priority Encoder Area   = " << TO_SQM(mat.subarray.priorityEnc.area) << endl;
		}
		cout << " |--- Output Buffer Area      = " << TO_SQM(mat.subarray.outputBuf.area * mat.subarray.numColumn / mat.subarray.muxSenseAmp) << endl;
		cout << endl;

		///////////////       latency              //////////////

		cout << "========= Search Latency Breakdown =========" << endl;
		// cout << endl;
		// cout << " |- Mat Search Latency = " << TO_SECOND(mat.subarray.readLatency)  << endl;
		// cout << " 	|-- Subarray Search Latency   = " << TO_SECOND(mat.subarray.readLatency) << endl;
		// cout << " 	|-- Predecoder Search Latency = " << TO_SECOND(mat.rowPredecoderBlock1.readLatency + mat.rowPredecoderBlock2.readLatency + mat.bitlineMuxPredecoderBlock1.readLatency + mat.bitlineMuxPredecoderBlock2.readLatency +
		// 		mat.senseAmpMuxLev1PredecoderBlock1.readLatency + mat.senseAmpMuxLev1PredecoderBlock2.readLatency +
		// 		mat.senseAmpMuxLev2PredecoderBlock1.readLatency + mat.senseAmpMuxLev2PredecoderBlock2.readLatency) << endl;
		cout << " |--- Input Encoder Latency      = " << TO_SECOND(mat.subarray.inputEnc.readLatency) << endl;
		cout << " |--- Row Decoder Latency        = " << TO_SECOND(mat.subarray.RowDecMergeNand.readLatency +mat.subarray.senseAmpMuxLev1Nand.readLatency +mat.subarray.senseAmpMuxLev2Nand.readLatency) << endl;
		double RowDriverLatency = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowDriverLatency = MAX(mat.subarray.RowDriver[i].readLatency, readLatency);
		}
		cout << " |--- Row Driver Latency         = " << TO_SECOND(RowDriverLatency) << endl;
		cout << " |--- Precharger Latency         = " << TO_SECOND(mat.subarray.precharger.readLatency) << endl;
		cout << " |--- Bitline Latency            = " << TO_SECOND(mat.subarray.bitlineDelay) << endl;
		double ColDriverLatency = 0;
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColDriverLatency = MAX(mat.subarray.ColMux[i].readLatency, readLatency);
		}
		cout << " |--- Column Mux Latency         = " << TO_SECOND(ColDriverLatency) << endl;
		cout << " |--- Sense Amplifier Latency    = " << TO_SECOND(mat.subarray.senseAmpLatency) << endl;
		cout << " |--- MUX of SA Latency          = " << TO_SECOND(mat.subarray.senseAmpMuxLev1.readLatency + mat.subarray.senseAmpMuxLev2.readLatency) << endl;
		if(inputParameter->withOutputAcc = true){
			cout << " |--- Output Accumulator Latency = " << TO_SECOND(mat.subarray.outputAcc.readLatency) << endl;
		}
		if(inputParameter->withPriorityEnc = true){
			cout << " |--- Priority Encoder Latency   = " << TO_SECOND(mat.subarray.priorityEnc.readLatency) << endl;
		}
		cout << endl;

		///////////////       energy              //////////////
		cout << "========= Search Dynamic Energy Breakdown =========" << endl;
		// cout << " |-- Mat Dynamic Search Energy  = " << TO_JOULE(mat.readDynamicEnergy) << endl;
		// cout << " 	|-- Subarray Search Energy   = " << TO_JOULE(mat.subarray.readDynamicEnergy) << endl;
		//cout << "Search Dynamic Energy" << TO_JOULE(mat.subarray.searchAverage) << endl;
		// cout << " 	|-- Predecoder Energy = " << TO_JOULE(mat.rowPredecoderBlock1.readDynamicEnergy + mat.rowPredecoderBlock2.readDynamicEnergy + mat.bitlineMuxPredecoderBlock1.readDynamicEnergy + mat.bitlineMuxPredecoderBlock2.readDynamicEnergy +
		// 		mat.senseAmpMuxLev1PredecoderBlock1.readDynamicEnergy + mat.senseAmpMuxLev1PredecoderBlock2.readDynamicEnergy +
		// 		mat.senseAmpMuxLev2PredecoderBlock1.readDynamicEnergy + mat.senseAmpMuxLev2PredecoderBlock2.readDynamicEnergy) << endl;
		cout << " |--- Input Encoder Dynamic Energy      = " << TO_JOULE(mat.subarray.inputEnc.readDynamicEnergy) << endl;
		cout << " |--- Row Decoder Dynamic Energy        = " << TO_JOULE(mat.subarray.RowDecMergeNand.readDynamicEnergy) << endl;
		double RowSearchDynamicEnergy = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowSearchDynamicEnergy += mat.subarray.RowDriver[i].readDynamicEnergy;
		}
		cout << " |--- RowDriver Dynamic Energy          = " << TO_JOULE(RowSearchDynamicEnergy) << endl; //For FeFET *16
		cout << " |--- Precharger Dynamic Energy         = " << TO_JOULE(mat.subarray.precharger.readDynamicEnergy) << endl;
		// TODO: not ly the breakdown
		cout << " |--- Cell Read Energy                  = " << TO_JOULE(mat.subarray.cellReadEnergy) << endl;
		double ColMuxreadDynamicEnergy = 0;
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColMuxreadDynamicEnergy += mat.subarray.ColMux[i].readDynamicEnergy;
		}
		cout << " |--- Column Mux Dynamic Energy         = " << TO_JOULE(ColMuxreadDynamicEnergy) << endl;
		cout << " |--- Sense Amplifier Dynamic Energy    = " << TO_JOULE(mat.subarray.senseAmp.readDynamicEnergy) << endl;
		cout << " |--- MUX of SA Dynamic Energy          = " << TO_JOULE(mat.subarray.senseAmpMuxLev1.readDynamicEnergy + mat.subarray.senseAmpMuxLev2.readDynamicEnergy) << endl;
		if(inputParameter->withOutputAcc == true){
			cout << " |--- Output Accumulator Dynamic Energy = " << TO_JOULE(mat.subarray.outputAcc.readDynamicEnergy) << endl;
		}
		if(inputParameter->withPriorityEnc == true){
			cout << " |--- Priority Encoder Dynamic Energy   = " << TO_JOULE(mat.subarray.priorityEnc.readDynamicEnergy) << endl;
		}
		cout << endl;
		

	///////////////////xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx////////////////////////////

		///////////////       energy              //////////////
		cout << "========= Write Dynamic Energy Breakdown =========" << endl;
		// cout << " |- Mat Write Search Energy  = " << TO_JOULE(mat.writeDynamicEnergy) << endl;
		// cout << "  |-- Subarray Dynamic Energy            = " << TO_JOULE(mat.subarray.writeDynamicEnergy) << endl;
		// cout << "  |-- Predecoder Dynamic Energy          = " << TO_JOULE(mat.rowPredecoderBlock1.writeDynamicEnergy + mat.rowPredecoderBlock2.writeDynamicEnergy + mat.bitlineMuxPredecoderBlock1.writeDynamicEnergy+ mat.bitlineMuxPredecoderBlock2.writeDynamicEnergy+
		// 		mat.senseAmpMuxLev1PredecoderBlock1.writeDynamicEnergy+ mat.senseAmpMuxLev1PredecoderBlock2.writeDynamicEnergy +
		// 		mat.senseAmpMuxLev2PredecoderBlock1.writeDynamicEnergy+ mat.senseAmpMuxLev2PredecoderBlock2.writeDynamicEnergy) << endl;
		cout << " |--- Cell Reset Energy                  = " << TO_JOULE(mat.subarray.cellResetEnergy) << endl;
		cout << " |--- Cell Set Energy                    = " << TO_JOULE(mat.subarray.cellSetEnergy) << endl;
		cout << " |--- Input Encoder Dynamic Energy       = " << TO_JOULE(mat.subarray.inputEnc.writeDynamicEnergy) << endl;
		cout << " |--- Row Decoder Dynamic Energy         = " << TO_JOULE(mat.subarray.RowDecMergeNand.writeDynamicEnergy) << endl;
		double RowWriteDynamicEnergy = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			RowWriteDynamicEnergy += mat.subarray.RowDriver[i].writeDynamicEnergy*16;
		}
		cout << " |--- RowDriver Dynamic Energy           = " << TO_JOULE(RowWriteDynamicEnergy) << endl;
		cout << " |--- Precharger Dynamic Energy          = " << TO_JOULE(mat.subarray.precharger.writeDynamicEnergy) << endl;
		// TODO: not really the breakdown
		double ColWriteDynamicEnergy = 0;
		for(int i=0;i<CAM_cell->camNumCol;i++){
			ColWriteDynamicEnergy += mat.subarray.ColMux[i].writeDynamicEnergy;
		}
		cout << " |--- Column Mux Dynamic Energy          = " << TO_JOULE(ColWriteDynamicEnergy) << endl;
		cout << " |--- Sense Amplifier Dynamic Energy     = " << TO_JOULE(mat.subarray.senseAmp.writeDynamicEnergy) << endl;
		cout << " |--- MUX of SA Dynamic Energy           = " << TO_JOULE(mat.subarray.senseAmpMuxLev1.writeDynamicEnergy + mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
		if(inputParameter->withOutputAcc == true){
			cout << " |--- Output Accumulator Dynamic Energy  = " << TO_JOULE(mat.subarray.outputAcc.writeDynamicEnergy) << endl;
		}
		if(inputParameter->withPriorityEnc == true){
			cout << " |--- Priority Encoder Dynamic Energy    = " << TO_JOULE(mat.subarray.priorityEnc.writeDynamicEnergy) << endl;		
		}
		cout << endl;
	

	///////////////////xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx////////////////////////////

		///////////////       leakage              //////////////
		cout << "============ Leakage Breakdown ============" << endl;
		// cout << " |- Mat Leakage =  " << TO_WATT(mat.leakage) << endl;
		// cout << " 	|-- Subarray Leakage   = " << TO_WATT(mat.subarray.leakage) << endl;
		// cout << " 	|-- Predocoder Leakage = " << TO_WATT(mat.rowPredecoderBlock1.leakage + mat.rowPredecoderBlock2.leakage +
		// 		mat.bitlineMuxPredecoderBlock1.leakage + mat.bitlineMuxPredecoderBlock2.leakage +
		// 		mat.senseAmpMuxLev1PredecoderBlock1.leakage + mat.senseAmpMuxLev1PredecoderBlock2.leakage +
		// 		mat.senseAmpMuxLev2PredecoderBlock1.leakage + mat.senseAmpMuxLev2PredecoderBlock2.leakage ) << endl;
		
		cout << " |--- Input Encoder Leakage      = " << TO_WATT(mat.subarray.inputEnc.leakage) << endl;
		cout << " |--- Row Decoder Leakage        = " << TO_WATT(mat.subarray.RowDecMergeNand.leakage) << endl;
		double leakage = 0;
		for(int i=0;i<CAM_cell->camNumRow;i++){
			leakage += mat.subarray.RowDriver[i].leakage;
		}
		cout << " |--- Row Driver Leakage         = " << TO_WATT(leakage) << endl;
		cout << " |--- Precharger Leakage         = " << TO_WATT(mat.subarray.precharger.leakage) << endl;
		leakage = 0;
		for(int i=0;i<CAM_cell->camNumCol;i++){
			leakage += mat.subarray.ColMux[i].leakage;
		}
		cout << " |--- Column Mux Leakage         = " << TO_WATT(leakage) << endl;
		cout << " |--- Sense Amplifier Leakage    = " << TO_WATT(mat.subarray.senseAmp.leakage) << endl;
		cout << " |--- MUX of SA Leakage          = " << TO_WATT(mat.subarray.senseAmpMuxLev1.leakage + mat.subarray.senseAmpMuxLev2.leakage) << endl;
		if(inputParameter->withOutputAcc == true){
			cout << " |--- Output Accumulator Leakage = " << TO_WATT(mat.subarray.outputAcc.leakage) << endl;
		}
		if(inputParameter->withPriorityEnc == true){
			cout << " |--- Priority Encoder Leakage   = " << TO_WATT(mat.subarray.priorityEnc.leakage) << endl;		
		}		



 }

Bank & Bank::operator=(const Bank &rhs) {
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
	cellSetEnergy = rhs.cellSetEnergy;
	cellResetEnergy = rhs.cellResetEnergy;
	leakage = rhs.leakage;
	initialized = rhs.initialized;
	invalid = rhs.invalid;
	numRowMat = rhs.numRowMat;
	numColumnMat = rhs.numColumnMat;
	capacity = rhs.capacity;
	blockSize = rhs.blockSize;
	associativity = rhs.associativity;
	numRowPerSet = rhs.numRowPerSet;
	numActiveMatPerRow = rhs.numActiveMatPerRow;
	numActiveMatPerColumn = rhs.numActiveMatPerColumn;
	muxSenseAmp = rhs.muxSenseAmp;
	internalSenseAmp = rhs.internalSenseAmp;
	muxOutputLev1 = rhs.muxOutputLev1;
	muxOutputLev2 = rhs.muxOutputLev2;
	areaOptimizationLevel = rhs.areaOptimizationLevel;
	memoryType = rhs.memoryType;
	camType = rhs.camType;
	searchFunction = rhs.searchFunction;
	numRowSubarray = rhs.numRowSubarray;
	numColumnSubarray = rhs.numColumnSubarray;
	numActiveSubarrayPerRow = rhs.numActiveSubarrayPerRow;
	numActiveSubarrayPerColumn = rhs.numActiveSubarrayPerColumn;
	mat = rhs.mat;
	searchLatency = rhs.searchLatency;
	searchDynamicEnergy = rhs.searchDynamicEnergy;
	numBitSerial = rhs.numBitSerial;
	return *this;
}
