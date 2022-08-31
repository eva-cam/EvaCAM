#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <math.h>
#include "InputParameter.h"
#include "MemCell.h"
#include "RowDecoder.h"
#include "Precharger.h"
#include "OutputDriver.h"
#include "SenseAmp.h"
#include "Technology.h"
#include "BasicDecoder.h"
#include "PredecodeBlock.h"
#include "SubArray.h"
#include "Mat.h"
#include "BankWithHtree.h"
#include "BankWithoutHtree.h"
#include "Wire.h"
#include "Result.h"
#include "formula.h"
#include "macros.h"
#include "CAM_Cell.h"
#include "CAM_Result.h"

using namespace std;

InputParameter *inputParameter;
Technology *tech;
Technology *FEFET_tech;
MemCell *cell;
Wire *localWire;
Wire *globalWire;


CAM_MemCell *CAM_cell;
CAM_Opt CAM_opt;
//void mat_only_print(Mat);




void applyConstraint();

int main(int argc, char *argv[])
{
	std::setw(10);
	cout << fixed << setprecision(3);
	string inputFileName;

	if (argc == 1) {
		inputFileName = "nvsim.cfg";
		cout << "Default configuration file (nvsim.cfg) is loaded" << endl;
	} else if (argc == 2) {
		inputFileName = argv[1];
		cout << "User-defined configuration file (" << inputFileName << ") is loaded" << endl;
	} else {
		cout << "[NVSIM Error]: Please use the correct format as follows" << endl;
		cout << "  Use the default configuration: " << argv[0] << endl;
		cout << "  Use the customized configuration: " << argv[0] << " <.cfg file>"  << endl;
		exit(-1);
	}
	cout << endl;

	inputParameter = new InputParameter();
	RESTORE_SEARCH_SIZE;
	inputParameter->ReadInputParameterFromFile(inputFileName);
	CAM_cell = new CAM_MemCell();
	CAM_cell->ReadCellFromFile(inputParameter->fileMemCell);
	CAM_cell->PrintCell();

	tech = new Technology();
	tech->Initialize(inputParameter->processNode, inputParameter->deviceRoadmap);

        FEFET_tech = tech;

        if (CAM_cell->memCellType == FEFETRAM) {
            FEFET_tech = new Technology();
            FEFET_tech->Initialize(inputParameter->processNode, FEFET);
        }
	Technology techHigh;
	double alpha = 0;
	
	if (inputParameter->processNode > 200){
		// TO-DO: technology node > 200 nm
	} else if (inputParameter->processNode > 120) { // 120 nm < technology node <= 200 nm
		techHigh.Initialize(200, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 120.0) / 60;
	} else if (inputParameter->processNode > 90) { // 90 nm < technology node <= 120 nm
		techHigh.Initialize(120, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 90.0) / 30;
	} else if (inputParameter->processNode > 65) { // 65 nm < technology node <= 90 nm
		techHigh.Initialize(90, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 65.0) / 25;
	} else if (inputParameter->processNode > 45) { // 45 nm < technology node <= 65 nm
		techHigh.Initialize(65, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 45.0) / 20;
	} else if (inputParameter->processNode >= 32) { // 32 nm < technology node <= 45 nm
		techHigh.Initialize(45, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 32.0) / 13;
	} else if (inputParameter->processNode >= 22) { // 22 nm < technology node <= 32 nm
		techHigh.Initialize(32, inputParameter->deviceRoadmap);
		alpha = (inputParameter->processNode - 22.0) / 10;
	} else {
		//TO-DO: technology node < 22 nm
	}




	tech->InterpolateWith(techHigh, alpha);
	cell = new MemCell();
	cell->ReadCellFromFile(inputParameter->fileMemCell);

	ofstream outputFile;
	string outputFileName;
	if (inputParameter->optimizationTarget == full_exploration) {
		stringstream temp;
		temp << inputParameter->outputFilePrefix << "_" << inputParameter->capacity / 1024 << "K_" << inputParameter->wordWidth
				<< "_" << inputParameter->associativity;
		if (inputParameter->internalSensing)
			temp << "_IN";
		else
			temp << "_EX";
		if (cell->readMode)
			temp << "_VOL";
		else
			temp << "_CUR";
		temp << ".csv";
		outputFileName = temp.str();
		outputFile.open(outputFileName.c_str(), ofstream::app);
	}


	applyConstraint();
	int numRowMat, numColumnMat, numActiveMatPerRow, numActiveMatPerColumn;
	int numRowSubarray, numColumnSubarray, numActiveSubarrayPerRow, numActiveSubarrayPerColumn;
	int muxSenseAmp, muxOutputLev1, muxOutputLev2, numRowPerSet;
	int areaOptimizationLevel;							/* actually BufferDesignTarget */
	int localWireType, globalWireType;					/* actually WireType */
	int localWireRepeaterType, globalWireRepeaterType;	/* actually WireRepeaterType */
	int isLocalWireLowSwing, isGlobalWireLowSwing;		/* actually boolean value */

	long long capacity;
	long blockSize;
	int associativity;

	/* for cache data array, memory array */

	CAM_Result bestDataResults[(int)full_exploration];
	Bank *dataBank;
	for (int i = 0; i < (int)full_exploration; i++)
		bestDataResults[i].optimizationTarget = (OptimizationTarget)i;

	localWire = new Wire();
	globalWire = new Wire();

	long long numSolution = 0;

	inputParameter->PrintInputParameter();

	/* adjust cache data array parameters according to the access mode */
	capacity = (long long)inputParameter->capacity * 8;
	blockSize = inputParameter->capacity * 8 / inputParameter->wordWidth;
	associativity = 1;

	INITIAL_BASIC_WIRE;
	CAM_BIGFOR {
		if (blockSize / (numActiveMatPerRow * numActiveMatPerColumn * numActiveSubarrayPerRow * numActiveSubarrayPerColumn) == 0) {
			/* To aggressive partitioning */
			continue;
					}
		CALCULATE(dataBank, data);


		if (!dataBank->invalid && !dataBank->mat.subarray.invalid) {
			Result tempResult;
			VERIFY_DATA_CAPACITY;
			numSolution++;
			UPDATE_BEST_DATA;
			if (inputParameter->optimizationTarget == full_exploration && !inputParameter->isPruningEnabled) {
				OUTPUT_TO_FILE;
			}
		}

		delete dataBank;
	}

	if (numSolution > 0) {
		cout << "*** There are " << numSolution << " Solutions. ***" << endl;
		Bank * trialBank;
		Result tempResult;
		/* refine local wire type */
		REFINE_LOCAL_WIRE_FORLOOP {
			localWire->Initialize(inputParameter->processNode, (WireType)localWireType,
					(WireRepeaterType)localWireRepeaterType, inputParameter->temperature,
					(bool)isLocalWireLowSwing);
			for (int i = 0; i < (int)full_exploration; i++) {
				LOAD_GLOBAL_WIRE(bestDataResults[i]);
				CAM_opt.BitSerialWidth = bestDataResults[i].bank->numBitSerial;
				CAM_opt.Proirity = bestDataResults[i].bank->mat.subarray.PriorityOptLevel;
				CAM_opt.RowDriver = bestDataResults[i].bank->mat.subarray.DriverOptLevel;
				TRY_AND_UPDATE(bestDataResults[i], data);
			}
			if (inputParameter->optimizationTarget == full_exploration && !inputParameter->isPruningEnabled) {
				OUTPUT_TO_FILE;
			}
		}
		/* refine global wire type */
		REFINE_GLOBAL_WIRE_FORLOOP {
			globalWire->Initialize(inputParameter->processNode, (WireType)globalWireType,
					(WireRepeaterType)globalWireRepeaterType, inputParameter->temperature,
					(bool)isGlobalWireLowSwing);
			for (int i = 0; i < (int)full_exploration; i++) {
				LOAD_LOCAL_WIRE(bestDataResults[i]);
				CAM_opt.BitSerialWidth = bestDataResults[i].bank->numBitSerial;
				CAM_opt.Proirity = bestDataResults[i].bank->mat.subarray.PriorityOptLevel;
				CAM_opt.RowDriver = bestDataResults[i].bank->mat.subarray.DriverOptLevel;
				TRY_AND_UPDATE(bestDataResults[i], data);
			}
			if (inputParameter->optimizationTarget == full_exploration && !inputParameter->isPruningEnabled) {
				OUTPUT_TO_FILE;
			}
		}
	}

	if (inputParameter->optimizationTarget == full_exploration && inputParameter->isPruningEnabled) {
		/* pruning is enabled */
		Result **** pruningResults;
		/* pruningResults[x][y][z] points to the result which is optimized for x, with constraint on y with z overhead */
		pruningResults = new Result***[(int)full_exploration];	/* full_exploration is always set as the last element in the enum, so if full_exploration is 8, what we want here is a 0-7 array, which is correct */
		for (int i = 0; i < (int)full_exploration; i++) {
			pruningResults[i] = new Result**[(int)full_exploration];
			for (int j = 0; j < (int)full_exploration; j++) {
				pruningResults[i][j] = new Result*[3];		/* 10%, 20%, and 30% overhead */
				for (int k = 0; k < 3; k++)
					pruningResults[i][j][k] = new Result;
			}
		}

		/* assign the constraints */
		for (int i = 0; i < (int)full_exploration; i++)
			for (int j = 0; j < (int)full_exploration; j++)
				for (int k = 0; k < 3; k++) {
					pruningResults[i][j][k]->optimizationTarget = (OptimizationTarget)i;
					*(pruningResults[i][j][k]->localWire) = *(bestDataResults[i].localWire);
					*(pruningResults[i][j][k]->globalWire) = *(bestDataResults[i].globalWire);
					switch ((OptimizationTarget)j) {
					case read_latency_optimized:
						pruningResults[i][j][k]->limitReadLatency = bestDataResults[j].bank->readLatency * (1 + (k + 1.0) / 10);
						break;
					case write_latency_optimized:
						pruningResults[i][j][k]->limitWriteLatency = bestDataResults[j].bank->writeLatency * (1 + (k + 1.0) / 10);
						break;
					case read_energy_optimized:
						pruningResults[i][j][k]->limitReadDynamicEnergy = bestDataResults[j].bank->readDynamicEnergy * (1 + (k + 1.0) / 10);
						break;
					case write_energy_optimized:
						pruningResults[i][j][k]->limitWriteDynamicEnergy = bestDataResults[j].bank->writeDynamicEnergy * (1 + (k + 1.0) / 10);
						break;
					case read_edp_optimized:
						pruningResults[i][j][k]->limitReadEdp = bestDataResults[j].bank->readLatency * bestDataResults[j].bank->readDynamicEnergy * (1 + (k + 1.0) / 10);
						break;
					case write_edp_optimized:
						pruningResults[i][j][k]->limitWriteEdp = bestDataResults[j].bank->writeLatency * bestDataResults[j].bank->writeDynamicEnergy * (1 + (k + 1.0) / 10);
						break;
					case area_optimized:
						pruningResults[i][j][k]->limitArea = bestDataResults[j].bank->area * (1 + (k + 1.0) / 10);
						break;
					case leakage_optimized:
						pruningResults[i][j][k]->limitLeakage = bestDataResults[j].bank->leakage * (1 + (k + 1.0) / 10);
						break;
					default:
						/* nothing should happen here */
						cout << "Warning: should not happen" << endl;
					}
				}

		/* delete */
		for (int i = 0; i < (int)full_exploration; i++) {
			for (int j = 0; j < (int)full_exploration; j++) {
				for (int k = 0; k < 3; k++)
					delete pruningResults[i][j][k];
				delete [] pruningResults[i][j];
			}
			delete [] pruningResults[i];
		}
	}

	/* If design constraint is applied */
	if (inputParameter->optimizationTarget != full_exploration && inputParameter->isConstraintApplied) {
		double allowedDataReadLatency = bestDataResults[read_latency_optimized].bank->readLatency * (inputParameter->readLatencyConstraint + 1);
		double allowedDataWriteLatency = bestDataResults[write_latency_optimized].bank->writeLatency * (inputParameter->writeLatencyConstraint + 1);
		double allowedDataReadDynamicEnergy = bestDataResults[read_energy_optimized].bank->readDynamicEnergy * (inputParameter->readDynamicEnergyConstraint + 1);
		double allowedDataWriteDynamicEnergy = bestDataResults[write_energy_optimized].bank->writeDynamicEnergy * (inputParameter->writeDynamicEnergyConstraint + 1);
		double allowedDataLeakage = bestDataResults[leakage_optimized].bank->leakage * (inputParameter->leakageConstraint + 1);
		double allowedDataArea = bestDataResults[area_optimized].bank->area * (inputParameter->areaConstraint + 1);
		double allowedDataReadEdp = bestDataResults[read_edp_optimized].bank->readLatency
				* bestDataResults[read_edp_optimized].bank->readDynamicEnergy * (inputParameter->readEdpConstraint + 1);
		double allowedDataWriteEdp = bestDataResults[write_edp_optimized].bank->writeLatency
				* bestDataResults[write_edp_optimized].bank->writeDynamicEnergy * (inputParameter->writeEdpConstraint + 1);
		for (int i = 0; i < (int)full_exploration; i++) {
			APPLY_LIMIT(bestDataResults[i]);
		}

		numSolution = 0;
		INITIAL_BASIC_WIRE;
		BIGFOR {
			if (blockSize / (numActiveMatPerRow * numActiveMatPerColumn * numActiveSubarrayPerRow * numActiveSubarrayPerColumn) == 0) {
				/* To aggressive partitioning */
				continue;
			}
			CALCULATE(dataBank, data);
			if (!dataBank->invalid && dataBank->readLatency <= allowedDataReadLatency && dataBank->writeLatency <= allowedDataWriteLatency
					&& dataBank->readDynamicEnergy <= allowedDataReadDynamicEnergy && dataBank->writeDynamicEnergy <= allowedDataWriteDynamicEnergy
					&& dataBank->leakage <= allowedDataLeakage && dataBank->area <= allowedDataArea
					&& dataBank->readLatency * dataBank->readDynamicEnergy <= allowedDataReadEdp && dataBank->writeLatency * dataBank->writeDynamicEnergy <= allowedDataWriteEdp) {
				Result tempResult;
				VERIFY_DATA_CAPACITY;
				numSolution++;
				UPDATE_BEST_DATA;
			}
			delete dataBank;
		}
	}

	if (inputParameter->optimizationTarget != full_exploration) {
		if (numSolution > 0) {

				bestDataResults[inputParameter->optimizationTarget].print();
			
		} else {
			cout << "No valid solutions." << endl;
		}
		cout << endl << "Finished!" << endl;
	} else {
		cout << endl << outputFileName << " generated successfully!" << endl;
		if (inputParameter->isPruningEnabled) {
			cout << "The results are pruned" << endl;
		}

		for (int i = 0; i < (int)full_exploration; i++) {
			cout << "[" << left << setw(2) << i << "]" << " ";
			cout << left << setw(8) << bestDataResults[i].bank->readLatency * 1e12 << "    ";
		    cout << left << setw(8) << bestDataResults[i].bank->writeLatency * 1e12 << "    ";
		    cout << left << setw(5) << bestDataResults[i].bank->readDynamicEnergy * 1e12 << "    ";
		    cout << left << setw(5) << bestDataResults[i].bank->writeDynamicEnergy * 1e12 << "    ";
			cout << left << setw(9) << bestDataResults[i].bank->readLatency * bestDataResults[i].bank->readDynamicEnergy * 1e24 << "    ";
		    cout << left << setw(9) << bestDataResults[i].bank->writeLatency * bestDataResults[i].bank->writeDynamicEnergy * 1e24 << "    ";
		    cout << left << setw(5) << bestDataResults[i].bank->leakage * 1e6 << "    ";
			cout << left << setw(8) << bestDataResults[i].bank->area * 1e12 << "    ";
		    cout << left << setw(8) << bestDataResults[i].bank->searchLatency * 1e12 << "    ";
		    cout << left << setw(5) << bestDataResults[i].bank->searchDynamicEnergy * 1e12 << "    ";
		    cout << left << setw(8) << bestDataResults[i].bank->searchDynamicEnergy * bestDataResults[i].bank->searchLatency * 1e24 << endl;
		}
		cout 	<< left << setw(8) << bestDataResults[7].bank->numBitSerial << "	"
				<< left << setw(8) << bestDataResults[8].bank->numBitSerial << "	"
				<< left << setw(8) << bestDataResults[9].bank->numBitSerial << "	"
				<< left << setw(8) << bestDataResults[3].bank->numBitSerial << "	"
				<< left << setw(8) << bestDataResults[6].bank->numBitSerial << "	"
				<< endl;
		cout 	<< left << setw(8) << bestDataResults[7].bank->mat.areaOptimizationLevel << "	"
				<< left << setw(8) << bestDataResults[8].bank->mat.areaOptimizationLevel << "	"
				<< left << setw(8) << bestDataResults[9].bank->mat.areaOptimizationLevel << "	"
				<< left << setw(8) << bestDataResults[3].bank->mat.areaOptimizationLevel << "	"
				<< left << setw(8) << bestDataResults[6].bank->mat.areaOptimizationLevel << "	"
				<< endl;
		cout 	<< left << setw(8) << bestDataResults[7].bank->mat.subarray.DriverOptLevel << "	"
				<< left << setw(8) << bestDataResults[8].bank->mat.subarray.DriverOptLevel << "	"
				<< left << setw(8) << bestDataResults[9].bank->mat.subarray.DriverOptLevel << "	"
				<< left << setw(8) << bestDataResults[3].bank->mat.subarray.DriverOptLevel << "	"
				<< left << setw(8) << bestDataResults[6].bank->mat.subarray.DriverOptLevel << "	"
				<< endl;
	    cout 	<< left << setw(8) << bestDataResults[7].bank->area * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[8].bank->area * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[9].bank->area * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[3].bank->area * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[6].bank->area * 1e12 << "	"
	    		<< endl;
	    cout 	<< left << setw(8) << bestDataResults[7].bank->searchLatency * 1e9 << "	"
	    	 	<< left << setw(8) << bestDataResults[8].bank->searchLatency * 1e9 << "	"
	    	 	<< left << setw(8) << bestDataResults[9].bank->searchLatency * 1e9 << "	"
	    	 	<< left << setw(8) << bestDataResults[3].bank->searchLatency * 1e9 << "	"
	    	 	<< left << setw(8) << bestDataResults[6].bank->searchLatency * 1e9 << "	"
	    		<< endl;
	    cout 	<< left << setw(8) << bestDataResults[7].bank->searchDynamicEnergy * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[8].bank->searchDynamicEnergy * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[9].bank->searchDynamicEnergy * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[3].bank->searchDynamicEnergy * 1e12 << "	"
	    	 	<< left << setw(8) << bestDataResults[6].bank->searchDynamicEnergy * 1e12 << "	"
	    		<< endl;
	    cout	<< left << setw(8) << bestDataResults[7].bank->writeDynamicEnergy * 1e12 << "	"
	    		<< left << setw(8) << bestDataResults[8].bank->writeDynamicEnergy * 1e12 << "	"
	    		<< left << setw(8) << bestDataResults[9].bank->writeDynamicEnergy * 1e12 << "	"
	    		<< left << setw(8) << bestDataResults[3].bank->writeDynamicEnergy * 1e12 << "	"
	    		<< left << setw(8) << bestDataResults[6].bank->writeDynamicEnergy * 1e12 << "	"
	    		<< endl;
	    cout	<< left << setw(8) << bestDataResults[7].bank->leakage * 1e6 << "	"
	    		<< left << setw(8) << bestDataResults[8].bank->leakage * 1e6 << "	"
	    		<< left << setw(8) << bestDataResults[9].bank->leakage * 1e6 << "	"
	    		<< left << setw(8) << bestDataResults[3].bank->leakage * 1e6 << "	"
	    		<< left << setw(8) << bestDataResults[6].bank->leakage * 1e6 << "	"
	    		<< endl;
	}

	if (outputFile.is_open())
		outputFile.close();
	if (localWire) delete localWire;
	if (globalWire) delete globalWire;
	return 0;
}

void applyConstraint() {
	/* Check functions that are not yet implemented */
	if (cell->memCellType == DRAM) {
		cout << "[ERROR] DRAM model is still under development" << endl;
		exit(-1);
	}
	if (cell->memCellType == eDRAM) {
		cout << "[Warning] Embedded DRAM model is still under development" << endl;
		//exit(-1);
	}
	if (cell->memCellType == MLCNAND) {
		cout << "[ERROR] MLC NAND flash model is still under development" << endl;
		exit(-1);
	}

	if (inputParameter->designTarget != cache && inputParameter->associativity > 1) {
		cout << "[WARNING] Associativity setting is ignored for non-cache designs" << endl;
		inputParameter->associativity = 1;
	}

	if (!isPow2(inputParameter->associativity)) {
		cout << "[ERROR] The associativity value has to be a power of 2 in this version" << endl;
		exit(-1);
	}

	if (inputParameter->routingMode == h_tree && inputParameter->internalSensing == false) {
		cout << "[ERROR] H-tree does not support external sensing scheme in this version" << endl;
		exit(-1);
	}
/*
	if (inputParameter->globalWireRepeaterType != repeated_none && inputParameter->internalSensing == false) {
		cout << "[ERROR] Repeated global wire does not support external sensing scheme" << endl;
		exit(-1);
	}
*/

	/* TO-DO: more rules to add here */
}




void mat_only_print(Mat mat){
	// all in um, ps, uw

	///////////////       area              //////////////
	cout << mat.height * 1e6 << endl << mat.width * 1e6 << endl;
	cout << mat.area * 1e12 << endl;
	cout << (mat.rowPredecoderBlock1.area + mat.rowPredecoderBlock2.area) * 1e12 << endl;
	cout << (mat.bitlineMuxPredecoderBlock1.area + mat.bitlineMuxPredecoderBlock2.area +
			mat.senseAmpMuxLev1PredecoderBlock1.area + mat.senseAmpMuxLev1PredecoderBlock2.area +
			mat.senseAmpMuxLev2PredecoderBlock1.area + mat.senseAmpMuxLev2PredecoderBlock2.area ) * 1e12 << endl;
	cout << mat.subarray.area * 1e12 << endl;
	cout << mat.subarray.inputEnc.area *1e12 << endl;
	cout << (mat.subarray.RowDecMergeNand.area +mat.subarray.senseAmpMuxLev1Nand.area +mat.subarray.senseAmpMuxLev2Nand.area) * 1e12 << endl;
	double area = 0;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		area += mat.subarray.RowDriver[i].area;
	}
	cout << area * 1e12 << endl;
	cout << mat.subarray.precharger.area * 1e12 << endl;
	cout << mat.subarray.lenRow * mat.subarray.lenCol * 1e12 << endl;
	area = 0;
	for(int i=0;i<CAM_cell->camNumCol;i++){
		area += mat.subarray.ColMux[i].area;
	}
	cout << area * 1e12 << endl;
	cout << mat.subarray.senseAmp.area * 1e12 << endl;
	cout << (mat.subarray.senseAmpMuxLev1.area + mat.subarray.senseAmpMuxLev2.area) * 1e12 << endl;
	cout << mat.subarray.outputAcc.area * 1e12 << endl;
	cout << mat.subarray.priorityEnc.area * 1e12 << endl;

	///////////////       latency              //////////////
	cout << mat.readLatency * 1e12 << endl;
	cout << (mat.rowPredecoderBlock1.readLatency + mat.rowPredecoderBlock2.readLatency) * 1e12 << endl;
	cout << (mat.bitlineMuxPredecoderBlock1.readLatency + mat.bitlineMuxPredecoderBlock2.readLatency +
			mat.senseAmpMuxLev1PredecoderBlock1.readLatency + mat.senseAmpMuxLev1PredecoderBlock2.readLatency +
			mat.senseAmpMuxLev2PredecoderBlock1.readLatency + mat.senseAmpMuxLev2PredecoderBlock2.readLatency ) * 1e12 << endl;
	cout << mat.subarray.readLatency * 1e12 << endl;
	cout << mat.subarray.inputEnc.readLatency *1e12 << endl;
	cout << (mat.subarray.RowDecMergeNand.readLatency +mat.subarray.senseAmpMuxLev1Nand.readLatency +mat.subarray.senseAmpMuxLev2Nand.readLatency) * 1e12 << endl;
	double readLatency = 0;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		readLatency = MAX(mat.subarray.RowDriver[i].readLatency, readLatency);
	}
	cout << readLatency * 1e12 << endl;
	cout << mat.subarray.precharger.readLatency * 1e12 << endl;
	cout << mat.subarray.bitlineDelay * 1e12 << endl;
	readLatency = 0;
	for(int i=0;i<CAM_cell->camNumCol;i++){
		readLatency = MAX(mat.subarray.ColMux[i].readLatency, readLatency);
	}
	cout << readLatency * 1e12 << endl;
	cout << mat.subarray.senseAmp.readLatency * 1e12 << endl;
	cout << (mat.subarray.senseAmpMuxLev1.readLatency + mat.subarray.senseAmpMuxLev2.readLatency) * 1e12 << endl;
	cout << mat.subarray.outputAcc.readLatency * 1e12 << endl;
	cout << mat.subarray.priorityEnc.readLatency * 1e12 << endl;

	///////////////       energy              //////////////
	cout << mat.readDynamicEnergy * 1e12 << endl;
	cout << (mat.rowPredecoderBlock1.readDynamicEnergy + mat.rowPredecoderBlock2.readDynamicEnergy) * 1e12 << endl;
	cout << (mat.bitlineMuxPredecoderBlock1.readDynamicEnergy + mat.bitlineMuxPredecoderBlock2.readDynamicEnergy +
			mat.senseAmpMuxLev1PredecoderBlock1.readDynamicEnergy + mat.senseAmpMuxLev1PredecoderBlock2.readDynamicEnergy +
			mat.senseAmpMuxLev2PredecoderBlock1.readDynamicEnergy + mat.senseAmpMuxLev2PredecoderBlock2.readDynamicEnergy ) * 1e12 << endl;
	cout << mat.subarray.readDynamicEnergy * 1e12 << endl;
	cout << mat.subarray.inputEnc.readDynamicEnergy *1e12 << endl;
	cout << (mat.subarray.RowDecMergeNand.readDynamicEnergy +mat.subarray.senseAmpMuxLev1Nand.readDynamicEnergy
			+mat.subarray.senseAmpMuxLev2Nand.readDynamicEnergy) * 1e12 << endl;
	double readDynamicEnergy = 0;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		readDynamicEnergy += mat.subarray.RowDriver[i].readDynamicEnergy;
	}
	cout << readDynamicEnergy * 1e12 << endl;
	cout << mat.subarray.precharger.readDynamicEnergy * 1e12 << endl;
	// TODO: not really the breakdown
	cout << mat.subarray.cellReadEnergy * 1e12 << endl;
	readDynamicEnergy = 0;
	for(int i=0;i<CAM_cell->camNumCol;i++){
		readDynamicEnergy += mat.subarray.ColMux[i].readDynamicEnergy;
	}
	cout << readDynamicEnergy * 1e12 << endl;
	cout << mat.subarray.senseAmp.readDynamicEnergy * 1e12 << endl;
	cout << (mat.subarray.senseAmpMuxLev1.readDynamicEnergy + mat.subarray.senseAmpMuxLev2.readDynamicEnergy) * 1e12 << endl;
	cout << mat.subarray.outputAcc.readDynamicEnergy * 1e12 << endl;
	cout << mat.subarray.priorityEnc.readDynamicEnergy * 1e12 << endl;

	///////////////       leakage              //////////////
	cout << mat.leakage * 1e6 << endl;
	cout << (mat.rowPredecoderBlock1.leakage + mat.rowPredecoderBlock2.leakage) * 1e6 << endl;
	cout << (mat.bitlineMuxPredecoderBlock1.leakage + mat.bitlineMuxPredecoderBlock2.leakage +
			mat.senseAmpMuxLev1PredecoderBlock1.leakage + mat.senseAmpMuxLev1PredecoderBlock2.leakage +
			mat.senseAmpMuxLev2PredecoderBlock1.leakage + mat.senseAmpMuxLev2PredecoderBlock2.leakage ) * 1e6 << endl;
	cout << mat.subarray.leakage * 1e6 << endl;
	cout << mat.subarray.inputEnc.leakage *1e6 << endl;
	cout << (mat.subarray.RowDecMergeNand.leakage +mat.subarray.senseAmpMuxLev1Nand.leakage +mat.subarray.senseAmpMuxLev2Nand.leakage) * 1e6 << endl;
	double leakage = 0;
	for(int i=0;i<CAM_cell->camNumRow;i++){
		leakage += mat.subarray.RowDriver[i].leakage;
	}
	cout << leakage * 1e6 << endl;
	cout << mat.subarray.precharger.leakage * 1e6 << endl;
	cout << 0 << endl;
	leakage = 0;
	for(int i=0;i<CAM_cell->camNumCol;i++){
		leakage += mat.subarray.ColMux[i].leakage;
	}
	cout << leakage * 1e6 << endl;
	cout << mat.subarray.senseAmp.leakage * 1e6 << endl;
	cout << (mat.subarray.senseAmpMuxLev1.leakage + mat.subarray.senseAmpMuxLev2.leakage) * 1e6 << endl;
	cout << mat.subarray.outputAcc.leakage * 1e6 << endl;
	cout << mat.subarray.priorityEnc.leakage * 1e6 << endl;
}

