#include "CAM_Result.h"
#include "Result.h"
#include "global.h"
#include "formula.h"
#include "macros.h"

#include <iostream>
#include <fstream>

using namespace std;


CAM_Result::CAM_Result() {
	// TODO Auto-generated constructor stub
	if (inputParameter->routingMode == h_tree)
		bank = new BankWithHtree();
	else
		bank = new BankWithoutHtree();
	localWire = new Wire();
	globalWire = new Wire();

	/* initialize the worst case */
	bank->readLatency = 1e41;
	bank->writeLatency = 1e41;
	bank->readDynamicEnergy = 1e41;
	bank->writeDynamicEnergy = 1e41;
	bank->leakage = 1e41;
	bank->height = 1e41;
	bank->width = 1e41;
	bank->area = 1e41;
	bank->searchLatency = 1e41;
	bank->searchDynamicEnergy = 1e41;

	/* No constraints */
	limitReadLatency = 1e41;
	limitWriteLatency = 1e41;
	limitReadDynamicEnergy = 1e41;
	limitWriteDynamicEnergy = 1e41;
	limitReadEdp = 1e41;
	limitWriteEdp = 1e41;
	limitArea = 1e41;
	limitLeakage = 1e41;

	/* Default read latency optimization */
	optimizationTarget = read_latency_optimized;
}

CAM_Result::~CAM_Result() {
	// TODO Auto-generated destructor stub
}

void CAM_Result::reset() {
	bank->readLatency = 1e41;
	bank->writeLatency = 1e41;
	bank->readDynamicEnergy = 1e41;
	bank->writeDynamicEnergy = 1e41;
	bank->leakage = 1e41;
	bank->height = 1e41;
	bank->width = 1e41;
	bank->area = 1e41;
	bank->searchLatency = 1e41;
	bank->searchDynamicEnergy = 1e41;
}

void CAM_Result::compareAndUpdate(Result &newResult) {
	if (newResult.bank->readLatency <= limitReadLatency && newResult.bank->writeLatency <= limitWriteLatency
			&& newResult.bank->readDynamicEnergy <= limitReadDynamicEnergy && newResult.bank->writeDynamicEnergy <= limitWriteDynamicEnergy
			&& newResult.bank->readLatency * newResult.bank->readDynamicEnergy <= limitReadEdp
			&& newResult.bank->writeLatency * newResult.bank->writeDynamicEnergy <= limitWriteEdp
			&& newResult.bank->area <= limitArea && newResult.bank->leakage <= limitLeakage) {
		bool toUpdate = false;
		switch (optimizationTarget) {
		case read_latency_optimized:
			if 	(newResult.bank->readLatency < bank->readLatency)
				toUpdate = true;
			break;
		case write_latency_optimized:
			if 	(newResult.bank->writeLatency < bank->writeLatency)
				toUpdate = true;
			break;
		case read_energy_optimized:
			if 	(newResult.bank->readDynamicEnergy < bank->readDynamicEnergy)
				toUpdate = true;
			break;
		case write_energy_optimized:
			if 	(newResult.bank->writeDynamicEnergy < bank->writeDynamicEnergy)
				toUpdate = true;
			break;
		case read_edp_optimized:
			if 	(newResult.bank->readLatency * newResult.bank->readDynamicEnergy < bank->readLatency * bank->readDynamicEnergy)
				toUpdate = true;
			break;
		case write_edp_optimized:
			if 	(newResult.bank->writeLatency * newResult.bank->writeDynamicEnergy < bank->writeLatency * bank->writeDynamicEnergy)
				toUpdate = true;
			break;
		case area_optimized:
			if 	(newResult.bank->area < bank->area)
				toUpdate = true;
			break;
		case leakage_optimized:
			if 	(newResult.bank->leakage < bank->leakage)
				toUpdate = true;
			break;
		case search_latency_optimized:
			if 	(newResult.bank->searchLatency < bank->searchLatency)
				toUpdate = true;
			break;
		case search_energy_optimized:
			if 	(newResult.bank->searchDynamicEnergy < bank->searchDynamicEnergy)
				toUpdate = true;
			break;
		case search_edp_optimized:
			if 	(newResult.bank->searchDynamicEnergy * newResult.bank->searchLatency < bank->searchDynamicEnergy * bank->searchLatency)
				toUpdate = true;
			break;
		default:	/* Exploration */
			/* should not happen */
			;
		}
		if (toUpdate) {
			*bank = *(newResult.bank);
			*localWire = *(newResult.localWire);
			*globalWire = *(newResult.globalWire);
		}
	}
}

void CAM_Result::print() {
    // cout << "Bank Area: " << bank->area * 1e12 << endl;
    // cout << "Bank Search Latency: " << bank->searchLatency * 1e9 << endl;
    // cout << "Bank Search Dynamic Energy: " << bank->searchDynamicEnergy * 1e12 << endl;
    // cout << "Bank Write Dynamic Energy: " << bank->writeDynamicEnergy * 1e12 << endl;
    // cout << "Bank Leakage: " << bank->leakage * 1e6 << endl;
	// cout << "==============================" <<endl;
	cout << endl << "=============" << endl << "CONFIGURATION" << endl << "=============" << endl;
	cout << "Bank Organization: " << bank->numRowMat << " x " << bank->numColumnMat << endl;
	cout << " - Row Activation   : " << bank->numActiveMatPerColumn << " / " << bank->numRowMat << endl;
	cout << " - Column Activation: " << bank->numActiveMatPerRow << " / " << bank->numColumnMat << endl;
	cout << "Mat Organization: " << bank->numRowSubarray << " x " << bank->numColumnSubarray << endl;
	cout << " - Row Activation   : " << bank->numActiveSubarrayPerColumn << " / " << bank->numRowSubarray << endl;
	cout << " - Column Activation: " << bank->numActiveSubarrayPerRow << " / " << bank->numColumnSubarray << endl;
	cout << " - Subarray Size    : " << bank->mat.subarray.numRow << " Rows x " << bank->mat.subarray.numColumn << " Columns" << endl;
	cout << "Mux Level:" << endl;
	cout << " - Senseamp Mux      : " << bank->muxSenseAmp << endl;
	cout << " - Output Level-1 Mux: " << bank->muxOutputLev1 << endl;
	cout << " - Output Level-2 Mux: " << bank->muxOutputLev2 << endl;
	cout << "Local Wire:" << endl;
	cout << " - Wire Type : ";
	switch (localWire->wireType) {
	case local_aggressive:
		cout << "Local Aggressive" << endl;
		break;
	case local_conservative:
		cout << "Local Conservative" << endl;
		break;
	case semi_aggressive:
		cout << "Semi-Global Aggressive" << endl;
		break;
	case semi_conservative:
		cout << "Semi-Global Conservative" << endl;
		break;
	case global_aggressive:
		cout << "Global Aggressive" << endl;
		break;
	case global_conservative:
		cout << "Global Conservative" << endl;
		break;
	default:
		cout << "DRAM Wire" << endl;
	}
	cout << " - Repeater Type: ";
	switch (localWire->wireRepeaterType) {
	case repeated_none:
		cout << "No Repeaters" << endl;
		break;
	case repeated_opt:
		cout << "Fully-Optimized Repeaters" << endl;
		break;
	case repeated_5:
		cout << "Repeaters with 5% Overhead" << endl;
		break;
	case repeated_10:
		cout << "Repeaters with 10% Overhead" << endl;
		break;
	case repeated_20:
		cout << "Repeaters with 20% Overhead" << endl;
		break;
	case repeated_30:
		cout << "Repeaters with 30% Overhead" << endl;
		break;
	case repeated_40:
		cout << "Repeaters with 40% Overhead" << endl;
		break;
	case repeated_50:
		cout << "Repeaters with 50% Overhead" << endl;
		break;
	default:
		cout << "Unknown" << endl;
	}
	cout << " - Low Swing : ";
	if (localWire->isLowSwing)
		cout << "Yes" << endl;
	else
		cout << "No" << endl;
	cout << "Global Wire:" << endl;
	cout << " - Wire Type : ";
	switch (globalWire->wireType) {
	case local_aggressive:
		cout << "Local Aggressive" << endl;
		break;
	case local_conservative:
		cout << "Local Conservative" << endl;
		break;
	case semi_aggressive:
		cout << "Semi-Global Aggressive" << endl;
		break;
	case semi_conservative:
		cout << "Semi-Global Conservative" << endl;
		break;
	case global_aggressive:
		cout << "Global Aggressive" << endl;
		break;
	case global_conservative:
		cout << "Global Conservative" << endl;
		break;
	default:
		cout << "DRAM Wire" << endl;
	}
	cout << " - Repeater Type: ";
	switch (globalWire->wireRepeaterType) {
	case repeated_none:
		cout << "No Repeaters" << endl;
		break;
	case repeated_opt:
		cout << "Fully-Optimized Repeaters" << endl;
		break;
	case repeated_5:
		cout << "Repeaters with 5% Overhead" << endl;
		break;
	case repeated_10:
		cout << "Repeaters with 10% Overhead" << endl;
		break;
	case repeated_20:
		cout << "Repeaters with 20% Overhead" << endl;
		break;
	case repeated_30:
		cout << "Repeaters with 30% Overhead" << endl;
		break;
	case repeated_40:
		cout << "Repeaters with 40% Overhead" << endl;
		break;
	case repeated_50:
		cout << "Repeaters with 50% Overhead" << endl;
		break;
	default:
		cout << "Unknown" << endl;
	}
	cout << " - Low Swing : ";
	if (globalWire->isLowSwing)
		cout << "Yes" << endl;
	else
		cout << "No" << endl;
	cout << "Buffer Design Style: ";
	switch (bank->areaOptimizationLevel) {
	case latency_first:
		cout << "Latency-Optimized" << endl;
		break;
	case area_first:
		cout << "Area-Optimized" << endl;
		break;
	default:	/* balance */
		cout << "Balanced" << endl;
	}

	cout << "==============================================" << endl << "               SUMMARY RESULT" << endl << "==============================================" << endl;

	cout << "Area:" << endl;

	cout << " - Total Area = " << TO_METER(bank->height) << " x " << TO_METER(bank->width)
			<< " = " << TO_SQM(bank->area) << endl;
	cout << " |--- Mat Area      = " << TO_METER(bank->mat.height) << " x " << TO_METER(bank->mat.width)
			<< " = " << TO_SQM(bank->mat.area) << "   (" << cell->area * tech->featureSize * tech->featureSize
			* bank->capacity / bank->numRowMat / bank->numColumnMat / bank->mat.area * 100 << "%)" << endl;
	cout << " |--- Subarray Area = " << TO_METER(bank->mat.subarray.height) << " x "
			<< TO_METER(bank->mat.subarray.width) << " = " << TO_SQM(bank->mat.subarray.area) << "   ("
			<< cell->area * tech->featureSize * tech->featureSize * bank->capacity / bank->numRowMat
			/ bank->numColumnMat / bank->numRowSubarray / bank->numColumnSubarray
			/ bank->mat.subarray.area * 100 << "%)" <<endl;
	cout << " - Area Efficiency = " << cell->area * tech->featureSize * tech->featureSize
			* bank->capacity / bank->area * 100 << "%" << endl;
	cout << "Timing:" << endl;

	cout << " -  Search Latency = " << TO_SECOND(bank->readLatency) << endl;
	if (inputParameter->routingMode == h_tree)
		cout << " |--- H-Tree Latency = " << TO_SECOND(bank->readLatency - bank->mat.readLatency) << endl;
	else
		cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->readLatency - bank->mat.readLatency) << endl;
	cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.readLatency) << endl;
	cout << "    |--- Predecoder Latency = " << TO_SECOND(bank->mat.predecoderLatency) << endl;
	// cout << "       |--- Row Decoder Latency = " << TO_SECOND(bank->mat.subarray.rowDecoder.readLatency) << endl;
	// cout << "       |--- Bitline Latency     = " << TO_SECOND(bank->mat.subarray.bitlineDelay) << endl;
	// if (inputParameter->internalSensing)
	// 	cout << "       |--- Senseamp Latency    = " << TO_SECOND(bank->mat.subarray.senseAmp.readLatency) << endl;
	// cout << "       |--- Mux Latency         = " << TO_SECOND(bank->mat.subarray.bitlineMux.readLatency
	// 												+ bank->mat.subarray.senseAmpMuxLev1.readLatency
	// 												+ bank->mat.subarray.senseAmpMuxLev2.readLatency) << endl;
	// cout << "       |--- Precharge Latency   = " << TO_SECOND(bank->mat.subarray.precharger.readLatency) << endl;

	if (cell->memCellType == PCRAM || cell->memCellType == FBRAM || cell->memCellType == FEFETRAM ||
			(cell->memCellType == memristor && (cell->accessType == CMOS_access || cell->accessType == BJT_access))) {
		cout << " - RESET Latency = " << TO_SECOND(bank->resetLatency) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Latency = " << TO_SECOND(bank->resetLatency - bank->mat.resetLatency) << endl;
		else
			cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->resetLatency - bank->mat.resetLatency) << endl;
		cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.resetLatency) << endl;
		cout << "    |--- Predecoder Latency = " << TO_SECOND(bank->mat.predecoderLatency) << endl;
		cout << "    |--- Subarray Latency   = " << TO_SECOND(bank->mat.subarray.resetLatency) << endl;
		// cout << "       |--- RESET Pulse Duration = " << TO_SECOND(cell->resetPulse) << endl;
		// cout << "       |--- Row Decoder Latency  = " << TO_SECOND(bank->mat.subarray.rowDecoder.writeLatency) << endl;
		// cout << "       |--- Charge Latency   = " << TO_SECOND(bank->mat.subarray.chargeLatency) << endl;
		// cout << " - SET Latency   = " << TO_SECOND(bank->setLatency) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Latency = " << TO_SECOND(bank->setLatency - bank->mat.setLatency) << endl;
		else
			cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->setLatency - bank->mat.setLatency) << endl;
		cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.setLatency) << endl;
		cout << "    |--- Predecoder Latency = " << TO_SECOND(bank->mat.predecoderLatency) << endl;
		cout << "    |--- Subarray Latency   = " << TO_SECOND(bank->mat.subarray.setLatency) << endl;
		// cout << "       |--- SET Pulse Duration   = " << TO_SECOND(cell->setPulse) << endl;
		// cout << "       |--- Row Decoder Latency  = " << TO_SECOND(bank->mat.subarray.rowDecoder.writeLatency) << endl;
		// cout << "       |--- Charger Latency      = " << TO_SECOND(bank->mat.subarray.chargeLatency) << endl;
	} else if (cell->memCellType == SLCNAND) {
		cout << " - Erase Latency = " << TO_SECOND(bank->resetLatency) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Latency = " << TO_SECOND(bank->resetLatency - bank->mat.resetLatency) << endl;
		else
			cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->resetLatency - bank->mat.resetLatency) << endl;
		cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.resetLatency) << endl;
		cout << " - Programming Latency   = " << TO_SECOND(bank->setLatency) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Latency = " << TO_SECOND(bank->setLatency - bank->mat.setLatency) << endl;
		else
			cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->setLatency - bank->mat.setLatency) << endl;
		cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.setLatency) << endl;
	} else {
		cout << " - Write Latency = " << TO_SECOND(bank->writeLatency) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Latency = " << TO_SECOND(bank->writeLatency - bank->mat.writeLatency) << endl;
		else
			cout << " |--- Non-H-Tree Latency = " << TO_SECOND(bank->writeLatency - bank->mat.writeLatency) << endl;
		cout << " |--- Mat Latency    = " << TO_SECOND(bank->mat.writeLatency) << endl;
		cout << "    |--- Predecoder Latency = " << TO_SECOND(bank->mat.predecoderLatency) << endl;
		cout << "    |--- Subarray Latency   = " << TO_SECOND(bank->mat.subarray.writeLatency) << endl;
		// if (cell->memCellType == MRAM)
		// 	cout << "       |--- Write Pulse Duration = " << TO_SECOND(cell->resetPulse) << endl;	// MRAM reset/set is equal
		// cout << "       |--- Row Decoder Latency = " << TO_SECOND(bank->mat.subarray.rowDecoder.writeLatency) << endl;
		// cout << "       |--- Charge Latency      = " << TO_SECOND(bank->mat.subarray.chargeLatency) << endl;
	}

	double readBandwidth = (double)bank->blockSize /
			(bank->mat.subarray.readLatency - bank->mat.subarray.rowDecoder.readLatency
			+ bank->mat.subarray.precharger.readLatency) / 8;
	cout << " - Read Bandwidth  = " << TO_BPS(readBandwidth) << endl;
	double writeBandwidth = (double)bank->blockSize /
			(bank->mat.subarray.writeLatency) / 8;
	cout << " - Write Bandwidth = " << TO_BPS(writeBandwidth) << endl;

	cout << "Power:" << endl;

	cout << " -  Read Dynamic Energy = " << TO_JOULE(bank->readDynamicEnergy) << endl;
	if (inputParameter->routingMode == h_tree)
		cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->readDynamicEnergy - bank->mat.readDynamicEnergy
													* bank->numActiveMatPerColumn * bank->numActiveMatPerRow) 
													<< endl;
	else
		cout << " |--- Non-H-Tree Dynamic Energy = " << TO_JOULE(bank->readDynamicEnergy - bank->mat.readDynamicEnergy
													* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
													<< endl;
	cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.readDynamicEnergy) << " per mat" << endl;
	cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.readDynamicEnergy - bank->mat.subarray.readDynamicEnergy
														* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
														<< endl;
	cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.readDynamicEnergy) << " per active subarray" << endl;
	// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.readDynamicEnergy) << endl;
	// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.readDynamicEnergy
	// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.readDynamicEnergy
	// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.readDynamicEnergy) << endl;
	// if (cell->memCellType == PCRAM || cell->memCellType == FBRAM || cell->memCellType == MRAM || cell->memCellType == memristor || cell->memCellType == FEFETRAM	 ) {
	// 	cout << "       |--- Bitline & Cell Read Energy = " << TO_JOULE(bank->mat.subarray.cellReadEnergy) << endl;
	// }
	// if (inputParameter->internalSensing)
	// 	cout << "       |--- Senseamp Dynamic Energy    = " << TO_JOULE(bank->mat.subarray.senseAmp.readDynamicEnergy) << endl;
	// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.readDynamicEnergy
	// 												+ bank->mat.subarray.senseAmpMuxLev1.readDynamicEnergy
	// 												+ bank->mat.subarray.senseAmpMuxLev2.readDynamicEnergy) << endl;
	// cout << "       |--- Precharge Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.precharger.readDynamicEnergy) << endl;

	if (cell->memCellType == PCRAM || cell->memCellType == FBRAM ||
			(cell->memCellType == memristor && (cell->accessType == CMOS_access || cell->accessType == BJT_access || cell->memCellType == FEFETRAM))) {
		cout << " - RESET Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy - bank->mat.resetDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		else
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy - bank->mat.resetDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.resetDynamicEnergy) << " per mat" << endl;
		cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.writeDynamicEnergy - bank->mat.subarray.writeDynamicEnergy
															* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
															<< endl;
		cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.writeDynamicEnergy) << " per active subarray" << endl;
		// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
		// cout << "       |--- Cell RESET Dynamic Energy  = " << TO_JOULE(bank->mat.subarray.cellResetEnergy) << endl;
		cout << " - SET Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy - bank->mat.setDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		else
			cout << " |--- Non-H-Tree Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy - bank->mat.setDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.setDynamicEnergy) << " per mat" << endl;
		cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.writeDynamicEnergy - bank->mat.subarray.writeDynamicEnergy
															* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
															<< endl;
		cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.writeDynamicEnergy) << " per active subarray" << endl;
		// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
		// cout << "       |--- Cell SET Dynamic Energy    = " << TO_JOULE(bank->mat.subarray.cellSetEnergy) << endl;
	} else if (cell->memCellType == SLCNAND) {
		cout << " - Erase Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy) << " per block" << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy - bank->mat.resetDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		else
			cout << " |--- Non-H-Tree Dynamic Energy = " << TO_JOULE(bank->resetDynamicEnergy - bank->mat.resetDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.resetDynamicEnergy) << " per mat" << endl;
		cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.writeDynamicEnergy - bank->mat.subarray.writeDynamicEnergy
															* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
															<< endl;
		cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.writeDynamicEnergy) << " per active subarray" << endl;
		// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
		// cout << " - Programming Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy) << " per page" << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy - bank->mat.setDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		else
			cout << " |--- Non-H-Tree Dynamic Energy = " << TO_JOULE(bank->setDynamicEnergy - bank->mat.setDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.setDynamicEnergy) << " per mat" << endl;
		cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.writeDynamicEnergy - bank->mat.subarray.writeDynamicEnergy
															* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
															<< endl;
		cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.writeDynamicEnergy) << " per active subarray" << endl;
		// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
	} else {
		cout << " - Write Dynamic Energy = " << TO_JOULE(bank->writeDynamicEnergy) << endl;
		if (inputParameter->routingMode == h_tree)
			cout << " |--- H-Tree Dynamic Energy = " << TO_JOULE(bank->writeDynamicEnergy - bank->mat.writeDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		else
			cout << " |--- Non-H-Tree Dynamic Energy = " << TO_JOULE(bank->writeDynamicEnergy - bank->mat.writeDynamicEnergy
														* bank->numActiveMatPerColumn * bank->numActiveMatPerRow)
														<< endl;
		cout << " |--- Mat Dynamic Energy    = " << TO_JOULE(bank->mat.writeDynamicEnergy) << " per mat" << endl;
		cout << "    |--- Predecoder Dynamic Energy = " << TO_JOULE(bank->mat.writeDynamicEnergy - bank->mat.subarray.writeDynamicEnergy
															* bank->numActiveSubarrayPerRow * bank->numActiveSubarrayPerColumn)
															<< endl;
		cout << "    |--- Subarray Dynamic Energy   = " << TO_JOULE(bank->mat.subarray.writeDynamicEnergy) << " per active subarray" << endl;
		// cout << "       |--- Row Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.rowDecoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Decoder Dynamic Energy = " << TO_JOULE(bank->mat.subarray.bitlineMuxDecoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1Decoder.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2Decoder.writeDynamicEnergy) << endl;
		// cout << "       |--- Mux Dynamic Energy         = " << TO_JOULE(bank->mat.subarray.bitlineMux.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev1.writeDynamicEnergy
		// 												+ bank->mat.subarray.senseAmpMuxLev2.writeDynamicEnergy) << endl;
		// if (cell->memCellType == MRAM) {
		// 	cout << "       |--- Bitline & Cell Write Energy= " << TO_JOULE(bank->mat.subarray.cellResetEnergy) << endl;
		// }
	}

	cout << " - Leakage Power = " << TO_WATT(bank->leakage) << endl;
	if (inputParameter->routingMode == h_tree)
		cout << " |--- H-Tree Leakage Power = " << TO_WATT(bank->leakage - bank->mat.leakage
													* bank->numColumnMat * bank->numRowMat)
													<< endl;
	else
		cout << " |--- Non-H-Tree Leakage Power = " << TO_WATT(bank->leakage - bank->mat.leakage
													* bank->numColumnMat * bank->numRowMat)
													<< endl;
	cout << " |--- Mat Leakage Power    = " << TO_WATT(bank->mat.leakage) << " per mat" << endl;
	cout << "    |--- Predecoder Leakage Power = " << TO_WATT(bank->mat.rowPredecoderBlock1.leakage + bank->mat.rowPredecoderBlock2.leakage +
				bank->mat.bitlineMuxPredecoderBlock1.leakage + bank->mat.bitlineMuxPredecoderBlock2.leakage +
				bank->mat.senseAmpMuxLev1PredecoderBlock1.leakage + bank->mat.senseAmpMuxLev1PredecoderBlock2.leakage +
				bank->mat.senseAmpMuxLev2PredecoderBlock1.leakage + bank->mat.senseAmpMuxLev2PredecoderBlock2.leakage ) << endl;
	cout << "    |--- Subarray Leakage Power   = " << TO_JOULE(bank->mat.subarray.leakage) << endl;
	
	cout << "==============================================" << endl << "         RESULT BREAKDOWN (SUBARRAY)" << endl << "==============================================" << endl;
	bank->printbreakdown();

	// cout << "==============================" <<endl;
	// cout << bank->numBitSerial << endl;
	// cout << bank->mat.areaOptimizationLevel << endl;
	// cout << bank->mat.subarray.DriverOptLevel << endl;

    // cout << bank->area * 1e12 << endl;
    // cout << bank->searchLatency * 1e9 << endl;
    // cout << bank->searchDynamicEnergy * 1e12 << endl;
    // cout << bank->writeDynamicEnergy * 1e12 << endl;
    // cout << bank->leakage * 1e6 << endl;
	// cout << "==============================" <<endl;
}




