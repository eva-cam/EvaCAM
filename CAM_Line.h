/*
 * CAM_Line.h
 *
 *  some line, whatever wl, bl, sl, ml, etc
 */

#ifndef CAM_LINE_H_
#define CAM_LINE_H_

#include "CAM_Cell.h"

using namespace std;

class CAM_Line{
public:
	CAM_Line();
	virtual ~CAM_Line();

	/* Functions */
	void Initialize(bool _isRow, int _index, double _len, long long _numCell);
	void Initialize(double _len, long long _numCell, double _MuxWidth); // for mux signal only
	void CalcMaxCurrent();
	void CalcMuxWidth();
	void PrintLine();

	/* Properties */
	bool initialized;
	bool invalid;

	CAM_CellPort CellPort;
	int index;
	double len;
	double cap;
	double res;
	bool isRow;
	double numCell;
	double maxCurrent;
	double minMuxWidth;
};



#endif /* CAM_LINE_H_ */
