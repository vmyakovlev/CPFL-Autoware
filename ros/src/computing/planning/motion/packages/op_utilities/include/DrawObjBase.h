/*
 * DrawObjBase.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef DRAWOBJBASE_TESTING
#define DRAWOBJBASE_TESTING

#include <string>
namespace OP_TESTING_NS
{

enum DISPLAY_SHAP {DISP_WHEEL, DISP_PEDAL, DISP_TEXT};

class DisplayDataObj
{
public:
	double x;
	double y;
	double value;
	std::string text;
	DISPLAY_SHAP shape;

	DisplayDataObj()
	{
		x = y = value = 0;
		shape = DISP_TEXT;
	}
};

class DrawObjBase
  {
  public:

    virtual void DrawSimu()=0;
    virtual void DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)=0;
    virtual void OnLeftClick(const double& x, const double& y) = 0;
    virtual void OnRightClick(const double& x, const double& y) = 0;
    virtual void OnKeyboardPress(const int& sKey, const unsigned char& key) = 0;

    DrawObjBase();
    virtual ~DrawObjBase();

    double m_followX;
    double m_followY;
    double m_followZ;
    double m_followA;
  };

}

#endif
