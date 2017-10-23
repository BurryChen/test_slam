/*
 * GlobalConst.h
 *
 *  Created on: 18.1.2010
 *      Author: admin
 */

#ifndef GEODEF_H_
#define GEODEF_H_

#ifndef WIN32
//#include <e32base.h>
#else
#include "sysheader.h"
#endif

//#include "datastructures.h"

// definition dependent on the OS platform
typedef int TInt;
//typedef double TReal32;
//typedef double TReal64;
//typedef bool TBool;
//typedef short TUint;

//#define ETrue 1
//#define EFalse 0

//================================================================================
#define	WGSWe					0.00007292115							// WGS-84 earth rotation rate
#define WGSSpeedOfLight			299792458.0								// WGS-84 speed of light in m/sec
#define L1_WaveLength			(WGSSpeedOfLight/1.57542e9)
#define Pi						3.1415926535897932384626433832795				// WGS 84 value of pi
#define RADDEG					57.295779513							// WGS 84 value of 180/pi
#define WGSWedot				7.2921151467E-5							// WGS 84 value of earth's rotation rate
#define WGSMu					3.986004418E+14							// WGS 84 value of earth's univ. grav. par
#define WGSRelCorrConst			-4.442807633E-10						//F - relativistic correction term constant
#define WGSMajor				6378137.0								//a - WGS-84 earth's semi major axis
#define	WGSMinor				6356752.3142							//b - WGS-84 earth's semi minor axis

#define WGSE1sqr				0.00669437999014						// first  numerical eccentricity
#define WGSE2sqr				0.0067394967422751						// second numerical eccentricity

#define WGS84_SQRT_U    		(1.99649818432174e7)
#define WGS84_OMEGDOTE  		(7.2921151467e-5)


#define MAX_SV_INVIEW 16

/// Vector in XYZ format.
typedef struct
{
    double dX; ///< X coordinate.
    double dY; ///< Y coordinate.
    double dZ; ///< Z coordinate.
    bool bAvailable;
} XYZ_POS;

/// Vector in LLA format, with the latitude and longitude expressed in
/// radians.
typedef struct
{
    double dLat; ///< Latitude in radians.
    double dLon; ///< Longitude in radians.
    double dAlt; ///< Altitude in meters.
    bool bAvailable;
} LLA_POS;
void LLA2XYZ(LLA_POS *llap, XYZ_POS *pXYZ);
void XYZ2LLA(XYZ_POS *pXYZ, LLA_POS *llap);

// Converts WGS-84 Lat, Lon and Alt to ECEF XYZ
void LLA2XYZ(LLA_POS *llap, XYZ_POS *pXYZ)
{
	double N;
	N = WGSMajor / sqrt(1.0 - WGSE1sqr * sin(llap->dLat) * sin(llap->dLat));
	pXYZ->dX = (N + llap->dAlt) * cos(llap->dLat) * cos(llap->dLon);
	pXYZ->dY = (N + llap->dAlt) * cos(llap->dLat) * sin(llap->dLon);
	pXYZ->dZ = (N * (1.0 - WGSE1sqr) + llap->dAlt) * sin(llap->dLat);
	pXYZ->bAvailable = true;
	return;
}

// Converts WGS-84 ECEF XYZ (in meters) to Lat, Lon, Alt 
void XYZ2LLA(XYZ_POS *pXYZ, LLA_POS *llap)
{
	double p, theta, sinTheta, cosTheta, temp, temp1;
	double upLat, lowLat;
	double x, y, z;
	
	x = pXYZ->dX; 
	y = pXYZ->dY; 
	z = pXYZ->dZ; 
	
	p = sqrt(x * x + y * y);
	if (p <= 0.0)
		{
		// input value not valid
		llap->bAvailable = false;
		return;
	}
	temp = (z * WGSMajor) / (p * WGSMinor);
	theta = atan(temp);
	sinTheta = sin(theta); 
	cosTheta = cos(theta);
	upLat = z + WGSE2sqr * WGSMinor * sinTheta * sinTheta * sinTheta;
	lowLat = p - WGSE1sqr * WGSMajor * cosTheta * cosTheta * cosTheta;

	llap->dLat = atan2(upLat, lowLat);
	
	llap->dLon = atan2(y, x);
	
	temp = WGSMajor / sqrt(1.0 - WGSE1sqr * sin(llap->dLat) * sin(llap->dLat));
	temp1 = cos(llap->dLat);
	llap->dAlt = (p / temp1) - temp;

	llap->bAvailable = true;

	return;
}


/*typedef enum
{ 
	SinceLastEpoch = 0, 
	SinceLastBTPos,	
	SinceLastWLANPos
}DistanceTypes; 


struct ACCRAWDATA
{
	double SecondDiffFromStart;
	double XAcc;
	double YAcc;
	double ZAcc;
	double GAcc;//General Acc, iniated with 0;
};

/// Vector in XYZ format.
typedef struct
{
    double dX; ///< X coordinate.
    double dY; ///< Y coordinate.
    double dZ; ///< Z coordinate.
    Bool bAvailable;
} XYZ_POS;

/// Vector in LLA format, with the latitude and longitude expressed in
/// radians.
typedef struct
{
    double dLat; ///< Latitude in radians.
    double dLon; ///< Longitude in radians.
    double dAlt; ///< Altitude in meters.
    TBool bAvailable;
} LLA_POS;


typedef struct
{
	TInt iPRN;
	TInt SVCN0;
        double SVAzimuth;
        double SVElevation;
} ASatelliteData;



struct TQualityGPS
	{
	ASatelliteData SVData[MAX_SV_INVIEW];
        double HRMS;
        double VRMS;
        double HDOP;
        double VDOP;
        double TDOP;
	TInt NumSVinView;
	TInt NumSVused;
	};

struct TQualityWireless
	{
        double SignificanceLevel;
	TUint NumAPReceived;
        double Reserved; // the DOP of APs, ...
	};

//Position data structure
struct TPosData
{
	//TPositionModuleId ModuleId;
	PositioningModule ModuleId;
        double Latitute;
        double Longitute;
        double Altitude;
        double Heading; // radian
        double Speed;
	//TReal32 
	//TDateTime DateTime;	
#ifndef WIN32
	TTime Time;
#endif
	TBool Available;
};


//Position data structure
struct TPosDataLite
{
	//TPositionModuleId ModuleId;
	PositioningModule ModuleId;
        double Latitute;
        double Longitute;
        double Altitude;
        double Heading; // radian
        double Speed;
	//TReal32 
	//TDateTime DateTime;	
        double TimeDiff;
	TBool Available;
};

//Acc results data structure
struct TACCSPEED
{
        double EstSpeed; // speed from ACC
        double Accuracy;
};
//Position Measurement structure
struct TPosMeasurements
{
	//TPositionModuleId ModuleId;
	TPosData PosData;
        double MMHeading; // degrees
	TACCSPEED AccSpeed; // speed from ACC
	TQualityGPS QualityGPS;
	TQualityWireless QualityWLAN;
	TQualityWireless QualityBT;
};

//Position Measurement structure
struct TPosMeasurementsLite
{
	//TPositionModuleId ModuleId;
	TPosDataLite PosData;
        double MMHeading; // degrees
	TACCSPEED AccSpeed; // speed from ACC
	TQualityGPS QualityGPS;
	TQualityWireless QualityWLAN;
	TQualityWireless QualityBT;
};

TBool MN(double* M, double* N, double* P, const TInt m, const TInt k, const TInt n);
TBool MNT(double* M, double* N, double* P, const TInt m, const TInt k, const TInt n);
TBool MV(double* M, double* N, double* V, const TInt m, const TInt n);
TBool LUDCMP(TInt n,double *flag);
TBool LUBKSB(TInt n);
TBool dMatrixInv(double *Matrix,TInt Dim);
void TrimAngle(double* pAngle);

void ENU2XYZConvertMatrix( ENU_VECTOR* pENU,XYZ_POS* pRef_XYZ, double ConvertMatrix[3][3],XYZ_POS* pXYZ);
void LLA2XYZ(LLA_POS *llap, XYZ_POS *pXYZ);
void XYZ2LLA(XYZ_POS *pXYZ, LLA_POS *llap);
void XYZ2ENUConvertMatrix(XYZ_POS* pXYZ,XYZ_POS* pRef_XYZ, double ConvertMatrix[3][3], ENU_VECTOR* pENU);
void GetXYZ2ENUMatrix(LLA_POS* pRef_LLA, double ConvertMatrix[3][3]);
void GetENU2XYZMatrix(LLA_POS* pRef_LLA, double ConvertMatrix[3][3]);
double GetDirection(XYZ_POS *referXYZ, XYZ_POS *endXYZ, double ConvertMatrix[3][3], ENU_VECTOR* endENU);
//
void PosMsr2Lite(double timeDiff, TPosMeasurements* pMsr, TPosMeasurementsLite* pMsrLite);
void PosMsrLite2PosMsr(TPosMeasurementsLite* pMsrLite, TPosMeasurements* pMsr);
double GetTimeDiff (UTC_TIME time1, UTC_TIME time2);*/

#endif /* GEODEF_H_ */
