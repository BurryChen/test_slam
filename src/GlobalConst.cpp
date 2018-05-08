/*
 * GlobalConst.cpp
 *
 *  Created on: 18.1.2010
 *      Author: admin
 */

#include "GlobalConst.h"
#include <math.h>

void PosMsr2Lite(double timeDiff, TPosMeasurements* pMsr, TPosMeasurementsLite* pMsrLite)
	{
	pMsrLite->AccSpeed = pMsr->AccSpeed;
	pMsrLite->MMHeading = pMsr->MMHeading;
	pMsrLite->QualityGPS = pMsr->QualityGPS;
	pMsrLite->QualityWLAN = pMsr->QualityWLAN;
	pMsrLite->QualityBT = pMsr->QualityBT;
	pMsrLite->PosData.ModuleId = pMsr->PosData.ModuleId; 
	pMsrLite->PosData.Latitute = pMsr->PosData.Latitute; 
	pMsrLite->PosData.Longitute = pMsr->PosData.Longitute; 
	pMsrLite->PosData.Altitude = pMsr->PosData.Altitude;
	pMsrLite->PosData.Heading = pMsr->PosData.Heading; 
	pMsrLite->PosData.Speed = pMsr->PosData.Speed;
	pMsrLite->PosData.Available = pMsr->PosData.Available; 
	
	pMsrLite->PosData.TimeDiff = timeDiff;
	}

void PosMsrLite2PosMsr(TPosMeasurementsLite* pMsrLite, TPosMeasurements* pMsr)
	{
	pMsr->AccSpeed = pMsrLite->AccSpeed;
	pMsr->MMHeading = pMsrLite->MMHeading;
	pMsr->QualityGPS = pMsrLite->QualityGPS;
	pMsr->QualityWLAN = pMsrLite->QualityWLAN;
	pMsr->QualityBT = pMsrLite->QualityBT;
	pMsr->PosData.ModuleId = pMsrLite->PosData.ModuleId; 
	pMsr->PosData.Latitute = pMsrLite->PosData.Latitute; 
	pMsr->PosData.Longitute = pMsrLite->PosData.Longitute; 
	pMsr->PosData.Altitude = pMsrLite->PosData.Altitude;
	pMsr->PosData.Heading = pMsrLite->PosData.Heading; 
	pMsr->PosData.Speed = pMsrLite->PosData.Speed;
	pMsr->PosData.Available = pMsrLite->PosData.Available; 
	
	}

// Converts WGS-84 Lat, Lon and Alt to ECEF XYZ
void LLA2XYZ(LLA_POS *llap, XYZ_POS *pXYZ)
{
	double N;
	N = WGSMajor / sqrt(1.0 - WGSE1sqr * sin(llap->dLat) * sin(llap->dLat));
	pXYZ->dX = (N + llap->dAlt) * cos(llap->dLat) * cos(llap->dLon);
	pXYZ->dY = (N + llap->dAlt) * cos(llap->dLat) * sin(llap->dLon);
	pXYZ->dZ = (N * (1.0 - WGSE1sqr) + llap->dAlt) * sin(llap->dLat);
	pXYZ->bAvailable = ETrue;
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
		llap->bAvailable = EFalse;
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

	llap->bAvailable = ETrue;

	return;
}
// Get the direction from startLLA to endLLA according to these two LLA coordinates
// The latitude and longitude should be radian unit!
double GetDirection(XYZ_POS *referXYZ, XYZ_POS *endXYZ, double ConvertMatrix[3][3], ENU_VECTOR* endENU)
	{
	double ArcAzim;

	XYZ2ENUConvertMatrix(endXYZ, referXYZ, ConvertMatrix, endENU);
	
	if(fabs(endENU->dNorth) < 0.00000001)
		{
		if(endENU->dEast >= 0)
			ArcAzim = Pi*0.5;
		else
			ArcAzim = Pi*1.5;
		}
	else
		{
		ArcAzim = atan(endENU->dEast/endENU->dNorth); // need consider the range, converting to azimuth!
		// convert the angle from [-PI/2, PI/2] to [0 2*PI]
		if(ArcAzim < 0)
			{
			if(endENU->dEast < 0)
				ArcAzim = Pi*2+ArcAzim;
			else
				ArcAzim = Pi+ArcAzim;
			}
		else if(endENU->dEast < 0 && endENU->dNorth < 0)
			{
			ArcAzim = Pi+ArcAzim;
			}
		}
	return ArcAzim;
	}

/**************************************************************************** */
//
// Converts wgs84 ecef coordinated into a local north,east,up frame
// rotated around the reference wgs84 ecef position
//
/****************************************************************************/
void XYZ2ENUConvertMatrix(XYZ_POS* pXYZ,XYZ_POS* pRef_XYZ, double ConvertMatrix[3][3], ENU_VECTOR* pENU)
{
 // double M[3][3];
 // double ref_el,ref_az;
 // double tempd;

#if 0
  // convert reference point to spherical earth centered coords
  tempd = sqrt(pRef_XYZ->dX*pRef_XYZ->dX + pRef_XYZ->dY*pRef_XYZ->dY);
  ref_el = atan2(pRef_XYZ->dZ,tempd);
  ref_az = atan2(pRef_XYZ->dY,pRef_XYZ->dX);
#endif
  
  pXYZ->dX -= pRef_XYZ->dX;
  pXYZ->dY -= pRef_XYZ->dY;
  pXYZ->dZ -= pRef_XYZ->dZ;
#if 0
  M[0][0] = -sin(ref_el)*cos(ref_az); // for north
  M[0][1] = -sin(ref_el)*sin(ref_az);
  M[0][2] = cos(ref_el);
  M[1][0] = -sin(ref_az); // for east
  M[1][1] = cos(ref_az);
  M[1][2] = 0.0;
  M[2][0] = cos(ref_el)*cos(ref_az); // for up
  M[2][1] = cos(ref_el)*sin(ref_az);
  M[2][2] = sin(ref_el);
#endif
  pENU->dEast = ConvertMatrix[0][0]*pXYZ->dX + ConvertMatrix[0][1]*pXYZ->dY + ConvertMatrix[0][2]*pXYZ->dZ; // east
  pENU->dNorth = ConvertMatrix[1][0]*pXYZ->dX + ConvertMatrix[1][1]*pXYZ->dY + ConvertMatrix[1][2]*pXYZ->dZ; // north
  pENU->dUp = ConvertMatrix[2][0]*pXYZ->dX + ConvertMatrix[2][1]*pXYZ->dY + ConvertMatrix[2][2]*pXYZ->dZ; // up
}

/**************************************************************************** */
//
// Converts wgs84 ecef coordinated into a local north,east,up frame
// rotated around the reference wgs84 ecef position
//
/****************************************************************************/
void ENU2XYZConvertMatrix( ENU_VECTOR* pENU,XYZ_POS* pRef_XYZ, double ConvertMatrix[3][3],XYZ_POS* pXYZ)
{
 // double M[3][3];
 // double ref_el,ref_az;
 // double tempd;
  

  pXYZ->dX = ConvertMatrix[0][0]*pENU->dEast + ConvertMatrix[0][1]*pENU->dNorth + ConvertMatrix[0][2]*pENU->dUp; // east
  pXYZ->dY = ConvertMatrix[1][0]*pENU->dEast + ConvertMatrix[1][1]*pENU->dNorth + ConvertMatrix[1][2]*pENU->dUp; // north
  pXYZ->dZ = ConvertMatrix[2][0]*pENU->dEast + ConvertMatrix[2][1]*pENU->dNorth + ConvertMatrix[2][2]*pENU->dUp; // up
  
  pXYZ->dX += pRef_XYZ->dX;
  pXYZ->dY += pRef_XYZ->dY;
  pXYZ->dZ += pRef_XYZ->dZ;
}

void GetXYZ2ENUMatrix(LLA_POS* pRef_LLA, double ConvertMatrix[3][3])
	{
	  ConvertMatrix[0][0] = -sin(pRef_LLA->dLon); // for east
	  ConvertMatrix[0][1] = cos(pRef_LLA->dLon);
	  ConvertMatrix[0][2] = 0.0;
	  ConvertMatrix[1][0] = -sin(pRef_LLA->dLat)*cos(pRef_LLA->dLon); // for north
	  ConvertMatrix[1][1] = -sin(pRef_LLA->dLat)*sin(pRef_LLA->dLon);
	  ConvertMatrix[1][2] = cos(pRef_LLA->dLat);
	  ConvertMatrix[2][0] = cos(pRef_LLA->dLat)*cos(pRef_LLA->dLon); // for down
	  ConvertMatrix[2][1] = cos(pRef_LLA->dLat)*sin(pRef_LLA->dLon);
	  ConvertMatrix[2][2] = sin(pRef_LLA->dLat);
	  
	  // conver "down" to "up"
	  ConvertMatrix[2][0] = -1*ConvertMatrix[2][0];
	  ConvertMatrix[2][1] = -1*ConvertMatrix[2][1];
	  ConvertMatrix[2][2] = -1*ConvertMatrix[2][2];
	}

void GetENU2XYZMatrix(LLA_POS* pRef_LLA, double ConvertMatrix[3][3])
	{
	  ConvertMatrix[0][0] = -sin(pRef_LLA->dLon); // for east
	  ConvertMatrix[0][1] = -sin(pRef_LLA->dLat)*cos(pRef_LLA->dLon);
	  ConvertMatrix[0][2] = cos(pRef_LLA->dLat)*cos(pRef_LLA->dLon);
	  ConvertMatrix[1][0] = cos(pRef_LLA->dLon); // for north
	  ConvertMatrix[1][1] = -sin(pRef_LLA->dLat)*sin(pRef_LLA->dLon);
	  ConvertMatrix[1][2] = cos(pRef_LLA->dLat)*sin(pRef_LLA->dLon);
	  ConvertMatrix[2][0] = 0; // for up
	  ConvertMatrix[2][1] = cos(pRef_LLA->dLat);
	  ConvertMatrix[2][2] = sin(pRef_LLA->dLat);
	}

//P[m][n]=M[m][k]*N[k][n]
/*
	Input:
	Output:
	Function: Do multiplication of two matrix M x N
*/
TBool MN(double* M, double* N, double* P, const TInt m, const TInt k, const TInt n)
{
	TInt i,j,l;
	double in1, in2, out;

	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			*(P+i*n+j) = 0.0;
			for(l=0;l<k;l++)
			{
				in1 = *(double*)(M+i*k+l);
				in2 = *(double*)(N+l*n+j);
				out = in1*in2;
				*(P+i*n+j) +=out;
			}
		}
	}
	return ETrue;
}


/*
	Input:
	Output:
	Function: Do multiplication of two matrix, M and transpose of N( N' )
*/
//P[m][n]=M[m][k]*TRNASPOSE(N[n][k])
TBool MNT(double* M, double* N, double* P, const TInt m, const TInt k, const TInt n)
{
	TInt i,j,l;
	double in1, in2, out;

	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			*(P+i*n+j) = 0.0;
			for(l=0;l<k;l++)
			{
				in1 = *(double*)(M+i*k+l);
				in2 = *(double*)(N+j*k+l);
				out = in1*in2;
				*(P+i*n+j) +=out;
			}
		}
	}
	return ETrue;
}

/*
	Input:
	Output:
	Function: Do multiplication of Matrxi M (m x n)and vector N (n x 1)
*/
//V[m]=M[m][n]*N[n]
TBool MV(double* M, double* N, double* V, const TInt m, const TInt n)
{
	TInt i,j;
	double in;

	for(i=0;i<m;i++)
	{
		V[i]=0.0;
		for(j=0;j<n;j++)
		{
			in=*(double*)(M+i*n+j);
			V[i] += in*N[j];
		}
	}
	return ETrue;
}


#define MAX_DIM 2
#define TINY 0.00000000000000000001
#define ABS(a) (((a) >= 0) ? (a) : (-(a)))
double MatrixSpace[MAX_DIM][MAX_DIM];//allocate memory space for 2D temporary Matrix stored the origin matrix needed Inverse
TInt indx[MAX_DIM];
double BSpace[MAX_DIM];
double VVSpace[MAX_DIM];


//===================Matrix inverse for its element in double decision
TBool dMatrixInv(double *Matrix,TInt Dim)
{
	TInt i,j,n;
	double d;
	n=Dim;
	//=========================initialize static variables
    for(i=0;i<MAX_DIM;i++)
    {
        indx[i]=0;
        BSpace[i]=0;
        VVSpace[i]=0;
        for(j=0;j<MAX_DIM;j++)
        	MatrixSpace[i][j]=0;
    }
    //========================copy origin matrix to matrixspace
    for(i=0;i<n;i++)
    {
        for(j=0;j<n;j++)
            MatrixSpace[i][j]=*(Matrix+i*n+j);
    }
	//=========LU decomposition
	if(!LUDCMP(n,&d)) 
		return FALSE;
  //  DBG_TraceDouble("MatrixSpace[0][0]=",MatrixSpace[0][0]);
    //        DBG_TraceDouble("MatrixSpace[0][1]=",MatrixSpace[0][1]);
      //      DBG_TraceDouble("MatrixSpace[0][2]=",MatrixSpace[0][2]);
        //    DBG_TraceDouble("MatrixSpace[1][0]=",MatrixSpace[1][0]);
         //   DBG_TraceDouble("MatrixSpace[1][1]=",MatrixSpace[1][1]);
         //   DBG_TraceDouble("MatrixSpace[1][2]=",MatrixSpace[1][2]);
         //   DBG_TraceDouble("MatrixSpace[2][0]=",MatrixSpace[2][0]);
          //  DBG_TraceDouble("MatrixSpace[2][1]=",MatrixSpace[2][1]);
           // DBG_TraceDouble("MatrixSpace[2][2]=",MatrixSpace[2][2]);
	//=========LU decomposition

	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
			BSpace[j]=0;
		BSpace[i]=1.0;
  
		if(!LUBKSB(n))
			return EFalse;
 
		for(j=0;j<n;j++)
			*(Matrix+j*n+i)=BSpace[j];
	}
    return ETrue;
}


//===================LUDCMP
TBool LUDCMP(TInt n,double *flag)
{
	//n is the Matrix's Dimension
	//flag is an output indicating the row interchanges was even or odd.
	double big,dum,sum,temp;
	TInt i,imax,j,k;

    double *vv=VVSpace;

	*flag=1.0;
	imax=0;
	for (i=0;i<n;i++) 
	{//find the pivoting element,utilize every row
		big=0.0;
		for (j=0;j<n;j++)
			{
			if ((temp=ABS(MatrixSpace[i][j])) > big) 
				big=temp;
			}
		if (big == 0.0) 
		{
			return EFalse;
		}
		vv[i]=1.0/big;//save the scaling
	}
	for (j=0;j<n;j++)
	{
		for (i=0;i<j;i++) 
		{
			sum=MatrixSpace[i][j];
			for (k=0;k<i;k++) 
				sum -= MatrixSpace[i][k]*MatrixSpace[k][j];
			MatrixSpace[i][j]=sum;
		}//i=0 loop
		big=0.0;
		for (i=j;i<n;i++) 
		{
			sum=MatrixSpace[i][j];
			for (k=0;k<j;k++)
				sum -= MatrixSpace[i][k]*MatrixSpace[k][j];
			 MatrixSpace[i][j]=sum;
			if ( (dum=vv[i]*ABS(sum)) >= big) 
			{
				big=dum;
				imax=i;
			}
		}//i=j loop
		if (j != imax) {
			for (k=0;k<n;k++) {
				dum=MatrixSpace[imax][k];
				MatrixSpace[imax][k]=MatrixSpace[j][k];
				MatrixSpace[j][k]=dum;
			}
			*flag = -(*flag);
			vv[imax]=vv[j];
		}//j~=imax
		indx[j]=imax;
		if (MatrixSpace[j][j] == 0.0) MatrixSpace[j][j]=TINY;
		if (j != n-1) {
			dum=1.0/(MatrixSpace[j][j]);
			for (i=j+1;i<n;i++) MatrixSpace[i][j] *= dum;
		}//j~=n
	}//j loop
	return ETrue;
}
//===================LUDCMP========

//=====================LUBKSB
TBool LUBKSB(TInt n)
{
	TInt i,ip,j;
    TInt ii;
	double sum;
    ii=0;
	for (i=0;i<n;i++) {
		ip=indx[i];
     
		sum=BSpace[ip];
      //debug   DBG_TraceDouble("sum=",sum);
		BSpace[ip]=BSpace[i];
      //debug   DBG_TraceDouble("BSpace[ip]=",BSpace[ip]);
		if (ii)
        {
			for (j=ii-1;j<=i-1;j++) sum -= MatrixSpace[i][j]*BSpace[j];
          //debug   DBG_TraceDouble("if-ii:sum=",sum);
        }
		if(ii==0&&sum!=0.0) 
          {
             ii=i+1;//in fact ii=i,but for logic,correct ii=i+1 so that the 2 loop can enter if(ii)
           }
		BSpace[i]=sum;
      //debug   DBG_TraceDouble("BSpace[i]=",BSpace[i]);
	}
	for (i=n-1;i>=0;i--) {
		sum=BSpace[i];
		for (j=i+1;j<=n-1;j++) sum -= MatrixSpace[i][j]*BSpace[j];
       //debug  DBG_TraceDouble("if-i-n=1:sum=",sum);
		BSpace[i]=sum/MatrixSpace[i][i];
	}

return ETrue;
}

/*
	Input: 
	Output: 
	Function:  Trim the angle to the range [0, 2Pi]
*/
void TrimAngle(double* pAngle)
{
	double ang = 0.0;

	ang = *pAngle;
	while(ang < 0.0)
		ang += 6.283185307179586476925286766559;
	while(ang > 6.283185307179586476925286766559) 
		ang -= 6.283185307179586476925286766559;
	*pAngle = ang;
}

double GetTimeDiff (UTC_TIME time1, UTC_TIME time2)
{
    double dRet;
    dRet = (time1.iHour - time2.iHour)*3600+(time1.iMinute - time2.iMinute)*60+(time1.iSec-time2.iSec)+(time1.iMsec-time2.iMsec)/1000.0;

    return dRet;
}
