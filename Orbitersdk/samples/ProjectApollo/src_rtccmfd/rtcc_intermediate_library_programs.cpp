/****************************************************************************
This file is part of Project Apollo - NASSP

Real-Time Computer Complex (RTCC) Intermediate Library Programs

Project Apollo is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Project Apollo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Project Apollo; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

See http://nassp.sourceforge.net/license/ for more details.

**************************************************************************/

#include "rtcc.h"

//Gimbal, Thrust, and Weight Loss Rate Subroutine
void RTCC::GIMGBL(double CSMWT, double LMWT, double &RY, double &RZ, double &T, double &WDOT, int ITC, unsigned &IC, int IA, int IJ, double D)
{
	//INPUTS:
	//CSMWT and LMWT, CSM and LM weights
	//ITC: 33 = SPS, 34 = APS, 35 = DPS, 36 = J2
	//IC: 1 = CSM, 5 = CSM + LM Ascent, 12 = LM, 13 = CSM and LM (docked)
	//IA: -1: trim angles, thrust and weight loss rate desired outputs, 0: thrust and weight loss rate desired outputs, 1: trim angles desired outputs
	//IJ: LM descent stage included in configuration at beginning of this maneuver (only applicable if ITC=33 and IC=2). 0 = included, 1 = not included

	if (ITC < 33)
	{
		RY = 0.0;
		RZ = 0.0;
		return;
	}

	int IND;
	double R, W[3], K;
	VECTOR3 XCG[2], XI;

	//CSM only
	if (IC == 1)
	{
		W[0] = CSMWT;
		W[1] = 0.0;
	}
	//LM only
	else if (IC == 12)
	{
		W[0] = 0.0;
		W[1] = LMWT;
	}
	//Both
	else
	{
		W[0] = CSMWT;
		W[1] = LMWT;
	}
	W[2] = W[0] + W[1];

	if (ITC == 34)
	{
		goto RTCC_GIMGBL_LABEL_3_3;
	}
	else if (ITC > 34)
	{
		if (IA == 0)
		{
			goto RTCC_GIMGBL_LABEL_3_5;
		}
		IND = 2;
		K = SystemParameters.MGVDGD;
		//Use LM DSC CG Table
		XI = GIMGB2(SystemParameters.MHVLCG.Weight, SystemParameters.MHVLCG.CG, SystemParameters.MHVLCG.N, W[1]);
	}
	else
	{
		if (IA == 0)
		{
			goto RTCC_GIMGBL_LABEL_3_4;
		}
		if (IC == 1)
		{
			IND = 2;
		}
		else
		{
			IND = 1;
		}
		K = SystemParameters.MGVSGD;
		//Use CSM CG Table
		XI = GIMGB2(SystemParameters.MHVCCG.Weight, SystemParameters.MHVCCG.CG, SystemParameters.MHVCCG.N, W[0]);
	}

	XCG[IND - 1].x = XI.x - K;
	XCG[IND - 1].y = XI.y;
	XCG[IND - 1].z = XI.z;

	//CSM or LM, but not docked?
	if (IC <= 12 && IC != 5)
	{
		goto RTCC_GIMGBL_LABEL_3_2;
	}
	if (IND > 1)
	{
		//Use CSM CG Table
		XI = GIMGB2(SystemParameters.MHVCCG.Weight, SystemParameters.MHVCCG.CG, SystemParameters.MHVCCG.N, W[0]);
		IND = 1;
		K = SystemParameters.MGVSGD + SystemParameters.MGVSTD;
	}
	else
	{
		if (IJ != 0)
		{
			//Use LM w/o descent CG table
			XI = GIMGB2(SystemParameters.MHVACG.Weight, SystemParameters.MHVACG.CG, SystemParameters.MHVACG.N, W[1]);
		}
		else
		{
			//Use LM w/ descent CG table
			XI = GIMGB2(SystemParameters.MHVLCG.Weight, SystemParameters.MHVLCG.CG, SystemParameters.MHVLCG.N, W[1]);
		}
		IND = 2;
		K = SystemParameters.MGVDGD + SystemParameters.MGVSTD;
	}

	XI.x = XI.x - K;

	XCG[IND - 1] = mul(_M(-1.0, 0.0, 0.0, 0.0, -cos(240.0*RAD - D), -sin(240.0*RAD - D), 0.0, -sin(240.0*RAD - D), cos(240.0*RAD - D)), XI);

	XCG[1] = (XCG[0] * W[0] + XCG[1] * W[1]) / W[2];

RTCC_GIMGBL_LABEL_3_2:
	R = length(XCG[1]);
	if (R > 10e-6)
	{
		RZ = asin(XCG[1].y / R);
		if (XCG[1].x > 10e-6)
		{
			RY = atan(-XCG[1].z / XCG[1].x);
		}
		else
		{
			RY = 0.0;
		}
	}
	else
	{
		RZ = 0.0;
		RY = 0.0;
	}
	if (IA == 1)
	{
		return;
	}
	if (ITC == 34)
	{
	RTCC_GIMGBL_LABEL_3_3:;
		//GIMGB2();
	}
	else if (ITC < 34)
	{
	RTCC_GIMGBL_LABEL_3_4:;
		//GIMGB2();
	}
	else
	{
	RTCC_GIMGBL_LABEL_3_5:;
		//GIMGB2();
	}
	//T = XI.x;
	//WDOT = XI.y;
}

VECTOR3 RTCC::GIMGB2(const double *WArr, const VECTOR3 *VecArr, int N, double W)
{
	VECTOR3 XI;
	int I;
	if (W <= WArr[0])
	{
		I = 1;
	RTCC_GIMGB2_4:
		XI = VecArr[I - 1];
		goto RTCC_GIMGB2_2;
	}
	if (W >= WArr[N - 1])
	{
		goto RTCC_GIMGB2_1;
	}
	I = 2;
RTCC_GIMGB2_3:
	if (W < WArr[I - 1])
	{
		//Linearly interpolate
		XI = VecArr[I - 2] + (VecArr[I - 1] - VecArr[I - 2]) / (WArr[I - 1] - WArr[I - 2])*(W - WArr[I - 2]);
		goto RTCC_GIMGB2_2;
	}
	if (W == WArr[I - 1])
	{
		goto RTCC_GIMGB2_4;
	}
	if (I < N)
	{
		I++;
		goto RTCC_GIMGB2_3;
	}
RTCC_GIMGB2_1:
	I = N;
	goto RTCC_GIMGB2_4;
RTCC_GIMGB2_2:
	return XI;
}

//LM AGS External DV Coordinate Transformation Subroutine
VECTOR3 RTCC::PIAEDV(VECTOR3 DV, VECTOR3 R_CSM, VECTOR3 V_CSM, VECTOR3 R_LM, bool i)
{
	//INPUTS:
	// i = 0: inertial to LVLH, 1: LVLH to inertial 

	VECTOR3 H, Z_PA, P, X_PA, Y_PA, DV_out;

	H = crossp(R_CSM, V_CSM);
	Z_PA = -unit(R_LM);
	P = crossp(Z_PA, H);
	X_PA = unit(P);
	Y_PA = crossp(Z_PA, X_PA);
	if (i)
	{
		DV_out = X_PA * DV.x + Y_PA * DV.y + Z_PA * DV.z;
	}
	else
	{
		DV_out = _V(dotp(DV, X_PA), dotp(DV, Y_PA), dotp(DV, Z_PA));
	}

	return DV_out;
}

//Right Ascension of Greenwich at Time T
double RTCC::PIAIES(double hour)
{
	double HA;

	HA = SystemParameters.MCLAMD + SystemParameters.MCERTS*hour;
	OrbMech::normalizeAngle(HA);
	return HA;
}

int RTCC::PIATSU(AEGDataBlock AEGIN, AEGDataBlock &AEGOUT, double &isg, double &gsg, double &hsg)
{
	//Input AEGIN has to contain orbit-defining elements, a counter (DN, Item10) and a desired selenographic argument of latitude (UD, Item8)

	PMMLAEG aeg(this);
	AEGHeader header;
	MATRIX3 Rot;
	VECTOR3 P, W, P_apo, W_apo;
	double eps_i, eps_t, usg, du, theta_dot, dt;
	int KE, K;

	eps_i = 1e-4;
	eps_t = 0.01;
	header.AEGInd = 1;
	header.ErrorInd = 0;
	AEGIN.TIMA = 0;

	AEGOUT = AEGIN;
	KE = 0;
	K = 1;
RTCC_PIATSU_1A:
	//Rotate from selenocentric to selenographic
	PLEFEM(5, AEGOUT.TS / 3600.0, 0, NULL, NULL, NULL, &Rot);
	PIVECT(AEGOUT.coe_osc.i, AEGOUT.coe_osc.g, AEGOUT.coe_osc.h, P, W);
	P_apo = mul(Rot, P);
	W_apo = mul(Rot, W);
	PIVECT(P_apo, W_apo, isg, gsg, hsg);
	if (isg < eps_i || isg > PI - eps_i)
	{
		//Orbit plane essentially equatorial
		KE = 2;
		return KE;
	}
	usg = AEGOUT.f + gsg;
	if (usg >= PI2)
	{
		usg = usg - PI2;
	}
	if (K > 1)
	{
		du = AEGIN.Item8 - usg;
		theta_dot = sqrt(OrbMech::mu_Moon*AEGOUT.coe_osc.a*(1.0 - pow(AEGOUT.coe_osc.e, 2))) / pow(AEGOUT.R, 2) + AEGOUT.g_dot;
		if (abs(du) > PI)
		{
			if (du > 0)
			{
				du = du - PI2;
			}
			else if (du <= 0)
			{
				du = du + PI2;
			}
		}
	}
	else
	{
		du = AEGIN.Item8 - usg + AEGIN.Item10*PI2;
		theta_dot = sqrt(OrbMech::mu_Moon / pow(AEGIN.coe_osc.a, 3));
	}
	dt = du / theta_dot;
	if (abs(dt) <= eps_t)
	{
		return KE;
	}
	if (K >= 7)
	{
		KE = 1;
		return KE;
	}
	AEGIN.TE = AEGOUT.TS + dt;
	K++;
	aeg.CALL(header, AEGIN, AEGOUT);
	if (header.ErrorInd)
	{
		KE = -2;
		return KE;
	}
	goto RTCC_PIATSU_1A;
}

void RTCC::PIBETA(double BETA, double ONOVA, double *F)
{
	double BETA2, THETA2, TH2, THETA, TH;
	//false = normal logic, true = modified
	bool flag;
	static const double TWOPI2 = PI2 * PI2;

	flag = false;
	BETA2 = BETA * BETA;
	THETA2 = BETA2 * ONOVA;
	TH2 = THETA2;
	if (TH2 < 0)
	{
		if (TWOPI2 + TH2 < 0)
		{
			flag = true;
			THETA = sqrt(-THETA2);
			TH = fmod(THETA, PI2);
			//Diagnostic
			if (TH == 0)
			{
				TH = PI2;
			}
			TH2 = -TH * TH;
		}
	}

	F[0] = OrbMech::stumpS(-TH2);
	F[1] = OrbMech::stumpC(-TH2);
	F[2] = 1.0 + TH2 * F[0];
	F[3] = 1.0 + TH2 * F[1];

	if (flag)
	{
		F[0] = (F[0] * TH*TH2 + TH - THETA) / THETA / THETA2;
		F[1] = F[1] * TH2 / THETA2;
		F[2] = F[2] * TH / THETA;
	}
	F[0] = F[0] * BETA2*BETA;
	F[1] = F[1] * BETA2;
	F[2] = F[2] * BETA;
}

double RTCC::PIBSHA(double hour)
{
	double GMT = hour - SystemParameters.MCCBES;
	return PIGMHA(GMT);
}

void RTCC::PIBURN(VECTOR3 R, VECTOR3 V, double T, double *B, VECTOR3 &ROUT, VECTOR3 &VOUT, double &TOUT)
{
	//B[0]: DT
	//B[1]: DR
	//B[2]: DV
	//B[3]: DS (arc length)
	//B[4]: dgamma
	//B[5]: dpsi
	//B[6]: isp
	//B[7]: characteristic DV
	//B[8]: mass ratio

	VECTOR3 H, R1, V1, R2, V2, R3, V3;
	double D;

	TOUT = T + B[0];
	H = crossp(R, V);
	D = dotp(R, V);
	if (B[3] == 0.0)
	{
		R1 = R;
		V1 = V;
	}
	else
	{
		R1 = R * cos(B[3]) + (V*pow(length(R), 2) - R * D)*sin(B[3]) / length(H);
		V1 = V * cos(B[3]) + (V*D - R * pow(length(R), 2))*sin(B[3]) / length(H);
	}
	if (B[4] == 0.0)
	{
		R2 = R1;
		V2 = V1;
	}
	else
	{
		R2 = R1;
		V2 = V1 * cos(-B[4]) + (V1*D - R1 * pow(length(V), 2))*sin(-B[4]) / length(H);
	}
	if (B[5] == 0.0)
	{
		R3 = R2;
		V3 = V2;
	}
	else
	{
		R3 = R2;
		V3 = R * 2.0*(dotp(R2, V2)*pow(sin(-B[5] / 2.0), 2)) / pow(length(R), 2) + V2 * cos(-B[5]) + crossp(R2, V2)*sin(-B[5]) / length(R);
	}
	if (B[1] == 0.0)
	{
		ROUT = R3;
	}
	else
	{
		ROUT = R3 * (length(R) + B[1]) / length(R);
	}
	if (B[2] == 0.0)
	{
		VOUT = V3;
	}
	else
	{
		VOUT = V3 * (length(V) + B[2]) / length(V);
	}
	B[7] = sqrt(B[2] * B[2] + 4.0*length(V)*(length(V) + B[2])*pow(sin(B[4] / 2.0), 2) + cos(asin(dotp(unit(R), unit(V))))*cos(asin(dotp(unit(R), unit(V))) + B[4])*pow(sin(B[5] / 2.0), 2));
	B[8] = exp(-abs(B[7] / B[6]));
}

void RTCC::PICSSC(bool vecinp, VECTOR3 &R, VECTOR3 &V, double &r, double &v, double &lat, double &lng, double &gamma, double &azi)
{
	if (vecinp)
	{
		r = length(R);
		lat = asin(R.z / r);
		lng = atan2(R.y, R.x);
		if (lng < 0)
		{
			lng += PI2;
		}
		VECTOR3 TEMP = mul(_M(cos(lng)*cos(lat), sin(lng)*cos(lat), sin(lat), -sin(lng), cos(lng), 0, -sin(lat)*cos(lng), -sin(lat)*sin(lng), cos(lat)), V);
		v = length(TEMP);
		gamma = asin(TEMP.x / v);
		azi = atan2(TEMP.y, TEMP.z);
		if (azi < 0)
		{
			azi += PI2;
		}
	}
	else
	{
		R = _V(cos(lat)*cos(lng), cos(lat)*sin(lng), sin(lat))*r;
		V = mul(_M(cos(lat)*cos(lng), -sin(lng), -sin(lat)*cos(lng), cos(lat)*sin(lng), cos(lng), -sin(lat)*sin(lng), sin(lat), 0, cos(lat)), _V(sin(gamma), cos(gamma)*sin(azi), cos(gamma)*cos(azi))*v);
	}
}

MATRIX3 RTCC::PIDREF(VECTOR3 AT, VECTOR3 R, VECTOR3 V, double PG, double YG, bool K)
{
	//K: 0 for heads-down, 1 for heads-up
	VECTOR3 X_T, Y_T, Z_T, X_P, Y_P, Z_P;
	double r, y_T;

	X_T = AT;
	r = length(R);
	Y_T = crossp(R, X_T);
	y_T = length(Y_T);

	if (K)
	{
		Y_T = -Y_T;
	}
	if (y_T / r < 0.0017)
	{
		Y_T = crossp(R, V);
		y_T = length(Y_T);
	}
	Y_T = Y_T / y_T;
	Z_T = crossp(X_T, Y_T);

	X_P = X_T * cos(PG)*cos(YG) - Y_T * cos(PG)*sin(YG) + Z_T * sin(PG);
	Y_P = X_T * sin(YG) + Y_T * cos(YG);
	Z_P = -X_T * sin(PG)*cos(YG) + Y_T * sin(PG)*sin(YG) + Z_T * cos(PG);
	return _M(X_P.x, X_P.y, X_P.z, Y_P.x, Y_P.y, Y_P.z, Z_P.x, Z_P.y, Z_P.z);
}

VECTOR3 RTCC::PIEXDV(VECTOR3 R_ig, VECTOR3 V_ig, double WT, double T, VECTOR3 DV, bool i)
{
	//INPUTS:
	// i = 0: inertial to LVLH, 1: LVLH to inertial 
	VECTOR3 H, Y_PH, Z_PH, X_PH, DV_out;
	double h, rr, r, dv, theta, V_F, V_D;

	H = crossp(R_ig, V_ig);
	h = length(H);
	rr = dotp(R_ig, R_ig);
	r = sqrt(rr);
	Y_PH = -H / h;
	Z_PH = -R_ig / r;
	X_PH = crossp(Y_PH, Z_PH);
	dv = length(DV);
	theta = h * dv*WT / (2.0*rr*T);
	if (i)
	{
		double V_S;
		V_F = DV.x*cos(theta) - DV.z*sin(theta);
		V_S = DV.y;
		V_D = DV.x*sin(theta) + DV.z*cos(theta);
		DV_out = X_PH * V_F + Y_PH * V_S + Z_PH * V_D;
	}
	else
	{
		V_F = dotp(DV, X_PH);
		DV_out.y = dotp(DV, Y_PH);
		V_D = dotp(DV, Z_PH);
		DV_out.x = V_F * cos(theta) + V_D * sin(theta);
		DV_out.z = -V_F * sin(theta) + V_D * cos(theta);
	}

	return DV_out;
}

void RTCC::PIFAAP(double a, double e, double i, double f, double u, double r, double &r_apo, double &r_peri)
{
	double a_ref, e_ref, p_ref, p, K1, K2, df, r1, r2;

	a_ref = r + 1.5*OrbMech::J2_Earth * OrbMech::R_Earth*(1.0 - 3.0 / 2.0*pow(sin(i), 2) + 5.0 / 6.0*pow(sin(i), 2)*cos(2.0*u));
	e_ref = 1.0 - r / a_ref;
	p_ref = a_ref * (1.0 - e_ref * e_ref);
	p = a * (1.0 - e * e);
	K1 = e / sqrt(p);
	K2 = e_ref / sqrt(p_ref);
	df = atan2(K1*sin(f), K2 - K1 * cos(f));
	r1 = p / (1.0 + e * cos(f + df)) - p_ref / (1.0 + e_ref * cos(df)) + r;
	r2 = p / (1.0 - e * cos(f + df)) - p_ref / (1.0 - e_ref * cos(df)) + r;
	if (r1 >= r2)
	{
		r_apo = r1;
		r_peri = r2;
	}
	else
	{
		r_peri = r1;
		r_apo = r2;
	}
}

double RTCC::PIGBHA()
{
	int Y, E, D, XN;
	static const double A = 0.0929;
	static const double B = 8640184.542;
	static const double W1 = 1.720217954160054e-2;
	double C, T, DE, BHA, DI, DELTA;

	E = Y = GZGENCSN.Year;
	//July and later uses epoch of next year
	if (GZGENCSN.MonthofLiftoff >= 7)
	{
		E = E + 1;
	}
	D = GZGENCSN.RefDayOfYear;
	XN = (E - 1901) / 4;
	C = -86400.0*(double)(E - 1900) - 74.164;
	T = 2 * C / (-B - sqrt(B*B - 4 * A*C));
	DE = 36525.0*T - 365.0*(double)(E - 1900) + 0.5 - (double)XN;
	if (Y == E)
	{
		DI = D;
	}
	else
	{
		int X = Y % 4;
		if (X == 0)
		{
			DI = D - 366.0;
		}
		else
		{
			DI = D - 365.0;
		}
	}
	DELTA = DI - DE;
	BHA = PI2 / 3.6 + W1 * DELTA;
	return BHA;
}

double RTCC::PIGMHA(double hour)
{
	return SystemParameters.MCLAMD + hour * SystemParameters.MCERTS;
}

void RTCC::PIMCKC(VECTOR3 R, VECTOR3 V, int body, double &a, double &e, double &i, double &l, double &g, double &h)
{
	double mu;

	if (body == BODY_EARTH)
	{
		mu = OrbMech::mu_Earth;
	}
	else
	{
		mu = OrbMech::mu_Moon;
	}

	double v = length(V);
	double r = length(R);
	double eps = v * v / 2.0 - mu / r;
	VECTOR3 H = crossp(R, V);

	a = -mu / (2.0*eps);
	VECTOR3 E = crossp(V, H) / mu - R / r;
	e = length(E);
	i = acos(H.z / length(H));
	double theta = acos2(dotp(unit(E), unit(R)));
	if (dotp(R, V) < 0)
	{
		theta = PI2 - theta;
	}
	if (e > 1.0)
	{
		//Hyperbolic
		double F = log((sqrt(e + 1.0) + sqrt(e - 1.0)*tan(theta / 2.0)) / (sqrt(e + 1.0) - sqrt(e - 1.0)*tan(theta / 2.0)));
		if (F < 0)
		{
			F += PI2;
		}
		l = e * sinh(F) - F;
	}
	else if (e == 1.0)
	{
		//Parabolic
		double tan_theta2 = tan(theta / 2.0);
		l = 0.5*tan_theta2 + 1.0 / 6.0*pow(tan_theta2 / 2.0, 3);
	}
	else
	{
		//Elliptic
		double EE = atan2(sqrt(1.0 - e * e)*sin(theta), e + cos(theta));
		if (EE < 0)
		{
			EE += PI2;
		}
		l = EE - e * sin(EE);
	}
	VECTOR3 K = _V(0, 0, 1);
	VECTOR3 N = crossp(K, H);
	g = acos2(dotp(unit(N), unit(E)));
	if (E.z < 0)
	{
		g = PI2 - g;
	}
	h = acos2(N.x / length(N));
	if (N.y < 0)
	{
		h = PI2 - h;
	}
}

void RTCC::PITFPC(double MU, int K, double AORP, double ECC, double rad, double &TIME, double &P, bool erunits)
{
	//INPUT:
	//MU: gravitational constant
	//K: outward leg (0.) and return lef (1.) flag. k is input as a floating point number
	//AORP: semimajor axis or semilatus rectum (abs(e-1) < 0.00001 is the deciding number)
	//ECC: eccentricity
	//rad: radial distance from focus
	//erunits: Input units are Earth radii
	//OUTPUT:
	//TIME: Time from periapsis to the desired radial distance
	//P: Orbital period, only calculared if orbit is eccentric

	double eps;

	if (erunits)
	{
		eps = 1.e-5;
	}
	else
	{
		eps = 63.78165;
	}

	//Parabolic case
	if (abs(ECC - 1.0) < 0.00001)
	{
		double C3;

		//Calculate characteristic energy
		C3 = MU * (ECC*ECC - 1.0) / AORP;
		if (abs(C3) < eps)
		{
			//Calculate true anomaly at given distance r
			double eta_apo = acos(abs(AORP) / rad - 1.0);
			double TEMP1 = tan(eta_apo / 2.0);
			//Calculate time
			TIME = abs(AORP) / 2.0*sqrt(abs(AORP) / MU)*(TEMP1 + 1.0 / 3.0*pow(TEMP1, 3.0));

			if (K != 0)
			{
				TIME = -TIME;
			}
			return;
		}
		else
		{
			//Calculate semi major axis, use non parabolic calculations
			AORP = AORP / (1.0 - ECC * ECC);
		}
	}

	double E;

	//Elliptical case
	if (ECC < 1.0)
	{
		E = acos(1.0 / ECC * (1.0 - rad / AORP));
		P = PI2 * AORP*sqrt(AORP / MU);
		TIME = AORP * sqrt(AORP / MU)*(E - ECC * sin(E));
	}
	//Hyperbolic case
	else
	{
		double coshE;
		coshE = 1.0 / ECC * (1.0 - rad / AORP);
		E = log(coshE + sqrt(coshE*coshE - 1.0));
		TIME = AORP * sqrt(abs(AORP) / MU)*(E - ECC * (exp(E) - exp(-E)) / 2.0);
	}

	if (K != 0)
	{
		TIME = -TIME;
	}
}

int RTCC::PITCIR(AEGHeader header, AEGDataBlock in, double R_CIR, AEGDataBlock &out)
{
	//Output: 0 = no error, 1 = essentially circular orbit, 2 = unrecoverable AEG error, 3 = requested height not in orbit, 4 = failed to converge on radius

	double cos_f_CI, f_CI, dt, sgn, ddt, eps_t, l2, dl;
	int I, IMAX;
	bool fail;

	IMAX = 10;
	eps_t = 0.01;

	if (in.ENTRY == 0)
	{
		//Initialize
		PMMAEGS(header, in, in);
		//Unrecoverable AEG error
		if (header.ErrorInd)
		{
			PMXSPT("PITCIR", 17);
			return 2;
		}
	}

	out = in;

	//Too circular?
	if (header.AEGInd == BODY_EARTH)
	{
		if (in.coe_mean.e < 0.001)
		{
			PMXSPT("PITCIR", 17);
			return 1;
		}
	}
	else
	{
		if (in.coe_mean.e < 0.0001)
		{
			PMXSPT("PITCIR", 17);
			return 1;
		}
	}

	in.TIMA = 0;

	cos_f_CI = (in.coe_osc.a*(1.0 - in.coe_osc.e*in.coe_osc.e) - R_CIR) / (in.coe_osc.e*R_CIR);
	fail = false;
	if (abs(cos_f_CI) > 1.0)
	{
		fail = true;
		cos_f_CI = cos_f_CI / abs(cos_f_CI);
	}
	f_CI = acos(cos_f_CI);
	if (f_CI >= in.f)
	{
		dt = (f_CI - in.f) / (in.g_dot+in.l_dot);
		sgn = 1.0;
	}
	else
	{
		if (PI2 - f_CI - in.f > 0)
		{
			dt = (PI2 - f_CI - in.f) / (in.g_dot + in.l_dot);
			sgn = -1.0;
		}
		else
		{
			dt = (f_CI + PI2 - in.f) / (in.g_dot + in.l_dot);
			sgn = 1.0;
		}
	}

	I = 0;

	do
	{
		in.TE = in.TS + dt;
		PMMAEGS(header, in, out);
		//AEG error
		if (header.ErrorInd)
		{
			PMXSPT("PITCIR", 17);
			return 2;
		}
		//Calculate desired true anomaly
		cos_f_CI = (out.coe_osc.a*(1.0 - out.coe_osc.e*out.coe_osc.e) - R_CIR) / (out.coe_osc.e*R_CIR);
		if (abs(cos_f_CI) > 1.0)
		{
			cos_f_CI = cos_f_CI / abs(cos_f_CI);
		}
		f_CI = sgn*acos(cos_f_CI);
		if (f_CI < 0)
		{
			f_CI += PI2;
		}
		//Calculate angle difference
		l2 = OrbMech::TrueToMeanAnomaly(f_CI, out.coe_osc.e);
		dl = l2 - out.coe_osc.l;
		if (dl > PI)
		{
			dl -= PI2;
		}
		else if (dl < -PI)
		{
			dl += PI2;
		}

		//Calculate ddt
		ddt = dl / (out.g_dot + out.l_dot);
		if (abs(ddt) > eps_t)
		{
			dt = dt + ddt;
		}
		I++;
	} while (abs(ddt) > eps_t && I < IMAX);

	//Failed to converge
	if (I >= IMAX)
	{
		PMXSPT("PITCIR", 19);
		return 4;
	}
	//Height not in orbit
	if (fail)
	{
		PMXSPT("PITCIR", 18);
		return 3;
	}

	return 0;
}

void RTCC::PIVECT(VECTOR3 P, VECTOR3 W, double &i, double &g, double &h)
{
	VECTOR3 n;

	i = acos(W.z / length(W));
	n = crossp(_V(0, 0, 1), W);
	h = acos(n.x / length(n));
	if (n.y < 0)
	{
		h = PI2 - h;
	}
	g = acos(dotp(unit(n), unit(P)));
	if (P.z < 0)
	{
		g = PI2 - g;
	}
}

void RTCC::PIVECT(double i, double g, double h, VECTOR3 &P, VECTOR3 &W)
{
	P = _V(-sin(h)*cos(i)*sin(g) + cos(h)*cos(g), cos(h)*cos(i)*sin(g) + sin(h)*cos(g), sin(i)*sin(g));
	W = _V(sin(h)*sin(i), -cos(h)*sin(i), cos(i));
}