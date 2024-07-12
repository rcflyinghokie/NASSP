/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Lunar Descent Planning Processor (Header)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  **************************************************************************/

#pragma once

#include "OrbMech.h"
#include "RTCCModule.h"
#include "RTCCTables.h"

struct LDPPOptions
{
	LDPPOptions();
	//Maneuver routine flag
	int MODE;
	//Maneuver sequence flag
	int IDO;
	//Powered-descent simulation flag
	int I_PD;
	//Descent azimuth flag: 0 = descent azimuth is not specified, 1 = descent azimuth is specified
	int I_AZ;
	//Powered-descent time flag
	int I_TPD;
	//Time for powered descent ignition (GMT)
	double T_PD;
	//Table of threshold times (GMT)
	double TH[4];
	//Number of dwell orbits desired between DOI and powered-descent ignition
	int M;
	//Radius of the desired landing site
	double R_LS;
	//Latitude of the desired landing site
	double Lat_LS;
	//Longitude of the desired landing site
	double Lng_LS;
	//Altitude of point of descent ignition
	double H_DP;
	//Powered-flight arc of descent
	double theta_D;
	//Powered-flight time of descent
	double t_D;
	//Descent azimuth desired
	double azi_nom;
	//Altitude wanted at apsis
	double H_W;
	//Average specific impulse of LM descent engine
	double I_SP;
	//Initial weight of LM
	double W_LM;
	//CSM state vector and time
	EphemerisData sv0;
};

struct LDPPResults
{
	LDPPResults();
	//Delta V vector in LVLH coordinates
	VECTOR3 DeltaV_LVLH[4];
	//Time of each maneuver
	double T_M[4];
	//Number of maneuvers in plan
	int i;
	//Time of PDI (if calculated)
	double t_PDI;
	//Time of touchdown (if calculated)
	double t_Land;
	//Azimuth at landing site
	double azi;
	EphemerisData sv_before[4];
	VECTOR3 V_after[4];
};

class LDPP : public RTCCModule
{
public:
	LDPP(RTCC *r);
	void Init(const LDPPOptions &in);
	int LDPPMain(LDPPResults &out);
protected:
	
	VECTOR3 SAC(int L, double h_W, int J, EphemerisData sv_L);
	void CHAPLA(EphemerisData sv_L, int IWA, int IGO, int &I, double &t_m, VECTOR3 &DV);
	int LLTPR(double T_H, EphemerisData sv_L, double &t_DOI, double &t_IGN, double &t_TD);
	double ArgLat(VECTOR3 R, VECTOR3 V);
	void CNODE(EphemerisData sv_A, EphemerisData sv_P, double &t_m, VECTOR3 &dV_LVLH);
	//Subroutine that iterates to find an upcoming apsis point
	EphemerisData STAP(EphemerisData sv0, bool &error);
	//Subroutine that iterates to find a specified radius in a given orbit
	bool STCIR(EphemerisData sv0, double h_W, bool ca_flag, EphemerisData &sv_out);
	EphemerisData TIMA(EphemerisData sv0, double u, bool &error);
	//Add a LVLH Delta V vector to state
	EphemerisData APPLY(EphemerisData sv0, VECTOR3 dV_LVLH);
	VECTOR3 LATLON(double GMT);
	//Utility functions
	EphemerisData PMMLAEG(EphemerisData sv0, int opt, double param, bool &error, double DN = 0.0);
	bool oneclickcoast(VECTOR3 R0, VECTOR3 V0, double gmt0, double dt, VECTOR3 &R1, VECTOR3 &V1);
	EphemerisData PositionMatch(EphemerisData sv_A, EphemerisData sv_P, double mu);
	double P29TimeOfLongitude(VECTOR3 R0, VECTOR3 V0, double GMT, double phi_d);

	double mu;
	OBJHANDLE hMoon;
	//Number of the plane-change maneuver
	int I_PC;
	//Number of maneuver
	int i;
	int IRUT;
	//Closest approach to landing site in CHAPLA
	double GMT_LS_CA;

	double t_M[4];
	VECTOR3 DeltaV_LVLH[4];
	double deltaw_s, u_man, deltaw;

	//State vector index
	//0-3 = maneuvers
	//0 = before, 1 = after
	EphemerisData LDPP_SV_E[4][2];
	EphemerisData sv_CSM, sv_V, sv_LM;

	LDPPOptions opt;

	//Angular iteration tolerance
	static const double zeta_theta;
	//time iteration tolerance
	static const double zeta_t;
};