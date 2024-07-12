/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Lunar Descent Planning Processor

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

#include "Orbitersdk.h"
#include "OrbMech.h"
#include "LMGuidanceSim.h"
#include "rtcc.h"
#include "LDPP.h"

LDPPOptions::LDPPOptions()
{
	MODE = 0;
	IDO = 0;
	I_PD = 0;
	I_AZ = 0;
	I_TPD = 0;
	T_PD = 0.0;
	M = 0;
	R_LS = 0.0;
	Lat_LS = 0.0;
	Lng_LS = 0.0;
	H_DP = 0.0;
	theta_D = 0.0;
	t_D = 0.0;
	azi_nom = 0.0;
	H_W = 0.0;
	I_SP = 0.0;
	W_LM = 0.0;
	for (int ii = 0;ii < 4;ii++)
	{
		TH[ii] = 0.0;
	}
}

LDPPResults::LDPPResults()
{
	for (int ii = 0;ii < 4;ii++)
	{
		DeltaV_LVLH[ii] = _V(0, 0, 0);
		T_M[ii] = 0.0;
	}
	i = 0;
	t_PDI = 0.0;
	t_Land = 0.0;
	azi = 0.0;
}

const double LDPP::zeta_theta = 0.00613*RAD; //0.1NM crossrange
const double LDPP::zeta_t = 0.01;

LDPP::LDPP(RTCC *r) : RTCCModule(r)
{
	mu = OrbMech::mu_Moon;
	I_PC = 0;
	i = 0;
	for (int ii = 0;ii < 4;ii++)
	{
		t_M[ii] = 0.0;
		DeltaV_LVLH[ii] = _V(0, 0, 0);
	}
	hMoon = oapiGetObjectByName("Moon");
}

void LDPP::Init(const LDPPOptions &in)
{
	opt.azi_nom = in.azi_nom;
	opt.H_DP = in.H_DP;
	opt.H_W = in.H_W;
	opt.IDO = in.IDO;
	opt.I_AZ = in.I_AZ;
	opt.I_PD = in.I_PD;
	opt.I_TPD = in.I_TPD;
	opt.Lat_LS = in.Lat_LS;
	opt.Lng_LS = in.Lng_LS;
	opt.M = in.M;
	opt.MODE = in.MODE;
	opt.R_LS = in.R_LS;
	opt.sv0 = in.sv0;
	for (int ii = 0;ii < 4;ii++)
	{
		opt.TH[ii] = in.TH[ii];
	}
	opt.theta_D = in.theta_D;
	opt.t_D = in.t_D;
	opt.T_PD = in.T_PD;
}

//All based on MSC memo 68-FM-23
int LDPP::LDPPMain(LDPPResults &out)
{
	double dt, t_DOI, t_IGN, U_H_DOI, U_OC, GMT, DU_1, DU_2, t_LS, U_LS, t_D, xi, t_PC, t_H_DOI, U_DOI, P_L, DU, T_GO, U_A, DR, U_CSM, t_L;
	VECTOR3 DV, RR_LS, RR_CSM, VV_CSM, c, HH_CSM, rr_LS, R_P, DV_apo, RR_LM, VV_LM, HH_LM, DV_aapo;
	int nn;
	bool error;

	GMT_LS_CA = 0.0;
	U_CSM = U_LS = t_L = t_IGN = 0.0;
	nn = 0;

	//Page 1
	sv_CSM = opt.sv0;

	I_PC = 0;
	i = 1;
	IRUT = 0;

	sv_V = sv_CSM;

	t_H_DOI = opt.TH[3];

	if (opt.MODE != 2)
	{
		goto LDPP_29_1;
	}

	//Not found in document
	if (opt.IDO > 0)
	{
		goto LDPP_3_1;
	}
	//Page 2
	sv_CSM = PMMLAEG(sv_CSM, 0, t_H_DOI, error);
	U_H_DOI = ArgLat(sv_CSM.R, sv_CSM.V);
	U_OC = U_H_DOI + PI2 * (double)opt.M;

	if (opt.M > 0)
	{
		sv_CSM = PMMLAEG(sv_CSM, 2, U_OC, error);
		if (error)
		{
			return 1;
		}
	}
	
	do
	{
		//Find GMT at landing site
		t_LS = P29TimeOfLongitude(sv_CSM.R, sv_CSM.V, sv_CSM.GMT, opt.Lng_LS);
		//Update CSM state vector to time
		sv_CSM = pRTCC->coast(sv_CSM, t_LS - sv_CSM.GMT);
		U_CSM = ArgLat(sv_CSM.R, sv_CSM.V);
		if (U_CSM < U_OC)
		{
			U_CSM += PI2;
		}
		while (U_CSM < U_LS)
		{
			U_CSM += PI2;
		}
		U_LS = U_CSM;
		DU_1 = U_LS - U_H_DOI;
		DU_2 = PI2 * (double)opt.M + PI + opt.theta_D;
		if (DU_1 < DU_2)
		{
			t_D = t_LS + 20.0*60.0;
			dt = t_D - sv_CSM.GMT;
			sv_CSM = pRTCC->coast(sv_CSM, dt);
		}
	} while (DU_1 < DU_2);

	//Page 3
LDPP_3_1:
	if (opt.MODE > 1)
	{
		goto LDPP_6_1;
	}
	if (opt.IDO >= 2)
	{
		goto LDPP_34_1;
	}
	I_PC = 1;
	CHAPLA(sv_CSM, opt.I_AZ, 0, i, t_PC, DV_apo);
LDPP_3_2:
	IRUT = 1;
	if (opt.IDO > 0)
	{
		//TBD: Maybe has to be moved to 10_2 (but probably not)
		t_M[i - 1] = t_PC;
		goto LDPP_10_2;
	}
	else if (opt.IDO == 0)
	{
		goto LDPP_5_1;
	}
LDPP_3_3:
	dt = t_PC - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	LDPP_SV_E[i - 1][0] = sv_CSM;
LDPP_4_1:
	sv_CSM = APPLY(sv_CSM, DV_apo);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	t_M[i - 1] = t_PC;
	DeltaV_LVLH[i - 1] = DV_apo;
	i++;
	goto LDPP_9_1;

LDPP_5_1:
	dt = t_PC - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	DV = SAC(2, 0, 1, sv_CSM);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	DV_apo = DV + DV_apo;

	goto LDPP_4_1;
LDPP_6_1:
	if (opt.MODE < 4)
	{
		goto LDPP_10_1;
	}
	else if (opt.MODE >= 5)
	{
		goto LDPP_19_1;
	}
LDPP_6_2:
	if (LLTPR(opt.TH[i - 1], sv_CSM, t_DOI, t_IGN, t_L))
	{
		return 2;
	}
	dt = t_DOI - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	sv_LM = sv_CSM;
	//Page 7
	LDPP_SV_E[i - 1][0] = sv_LM;
	DV = SAC(1, opt.H_DP, 0, sv_LM);
	sv_LM = APPLY(sv_LM, DV);
	LDPP_SV_E[i - 1][1] = sv_LM;
	t_M[i - 1] = t_DOI;
	DeltaV_LVLH[i - 1] = DV;
	//Page 8
	dt = t_IGN - sv_LM.GMT;
	sv_LM = pRTCC->coast(sv_LM, dt);

	if (IRUT <= 0)
	{
		goto LDPP_32_1;
	}

	GMT = t_IGN + opt.t_D;
	dt = GMT - sv_LM.GMT;
	sv_LM = pRTCC->coast(sv_LM, dt);
	RR_LM = unit(sv_LM.R);
	VV_LM = unit(sv_LM.V);
	HH_LM = unit(crossp(RR_LM, VV_LM));
	RR_LS = LATLON(GMT);

	goto LDPP_18_1;
LDPP_9_2:
	if (I_PC >= 3)
	{
		LDPP_SV_E[i - 1][0] = sv_CSM;
		LDPP_SV_E[i - 1][1] = sv_CSM;
		if (t_M[i - 2] > opt.TH[i - 1])
		{
			opt.TH[i - 1] = t_M[i - 2];
		}
		t_M[i - 1] = opt.TH[i - 1];
		i++;
		goto LDPP_9_3;
	}
LDPP_9_1:
	if (opt.MODE >= 7)
	{
		goto LDPP_30_1;
	}
LDPP_9_3:
	if (t_M[i - 2] > t_H_DOI)
	{
		t_H_DOI = t_M[i - 2];
	}
	dt = t_H_DOI - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	goto LDPP_6_2;

LDPP_10_1:
	if (opt.MODE != 3)
	{
		goto LDPP_14_1;
	}
	if (opt.IDO >= 0)
	{
		dt = opt.TH[i - 1] - sv_CSM.GMT;
		sv_CSM = pRTCC->coast(sv_CSM, dt);
		sv_CSM = STAP(sv_CSM, error);
		if (error)
		{
			return 1;
		}
		t_M[i - 1] = sv_CSM.GMT;
	}
	else
	{
		t_M[i - 1] = opt.TH[i - 1];
	LDPP_10_2:
		dt = t_M[i - 1] - sv_CSM.GMT;
		sv_CSM = pRTCC->coast(sv_CSM, dt);
	}
	DV = SAC(2, opt.H_W, 0, sv_CSM);
	//Page 11
	LDPP_SV_E[i - 1][0] = sv_CSM;
	if (opt.MODE < 3)
	{
		DV = DV + DV_apo;
	}
	sv_CSM = APPLY(sv_CSM, DV);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	DeltaV_LVLH[i - 1] = DV;
	//Page 12
	i++;
	if (t_M[i - 2] > opt.TH[i - 1])
	{
		opt.TH[i - 1] = t_M[i - 2];
	}
	dt = opt.TH[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	do
	{
		sv_CSM = STAP(sv_CSM, error);
		if (error)
		{
			return 1;
		}
	} while (abs(length(sv_CSM.R) - (opt.R_LS + opt.H_W)) > 2.0*1852.0);
	//Page 13
	t_M[i - 1] = sv_CSM.GMT;
	LDPP_SV_E[i - 1][0] = sv_CSM;
	DV = SAC(1, 0, 1, sv_CSM);
	sv_CSM = APPLY(sv_CSM, DV);
LDPP_13_2:
	LDPP_SV_E[i - 1][1] = sv_CSM;
LDPP_13_3:
	DeltaV_LVLH[i - 1] = DV;
	i++;
	goto LDPP_9_1;
LDPP_14_1:
	if (opt.IDO <= 0)
	{
		goto LDPP_15_1;
	}
	dt = opt.TH[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	if (STCIR(sv_CSM, opt.H_W, true, sv_CSM))
	{
		return 1;
	}
	t_M[i - 1] = sv_CSM.GMT;
	LDPP_SV_E[i - 1][0] = sv_CSM;
	DV = SAC(2, 0, 1, sv_CSM);
	sv_CSM = APPLY(sv_CSM, DV);
	goto LDPP_13_2;
	//Page 15
LDPP_15_1:
	U_DOI = U_LS - opt.theta_D - PI - PI2 * (double)opt.M;
	P_L = OrbMech::period(sv_CSM.R, sv_CSM.V, mu);
	U_OC = U_DOI - PI - PI2 * trunc((t_H_DOI - opt.TH[i - 1]) / P_L);
	//TBD
	U_OC -= PI2;
LDPP_15_2:
	sv_CSM = TIMA(sv_CSM, U_OC, error);
	if (error)
	{
		return 1;
	}
	t_M[i - 1] = sv_CSM.GMT;
	LDPP_SV_E[i - 1][0] = sv_CSM;
	DV = SAC(1, opt.H_W, 0, sv_CSM);
	//Page 16
	sv_CSM = APPLY(sv_CSM, DV);
	if (LLTPR(opt.TH[i - 1], sv_CSM, t_DOI, t_IGN, t_L))
	{
		return 2;
	}
	LDPP_SV_E[i - 1][1] = sv_CSM;
	P_L = OrbMech::period(sv_CSM.R, sv_CSM.V, mu);
	t_D = t_M[i - 1] + P_L * trunc((t_H_DOI - opt.TH[i - 1]) / P_L);
	dt = t_D - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	//Page 17
	sv_CSM = STAP(sv_CSM, error);
	if (error)
	{
		return 1;
	}
	U_A = ArgLat(sv_CSM.R, sv_CSM.V);
	DU = U_A - U_DOI;
	if (abs(DU) <= zeta_theta)
	{
		goto LDPP_13_3;
	}
	sv_CSM = sv_V;
	U_OC = U_OC - DU;
	//TBD
	while (U_OC < 0)
	{
		U_OC += PI2;
	}
	dt = t_M[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	goto LDPP_15_2;
	//Page 18
LDPP_18_1:
	rr_LS = unit(RR_LS);
	c = unit(crossp(rr_LS, HH_LM));
	R_P = unit(crossp(HH_LM, c));
	xi = acos(dotp(R_P, rr_LS));
	nn++;
	if (abs(xi) <= zeta_theta || nn > 10)
	{
		goto LDPP_32_1;
	}
	CHAPLA(sv_CSM, opt.I_AZ, 0, i, t_PC, DV_apo);
	if (opt.MODE < 5)
	{
		if (opt.MODE != 1)
		{
			goto LDPP_3_2;
		}
		else
		{
			if (opt.IDO >= 2)
			{
				goto LDPP_35_2;
			}
			else
			{
				goto LDPP_3_2;
			}
		}
	}
	else
	{
		goto LDPP_23_1;
	}
LDPP_19_1:
	IRUT = 1;
	if (opt.IDO < 0)
	{
		LDPP_SV_E[i - 1][0] = sv_CSM;
		LDPP_SV_E[i - 1][1] = sv_CSM;
		t_M[i - 1] = opt.TH[i - 1];
		i++;
		T_GO = opt.TH[i - 1];
		I_PC = 1;
	}
	else if (opt.IDO == 0)
	{
		I_PC = 2;
		T_GO = opt.TH[i - 1];
	}
	else
	{
		I_PC = 3;
		T_GO = opt.TH[i - 1];
	}
LDPP_19_2:
	dt = T_GO - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	sv_CSM = STAP(sv_CSM, error);
	if (error)
	{
		return 1;
	}
	t_M[i - 1] = sv_CSM.GMT;
	//Page 20
	DV = SAC(1, opt.H_W, 0, sv_CSM);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	sv_CSM = APPLY(sv_CSM, DV);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	DeltaV_LVLH[i - 1] = DV;
	//Page 21
	i++;
	if (t_M[i - 2] > opt.TH[i - 1])
	{
		opt.TH[i - 1] = t_M[i - 2];
	}
	if (I_PC == 2)
	{
		dt = opt.TH[i - 1] - sv_CSM.GMT;
		sv_CSM = pRTCC->coast(sv_CSM, dt);
		LDPP_SV_E[i - 1][0] = sv_CSM;
		LDPP_SV_E[i - 1][1] = sv_CSM;
		t_M[i - 1] = opt.TH[i - 1];
		i++;
	}
LDPP_21_2:
	dt = opt.TH[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	do
	{
		sv_CSM = STAP(sv_CSM, error);
		if (error)
		{
			return 1;
		}
		t_M[i - 1] = sv_CSM.GMT;
		DR = abs(length(sv_CSM.R) - opt.R_LS - opt.H_W);
	} while (DR > 1000.0*0.3048);
	//Page 22
	DV = SAC(1, 0, 1, sv_CSM);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	sv_CSM = APPLY(sv_CSM, DV);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	DeltaV_LVLH[i - 1] = DV;
	i++;
	goto LDPP_9_2;
LDPP_23_1:
	i = 1;
	if (opt.IDO > 0)
	{
		goto LDPP_25_2;
	}
	else if (opt.IDO == 0)
	{
		goto LDPP_27_1;
	}
	sv_CSM = sv_V;
	dt = t_PC - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	//Page 24
	sv_CSM = APPLY(sv_CSM, DV_apo);
	t_M[i - 1] = t_PC;
	DeltaV_LVLH[i - 1] = DV_apo;
	LDPP_SV_E[i - 1][1] = sv_CSM;
	i++;
	//Page 25
	if (t_M[i - 2] > opt.TH[i - 1])
	{
		opt.TH[i - 1] = t_M[i - 2];
	}
	T_GO = opt.TH[i - 1];
	goto LDPP_19_2;
LDPP_25_2:
	i = 2;
	sv_CSM = LDPP_SV_E[1][1];
	dt = t_PC - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	//Page 26
	i++;
	LDPP_SV_E[i - 1][0] = sv_CSM;
	sv_CSM = APPLY(sv_CSM, DV_apo);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	t_M[i - 1] = t_PC;
	DeltaV_LVLH[i - 1] = DV_apo;
	i++;
	goto LDPP_9_1;
	//Page 27
LDPP_27_1:
	sv_CSM = LDPP_SV_E[0][1];
	i = 2;
	dt = t_PC - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	sv_CSM = APPLY(sv_CSM, DV_apo);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	//Page 28
	t_M[i - 1] = t_PC;
	DeltaV_LVLH[i - 1] = DV_apo;
	i++;
	if (t_M[i - 2] > opt.TH[i - 1])
	{
		opt.TH[i - 1] = t_M[i - 2];
	}
	goto LDPP_21_2;
LDPP_29_1:
	if (opt.MODE == 6)
	{
		goto LDPP_32_2;
	}

	dt = opt.TH[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	if (opt.MODE < 7)
	{
		goto LDPP_3_1;
	}

	I_PC = 1;
	IRUT = 1;

	CHAPLA(sv_CSM, opt.I_AZ, 0, i, t_PC, DV_apo);

	goto LDPP_3_3;
	//Page 30
LDPP_30_1:
	if (t_M[i - 2] > opt.TH[i - 1])
	{
		opt.TH[i - 1] = t_M[i - 2];
	}
	dt = opt.TH[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	GMT = P29TimeOfLongitude(sv_CSM.R, sv_CSM.V, sv_CSM.GMT, opt.Lng_LS);
	dt = GMT - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	RR_LS = LATLON(GMT);
	RR_CSM = unit(sv_CSM.R);
	VV_CSM = unit(sv_CSM.V);
	HH_CSM = unit(crossp(RR_CSM, VV_CSM));
	//Page 31
	rr_LS = unit(RR_LS);
	c = unit(crossp(rr_LS, HH_CSM));
	R_P = unit(crossp(HH_CSM, c));
	xi = acos(dotp(R_P, rr_LS));
	if (abs(xi) <= zeta_theta)
	{
		goto LDPP_33_1;
	}
	CHAPLA(sv_CSM, opt.I_AZ, 0, i, t_PC, DV_apo);
	i = 1;
	goto LDPP_3_3;
	//Page 32
LDPP_32_1:
	opt.W_LM = opt.W_LM / exp(length(DeltaV_LVLH[i - 1]) / opt.I_SP);
	if (opt.I_PD <= 0)
	{
	LDPP_32_2:
		if (opt.I_TPD <= 0)
		{

		}
		else
		{

		}
		//TBD: Powered Descent Simulation
	}
LDPP_33_1:
	//Compute display quantities and output
	for (int ii = 0;ii < 4;ii++)
	{
		out.DeltaV_LVLH[ii] = DeltaV_LVLH[ii];
		out.T_M[ii] = t_M[ii];
		out.sv_before[ii] = LDPP_SV_E[ii][0];
		out.V_after[ii] = LDPP_SV_E[ii][1].V;
	}
	if (opt.MODE == 7)
	{
		i = i - 1;
		out.azi = opt.azi_nom;
		out.t_Land = 0.0;
		out.t_PDI = 0.0;
		out.T_M[1] = GMT_LS_CA; //Show as second maneuver time
	}
	else
	{
		//For now, from the old DOI calculation
		double dt4;
		OrbMech::time_theta(sv_LM.R, sv_LM.V, opt.theta_D - 15.0*RAD, mu, dt4);
		out.azi = opt.azi_nom;
		out.t_Land = t_L + dt4;
		out.t_PDI = t_IGN + dt4;
	}
	out.i = i;
	return 0;

	//Page 34
LDPP_34_1:
	if (opt.IDO < 3)
	{
		if (STCIR(sv_CSM, opt.H_W, true, sv_CSM))
		{
			return 1;
		}
		t_M[i - 1] = sv_CSM.GMT;
	}
	else
	{
		t_M[i - 1] = opt.TH[i - 1];
	}
	deltaw_s = 0.0;
	IRUT = 1;
	dt = t_M[i - 1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	u_man = ArgLat(sv_CSM.R, sv_CSM.V);
	LDPP_SV_E[i - 1][0] = sv_CSM;
	DV = SAC(1, 0, 1, sv_CSM);
	DV_aapo = DV;

	goto LDPP_35_1;

	//Page 35
LDPP_35_2:
	deltaw_s = deltaw;
	DV = DV_apo + DV_aapo;
LDPP_35_1:
	sv_CSM = APPLY(sv_CSM, DV);
	LDPP_SV_E[i - 1][1] = sv_CSM;
	DeltaV_LVLH[i - 1] = DV;
	i++;
	if (t_M[i - 2] > t_H_DOI)
	{
		t_H_DOI = t_M[i - 2];
	}
	dt = t_H_DOI - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	goto LDPP_6_2;
}

VECTOR3 LDPP::SAC(int L, double h_W, int J, EphemerisData sv_L)
{
	EphemerisData sv_L2;
	MATRIX3 Q_Xx;
	VECTOR3 DV;
	double u_b, u_d, R_A, R_A_apo, r, a, v, n, DN, dt, u_c, dr;
	int nn;

	Q_Xx = OrbMech::LVLH_Matrix(sv_L.R, sv_L.V);

	r = length(sv_L.R);
	u_b = ArgLat(sv_L.R, sv_L.V);
	u_d = u_b + PI;
	if (u_d >= PI2)
	{
		u_d -= PI2;
		DN = 1.0;
	}
	else
	{
		DN = 0.0;
	}
	if (J == 0)
	{
		R_A = opt.R_LS + h_W;
	}
	else
	{
		R_A = length(sv_L.R);
	}
	R_A_apo = R_A;
	do
	{
		a = (R_A_apo + r) / 2.0;
		v = sqrt(-mu / a + 2.0*mu / r);

		sv_L2 = sv_L;
		sv_L2.V = tmul(Q_Xx, _V(v, 0, 0));
		DV = sv_L2.V - sv_L.V;

		n = PI2 / OrbMech::period(sv_L2.R, sv_L2.V, mu);
		u_c = u_b;
		nn = 0;
		do
		{
			if (nn == 0)
			{
				dt = (u_d - u_c + PI2 * DN) / n;
			}
			else
			{
				dt = (u_d - u_c) / n;
			}
			
			sv_L2 = pRTCC->coast(sv_L2, dt);
			u_c = ArgLat(sv_L2.R, sv_L2.V);
			nn++;
		} while (abs(dt) > zeta_t);

		dr = R_A - length(sv_L2.R);
		R_A_apo += dr;
	} while (abs(dr) > 1.0);
	return mul(OrbMech::LVLH_Matrix(sv_L.R, sv_L.V), DV);
}

double LDPP::ArgLat(VECTOR3 R, VECTOR3 V)
{
	VECTOR3 H, E, N;
	double TA, w, u;
	H = crossp(R, V);
	E = crossp(V, H) / mu;
	N = crossp(_V(0, 0, 1), H);
	TA = acos2(dotp(unit(E), unit(R)));
	if (dotp(R, V) < 0)
	{
		TA = PI2 - TA;
	}
	w = acos2(dotp(unit(N), unit(E)));
	if (E.z < 0)
	{
		w = PI2 - w;
	}
	u = TA + w;
	if (u > PI2)u -= PI2;

	return u;
}

VECTOR3 LDPP::LATLON(double GMT)
{
	VECTOR3 R_LS;

	pRTCC->ELVCNV(OrbMech::r_from_latlong(opt.Lat_LS, opt.Lng_LS, opt.R_LS), GMT, 1, RTCC_COORDINATES_MCT, RTCC_COORDINATES_MCI, R_LS);

	return R_LS;
}

int LDPP::LLTPR(double T_H, EphemerisData sv_L, double &t_DOI, double &t_IGN, double &t_TD)
{
	EphemerisData sv_L0;
	VECTOR3 H_c, h_c, C, c, V_H, R_PP, V_PP, d, R_ppu, D_L, RR_LS, rr_LS, h_c2;
	double t, dt, R_D, S_w, R_p, R_a, a_D, t_H, t_L, cc, eps_R, alpha, E_I, t_x;
	int N, S;

	t = sv_L.GMT;
	dt = T_H - t;
	N = 0;
	S_w = 1.0;
	R_D = opt.R_LS + opt.H_DP;

	sv_L0 = sv_L;

	do
	{
		N = N + 1;
		if (N > 15)
		{
			return 1;
		}

		t = t + S_w * dt;
		sv_L = pRTCC->coast(sv_L, t - sv_L.GMT);

		S = 0;
		R_p = R_D;
		R_a = length(sv_L.R);
		H_c = crossp(sv_L.R, sv_L.V);
		h_c = unit(H_c);
		a_D = mu * R_a / (2.0*mu - R_a * pow(length(sv_L.V), 2));

		t_H = (PI2*(double)opt.M + PI)*sqrt(pow(R_a + R_p, 3) / (8.0*mu));

	LDPP_LLTPR_2_2:
		C = crossp(h_c, sv_L.R);
		c = unit(C);
		V_H = c * sqrt(2.0*mu*R_p / (R_a*(R_p + R_a)));
		sv_L.V = V_H;

		oneclickcoast(sv_L.R, sv_L.V, sv_L.GMT, t_H, R_PP, V_PP);

		//Not in LDPP document
		double dt_peri = OrbMech::timetoperi(R_PP, V_PP, mu);
		oneclickcoast(R_PP, V_PP, sv_L.GMT + t_H, dt_peri, R_PP, V_PP);

		if (S <= 0)
		{
			R_p = 2.0*R_D - length(R_PP);
			S = 1;
			goto LDPP_LLTPR_2_2;
		}

		t_H += dt_peri;
		t_x = t + t_H;

		//Not in LDPP document
		h_c2 = unit(crossp(R_PP, V_PP));

		t_L = t_x + opt.t_D;
		R_ppu = unit(R_PP);
		d = unit(crossp(h_c2, R_ppu));
		D_L = R_ppu * cos(opt.theta_D) + d * sin(opt.theta_D);

		RR_LS = LATLON(t_L);

		cc = dotp(h_c2, RR_LS);
		RR_LS = RR_LS - h_c2 * cc;
		rr_LS = unit(RR_LS);
		eps_R = length(D_L - rr_LS);

		if (eps_R <= zeta_theta)
		{
			t_DOI = t;
		}
		else
		{
			alpha = acos(dotp(D_L, rr_LS));
			E_I = dotp(h_c2, crossp(D_L, rr_LS));
			if (E_I >= 0)
			{
				S_w = 1.0;
			}
			else
			{
				if (N == 1)
				{
					alpha = PI2 - alpha;
					S_w = 1.0;
				}
				else
				{
					S_w = -1.0;
				}
			}
			dt = alpha * sqrt(pow(a_D, 3) / mu);

			sv_L = sv_L0;
		}
	} while (eps_R > zeta_theta);

	t_IGN = t_x;
	t_TD = t_L;
	return 0;
}

void LDPP::CHAPLA(EphemerisData sv_L, int IWA, int IGO, int &I, double &t_m, VECTOR3 &DV)
{
	//INPUTS:
	//sv_L: input state vector
	//IWA: 0 = the azimuth is not specified, 1 = the azimuth is specified
	//IGO: 0 = compute plane change as if it were the only maneuver being done, 1 = compute plane change as part of maneuver sequence
	//I: number of maneuvers in sequence
	//I_PC: number of the plane-change maneuver

	//OUTPUTS:
	//t_m: maneuver time
	//DV: maneuver delta V

	OELEMENTS coe_b, coe_a;
	EphemerisData sv_A, sv_P;
	MATRIX3 Rot;
	VECTOR3 R_TH, V_TH, RR_LS, R_L, V_L, rr_LS, rr_L, vv_L, H_L, h_L, Q, c, R_p, rr_p, H_d, h_d, R_LS_equ;
	double dt1, dt2, GMT_TH, GMT_LS, n_L, theta, s1, s2, dt3;
	int ii;

	dt1 = opt.TH[3] - sv_L.GMT;
	oneclickcoast(sv_L.R, sv_L.V, sv_L.GMT, dt1, R_TH, V_TH);
	GMT_TH = sv_L.GMT + dt1;
	GMT_LS = P29TimeOfLongitude(R_TH, V_TH, GMT_TH, opt.Lng_LS);
	dt2 = GMT_LS - GMT_TH;
	oneclickcoast(R_TH, V_TH, GMT_TH, dt2, R_L, V_L);
	n_L = PI2 / OrbMech::period(R_L, V_L, mu);
	R_LS_equ = OrbMech::r_from_latlong(opt.Lat_LS, opt.Lng_LS, opt.R_LS);

	do
	{
		RR_LS = LATLON(GMT_LS);
		rr_LS = unit(RR_LS);

		rr_L = unit(R_L);
		vv_L = unit(V_L);
		H_L = crossp(rr_L, vv_L);
		h_L = unit(H_L);

		Q = crossp(rr_LS, h_L);
		c = unit(Q);
		R_p = crossp(h_L, c);
		rr_p = unit(R_p);

		theta = acos(dotp(rr_p, rr_L));
		H_d = crossp(rr_L, rr_p);
		h_d = unit(H_d);
		s1 = h_d.z / abs(h_d.z);
		s2 = h_L.z / abs(h_L.z);
		theta = theta * s1 / s2;
		dt3 = theta / n_L;
		if (abs(dt3) > zeta_t)
		{
			oneclickcoast(R_L, V_L, GMT_LS, dt3, R_L, V_L);
			GMT_LS = GMT_LS + dt3;
		}
	} while (abs(dt3) > zeta_t);

	//Save for display
	GMT_LS_CA = GMT_LS;

	if (opt.IDO >= 2)
	{
		goto LDPP_CHAPLA_9_1;
	}
	
	pRTCC->ELVCNV(GMT_LS, RTCC_COORDINATES_MCT, RTCC_COORDINATES_MCI, Rot);

	R_L = tmul(Rot, R_L);
	V_L = tmul(Rot, V_L);

	VECTOR3 R_J, V_J;
	double rmag, vmag, rtasc, decl, fpav, az;//, u_w;

	coe_b = OrbMech::coe_from_sv(R_L, V_L, mu);
	OrbMech::rv_from_adbar(R_L, V_L, rmag, vmag, rtasc, decl, fpav, az);

	coe_a.e = coe_b.e;
	coe_a.h = coe_b.h;
	coe_a.TA = coe_b.TA;

	if (IWA == 0)
	{
		opt.azi_nom = az;
	}

	OrbMech::adbar_from_rv(rmag, vmag, opt.Lng_LS, opt.Lat_LS, fpav, opt.azi_nom, R_J, V_J);

	/*coe_a.i = acos(sin(opt.azi_nom)*cos(opt.Lat_LS));
	u_w = atan2(sin(opt.Lat_LS)*sin(opt.azi_nom), -cos(coe_a.i)*cos(opt.azi_nom));
	if (u_w < 0) u_w += PI2;
	//This equation is probably wrong
	coe_a.RA = opt.Lng_LS - u_w - 2.0*atan(tan(opt.Lat_LS / 2.0)*(sin(0.5*(opt.azi_nom + PI05)) / sin(0.5*(opt.azi_nom - PI05))));
	coe_a.w = coe_b.w - 2.0*atan(tan((coe_a.RA - coe_b.RA) / 2.0)*(sin(0.5*(PI - coe_a.i - coe_b.i)) / sin(0.5*(PI - coe_a.i + coe_b.i))));

	OrbMech::sv_from_coe(coe_a, mu, R_J, V_J);*/
	//U_J = unit(R_J);
	R_J = mul(Rot, R_J);
	V_J = mul(Rot, V_J);

	sv_P.GMT = GMT_LS;
	sv_P.R = R_J;
	sv_P.V = V_J;
	sv_P.RBI = BODY_MOON;

	sv_A = sv_L;

	bool error;

	if (IGO > 0)
	{
		ii = I;
		CELEMENTS elem_A, elem_P;

		do
		{
			sv_A = PMMLAEG(sv_A, 0, t_M[ii - 1], error);
			sv_P = PMMLAEG(sv_P, 0, t_M[ii - 1], error);
			sv_A = LDPP_SV_E[ii - 1][0];
			elem_A = OrbMech::GIMIKC(sv_A.R, sv_A.V, mu);
			elem_P = OrbMech::GIMIKC(sv_P.R, sv_P.V, mu);
			elem_P.a = elem_A.a;
			elem_P.e = elem_A.e;
			OrbMech::GIMKIC(elem_P, mu, sv_P.R, sv_P.V);
			sv_P.GMT = sv_A.GMT;
			sv_P.RBI = BODY_MOON;
			if (ii > I_PC)
			{
				ii--;
				continue;
			}
			else
			{
				break;
			}
		} while (ii + 1 > I_PC);
	}
	else
	{
		ii = 1;
	}

	I = ii;
	sv_CSM = sv_A;
	
	sv_A = PMMLAEG(sv_A, 0, opt.TH[ii - 1], error);
	sv_P = PositionMatch(sv_P, sv_A, mu);

	//Common node
	CNODE(sv_A, sv_P, t_m, DV);

	return;

LDPP_CHAPLA_9_1:

	double u_ca = ArgLat(R_L, V_L);
	double nu = acos(dotp(rr_LS, rr_p));
	double delta_nu = acos(dotp(rr_LS, h_L));
	deltaw = sin(nu) / (cos(nu)*cos(u_ca - u_man));
	deltaw = deltaw * cos(delta_nu) / abs(cos(delta_nu));
	I = 1;
	deltaw = deltaw_s + deltaw;

	sv_CSM = LDPP_SV_E[0][0];

	double dv_pc = 2.0*length(V_L)*sin(abs(deltaw) / 2.0);
	double dv_z = dv_pc * cos(abs(deltaw) / 2.0);
	double dv_r = 0.0;
	double dv_h = -dv_pc * sin(abs(deltaw) / 2.0);
	double u_dum = u_ca - u_man;
	double u_sign = u_dum - PI2 * trunc(u_dum / PI2);
	deltaw_s = deltaw;
	if (deltaw_s - PI05 < 0)
	{
		dv_z = -dv_z;
	}
	if (u_sign + PI >= 0)
	{
		dv_z = -dv_z;
	}

	DV.x = dv_h;
	DV.y = dv_z;
	DV.z = dv_r;
}

EphemerisData LDPP::APPLY(EphemerisData sv0, VECTOR3 dV_LVLH)
{
	sv0.V += tmul(OrbMech::LVLH_Matrix(sv0.R, sv0.V), dV_LVLH);
	return sv0;
}

EphemerisData LDPP::STAP(EphemerisData sv0, bool &error)
{
	CELEMENTS elem = OrbMech::GIMIKC(sv0.R, sv0.V, mu);
	double DN = 0.0;
	//Close to apsis

	if (elem.l < 0.001*RAD || elem.l > PI2 - 0.001*RAD || abs(elem.l - PI) < 0.001*RAD)
	{
		return PMMLAEG(sv0, 3, 0.0, error, 0.5);
	}

	if (elem.l < PI)
	{
		//Apoapsis
		return PMMLAEG(sv0, 1, PI, error, DN);
	}
	else
	{
		//Periapsis
		return PMMLAEG(sv0, 1, 0, error, DN);
	}
}

bool LDPP::STCIR(EphemerisData sv0, double h_W, bool ca_flag, EphemerisData& sv_out)
{
	CELEMENTS coe;
	double r_H, f_cir, f_I, g_dot, eta_I, dt, cos_f_cf, f_F, f_cf, eta_F, temp;

	r_H = opt.R_LS + h_W;
	g_dot = 0.0;
	eta_I = PI2 / OrbMech::period(sv0.R, sv0.V, mu);

	coe = OrbMech::GIMIKC(sv0.R, sv0.V, mu);
	temp = (coe.a*(1.0 - coe.e * coe.e) - r_H) / (coe.e*r_H);
	if (abs(temp) > 1.0)
	{
		if (ca_flag)
		{
			if (temp > 1.0)
			{
				temp = 1.0;
			}
			else if (temp < -1.0)
			{
				temp = -1.0;
			}
		}
		else
		{
			sv_out = sv0;
			return true;
		}
	}
	f_cir = acos(temp);
	f_I = OrbMech::MeanToTrueAnomaly(coe.l, coe.e);
	if (f_cir - f_I >= 0)
	{
		dt = (f_cir - f_I) / (g_dot + eta_I);
	}
	else
	{
		if (PI2 - f_cir - f_I > 0)
		{
			dt = (PI2 - f_cir - f_I) / (g_dot + eta_I);
		}
		else
		{
			dt = (f_cir + PI2 - f_I) / (g_dot + eta_I);
		}
	}
	do
	{
		sv0 = pRTCC->coast(sv0, dt);
		coe = OrbMech::GIMIKC(sv0.R, sv0.V, mu);
		cos_f_cf = (coe.a*(1.0 - coe.e*coe.e) - r_H) / (coe.e*r_H);

		if (abs(cos_f_cf) > 1.0)
		{
			if (ca_flag)
			{
				if (cos_f_cf > 1.0)
				{
					cos_f_cf = 1.0;
				}
				else if (cos_f_cf < -1.0)
				{
					cos_f_cf = -1.0;
				}
			}
			else
			{
				sv_out = sv0;
				return true;
			}
		}
		f_cf = acos(cos_f_cf);
		f_F = OrbMech::MeanToTrueAnomaly(coe.l, coe.e);
		eta_F = PI2 / OrbMech::period(sv0.R, sv0.V, mu);
		temp = f_cf - f_F;
		if (temp > PI)
		{
			temp -= PI2;
		}
		else if (temp < -PI)
		{
			temp += PI2;
		}
		dt = temp / (g_dot + eta_F);
	} while (abs(dt) > 0.1);

	sv_out = sv0;
	return false;

	/*double dt2, dt21, dt22, r_H;

	r_H = opt.R_LS + h_W;
	dt21 = OrbMech::time_radius_integ(sv0.R, sv0.V, sv0.MJD, r_H, 1.0, hMoon, hMoon);
	dt22 = OrbMech::time_radius_integ(sv0.R, sv0.V, sv0.MJD, r_H, -1.0, hMoon, hMoon);

	if (abs(dt21) > abs(dt22))
	{
		dt2 = dt22;
	}
	else
	{
		dt2 = dt21;
	}

	sv_out = OrbMech::coast(sv0, dt2);
	return false;*/
}

EphemerisData LDPP::TIMA(EphemerisData sv0, double u, bool &error)
{
	double DN = 0.0;

	while (u < 0)
	{
		DN -= 1.0;
		u += PI2;
	}
	while (u > PI2)
	{
		DN += 1.0;
		u -= PI2;
	}

	return PMMLAEG(sv0, 2, u, error, DN);
}

void LDPP::CNODE(EphemerisData sv_A, EphemerisData sv_P, double &t_m, VECTOR3 &dV_LVLH)
{
	CELEMENTS coe_M, coe_T;
	double GMT_CN, U_L, U_U, i_T, i_M, h_T, h_M, cos_dw, DEN, U_CN;
	int ICT = 0;

	GMT_CN = sv_A.GMT;

	do
	{

		if (ICT == 0)
		{
			U_L = ArgLat(sv_A.R, sv_A.V);
			U_U = U_L + PI;
			if (U_U > PI2) U_U -= PI2;
		}

		coe_M = OrbMech::GIMIKC(sv_A.R, sv_A.V, mu);
		coe_T = OrbMech::GIMIKC(sv_P.R, sv_P.V, mu);
		i_T = coe_T.i;
		i_M = coe_M.i;
		h_T = coe_T.h;
		h_M = coe_M.h;

		cos_dw = cos(i_T)*cos(i_M) + sin(i_T)*sin(i_M)*cos(h_M - h_T);
		DEN = cos_dw * cos(i_M) - cos(i_T);
		if (h_T < h_M)
		{
			DEN = -DEN;
		}
		U_CN = atan2(sin(i_T)*sin(i_M)*sin(abs(h_M - h_T)), DEN);

		if (U_L > PI)
		{
			if (U_U <= U_CN)
			{
				U_CN = U_CN + PI;
			}
		}
		else
		{
			if (U_L > U_CN)
			{
				U_CN = U_CN + PI;
			}
		}

		ICT++;

		if (ICT > 3) break;

		bool error;
		sv_A = PMMLAEG(sv_A, 2, U_CN, error);
		if (error)
		{
			return;
		}
		sv_P = PositionMatch(sv_P, sv_A, mu);
		GMT_CN = sv_A.GMT;
	} while (ICT <= 3);

	VECTOR3 DV_Test = mul(OrbMech::LVLH_Matrix(sv_A.R, sv_A.V), sv_P.V - sv_A.V);

	double dw = acos(cos_dw);
	double dv_PC = 2.0*sqrt(mu)*sin(dw / 2.0)*sqrt(2.0 / length(sv_A.R) - 1.0 / coe_M.a);
	VECTOR3 H_P = unit(crossp(sv_P.R, sv_P.V));
	VECTOR3 H_C = unit(crossp(sv_A.R, sv_A.V));
	VECTOR3 K = unit(crossp(H_P, sv_A.R));
	double S = dotp(H_C, K);

	dV_LVLH.x = -dv_PC * sin(0.5*dw);
	dV_LVLH.y = -S * dv_PC*cos(0.5*dw) / abs(S);
	dV_LVLH.z = 0.0;
	t_m = GMT_CN;
}

EphemerisData LDPP::PMMLAEG(EphemerisData sv0, int opt, double param, bool &error, double DN)
{
	SV sv1, sv2;
	EphemerisData sv3;
	double GMTBase;

	GMTBase = pRTCC->GetGMTBase();

	sv1 = pRTCC->ConvertEphemDatatoSV(sv0);

	//For time option, convert GMT to MJD
	if (opt == 0)
	{
		param = OrbMech::MJDfromGET(param, pRTCC->GetGMTBase());
	}

	sv2 = OrbMech::PMMLAEG(pRTCC->SystemParameters.AGCEpoch, sv1, opt, param, error, DN);

	sv3 = pRTCC->ConvertSVtoEphemData(sv2);

	return sv3;
}

bool LDPP::oneclickcoast(VECTOR3 R0, VECTOR3 V0, double gmt0, double dt, VECTOR3 &R1, VECTOR3 &V1)
{
	return OrbMech::oneclickcoast(pRTCC->SystemParameters.AGCEpoch, R0, V0, OrbMech::MJDfromGET(gmt0, pRTCC->GetGMTBase()), dt, R1, V1, hMoon, hMoon);
}

EphemerisData LDPP::PositionMatch(EphemerisData sv_A, EphemerisData sv_P, double mu)
{
	//Wrapper to call PositionMatch with EphemerisData instead of SV
	SV sv_A1, sv_P1, sv_P2;

	sv_A1 = pRTCC->ConvertEphemDatatoSV(sv_A);
	sv_P1 = pRTCC->ConvertEphemDatatoSV(sv_P);

	sv_P2 = OrbMech::PositionMatch(pRTCC->SystemParameters.AGCEpoch, sv_A1, sv_P1, mu);

	return pRTCC->ConvertSVtoEphemData(sv_P2);
}

double LDPP::P29TimeOfLongitude(VECTOR3 R0, VECTOR3 V0, double GMT, double phi_d)
{
	//Wrapper to call P29TimeOfLongitude with GMT instead of MJD
	double GMTBase, MJD, MJD_lng;

	GMTBase = pRTCC->GetGMTBase();
	MJD = OrbMech::MJDfromGET(GMT, GMTBase);

	MJD_lng = OrbMech::P29TimeOfLongitude(pRTCC->SystemParameters.MAT_J2000_BRCS, R0, V0, MJD, hMoon, phi_d);

	return OrbMech::GETfromMJD(MJD_lng, GMTBase);
}