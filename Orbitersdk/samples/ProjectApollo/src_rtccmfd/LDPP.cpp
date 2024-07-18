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
	theta_PDI = 0.0;
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
	for (int ii = 0;ii < 4;ii++)
	{
		t_M[ii] = 0.0;
		DeltaV_LVLH[ii] = _V(0, 0, 0);
	}
	hMoon = oapiGetObjectByName("Moon");
}

void LDPP::Init(const LDPPOptions &in)
{
	opt = in;
}

//All based on MSC memo 68-FM-23
int LDPP::LDPPMain(LDPPResults &out)
{
	switch (opt.MODE)
	{
	case 1:
		Mode1();
		break;
	case 2:
		Mode2();
		break;
	case 3:
		Mode3();
		break;
	case 4:
		Mode4();
		break;
	}

	OutputCalculations();
	out = outp;
	return 0;
}

int LDPP::Mode1()
{
	if (opt.IDO <= 1)
	{
		return Mode1_1();
	}
	else
	{
		return Mode1_2();
	}
}

int LDPP::Mode1_1()
{
	//Mode 1, Sequence 1-3

	VECTOR3 DV, DV_apo;
	double dt, t_PC, xi, TH;
	int iter;
	bool asseq, stop, error;

	if (opt.IDO == 1)
	{
		//ASP, CIA, DOI
		I_PC = 1;
		I_Num = 3;
	}
	else
	{
		//PC + DOI or PCC + DOI
		I_PC = 1;
		I_Num = 2;
	}

	//Input state vector as CSM state vector
	sv_CSM = opt.sv0;

	//Advance to threshold time of first maneuver
	dt = opt.TH[0] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	//Save state vector at threshold time
	sv_V = sv_CSM;

	//Prepare iteration loop
	asseq = false;
	stop = false;
	iter = 0;
	TH = opt.TH[3];

	do
	{
		iter++;
		//Compute plane change alone
		CHAPLA(sv_CSM, opt.I_AZ, asseq, I_Num, TH, t_PC, DV_apo);
		//Advance to t_PC
		dt = t_PC - sv_V.GMT;
		sv_CSM = pRTCC->coast(sv_V, dt);
		//Is the plane change a combination maneuver?
		if (opt.IDO >= 0)
		{
			//Combination maneuver
			if (opt.IDO == 0 || opt.IDO >= 2)
			{
				//Combination plane change and circularization
				DV = SAC(0.0, true, sv_CSM);
			}
			else
			{
				//Combination plane change and height change
				DV = SAC(opt.H_W, false, sv_CSM);
			}
			//Combine plane change and circularization DV
			DV_apo = DV + DV_apo;
		}
		//Apply and save maneuver
		sv_CSM = SaveElements(sv_CSM, 0, DV_apo);
		if (opt.IDO == 1)
		{
			//Circularization at an apsis maneuver
			dt = opt.TH[1] - sv_CSM.GMT;
			sv_CSM = pRTCC->coast(sv_CSM, dt);
			for (int iii = 0; iii < 2; iii++)
			{
				sv_CSM = STAP(sv_CSM, error);
				if (error)
				{
					return 1;
				}
				if (abs(length(sv_CSM.R) - (opt.R_LS + opt.H_W)) < 2.0*1852.0) break;
			}
			//Circularize
			DV_apo = SAC(0.0, true, sv_CSM);
			sv_CSM = SaveElements(sv_CSM, 1, DV_apo);
		}
		//DOI
		DOIManeuver(I_Num - 1);
		//Calculate out of plane error
		xi = OutOfPlaneError(sv_LM);
		if (abs(xi) <= zeta_theta || iter >= 10)
		{
			stop = true;
		}
		else
		{
			//For backwards propagation
			sv_CSM = sv_LM;
			//To ensure it finds the right landing site passage
			TH = t_TD - 1000.0;
			asseq = true;
		}
	} while (stop == false);

	return 0;
}

int LDPP::Mode1_2()
{
	//Mode 1, Sequence 4-5

	VECTOR3 DV, DV_aapo, DV_apo;
	double dt, deltaw_s, xi;
	int iter;
	bool error, stop;
	
	I_PC = 1;
	I_Num = 2;

	//Input state vector as CSM state vector
	sv_CSM = opt.sv0;

	//Advance to threshold time of first maneuver
	dt = opt.TH[0] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	//Take state vector to plane change ignition time
	if (opt.IDO < 3)
	{
		//PCC at altitude
		error = STCIR(sv_CSM, opt.H_W, true, sv_CSM);
		if (error)
		{

		}
	}
	else
	{
		//Threshold time is time of ignition
	}

	//Save state vector at PCC time
	sv_V = sv_CSM;

	//Calculate circularization maneuver
	DV_aapo = SAC(0.0, true, sv_CSM);

	//Prepare iteration
	deltaw_s = 0.0;
	DV_apo = _V(0, 0, 0);
	iter = 0;
	stop = false;

	do
	{
		iter++;
		//Reset state vector
		sv_CSM = sv_V;
		//Calculate PCC DV
		DV = DV_aapo + DV_apo;
		//Save maneuver data and apply DV
		sv_CSM = SaveElements(sv_CSM, 0, DV);

		//DOI
		DOIManeuver(1);

		//Calculate out of plane error
		xi = OutOfPlaneError(sv_LM);
		if (abs(xi) <= zeta_theta || iter >= 10)
		{
			stop = true;
		}
		else
		{
			//Calculate plane change
			CHAPLA_FixedTIG(sv_V, sv_LM, t_TD - 1000.0, deltaw_s, DV_apo);
		}
	} while (stop == false);

	return 0;
}

int LDPP::Mode2()
{
	if (opt.IDO <= 0)
	{
		return Mode2_1();
	}
	else
	{
		return Mode2_2();
	}
}

int LDPP::Mode2_1()
{
	I_Num = 2;

	//Mode 2, Sequence 1: Compute CSM maneuver to establish an apsis and an input altitude at the DOI maneuver point
	return 0;
}

int LDPP::Mode2_2()
{
	//Mode 2, Sequence 2: Compute CSM maneuver to circularize orbit at an input altitude

	VECTOR3 DV_apo;
	double dt;
	bool error;

	I_Num = 2;

	//Input state vector as CSM state vector
	sv_CSM = opt.sv0;

	//Advance to threshold time of first maneuver
	dt = opt.TH[0] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	//Advance to altitude
	error = STCIR(sv_CSM, opt.H_W, true, sv_CSM);
	//Calculate circ burn
	DV_apo = SAC(0.0, true, sv_CSM);
	//Apply DV
	sv_CSM = SaveElements(sv_CSM, 0, DV_apo);
	//Calculate DOI
	DOIManeuver(1);
	return 0;
}

int LDPP::Mode3()
{
	//Compute CSM two-maneuver sequence with the first maneuver performed at an input time or apsis and the second maneuver performed at an input altitude to circularize the orbit

	VECTOR3 DV;
	double dt;
	bool error;

	I_Num = 3;

	//Input state vector as CSM state vector
	sv_CSM = opt.sv0;

	//Advance to threshold time of first maneuver
	dt = opt.TH[0] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	if (opt.IDO >= 0)
	{
		//Advance CSM to next apsis
		sv_CSM = STAP(sv_CSM, error);
	}

	//Compute a maneuver to give an altitude 180° around
	DV = SAC(opt.H_W, false, sv_CSM);
	//Save maneuver
	sv_CSM = SaveElements(sv_CSM, 0, DV);
	//Advance to next threshold time
	dt = opt.TH[1] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	//Advance to desired apsis
	for (int i = 0; i < 2; i++)
	{
		sv_CSM = STAP(sv_CSM, error);
		if (abs(length(sv_CSM.R) - (opt.R_LS + opt.H_W)) < 2.0*1852.0) break;
	}
	//Compute a maneuver to circularize CSM orbit
	DV = SAC(0.0, true, sv_CSM);
	//Apply manuever
	sv_CSM = SaveElements(sv_CSM, 1, DV);
	//DOI
	DOIManeuver(2);
	return 0;
}

int LDPP::Mode4()
{
	//DOI

	double dt;

	I_Num = 1;

	//Input state vector as CSM state vector
	sv_CSM = opt.sv0;

	//Advance to threshold time of first maneuver
	dt = opt.TH[0] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);

	DOIManeuver(0);
	return 0;
}

int LDPP::Mode5()
{
	//Double Hohmann plane change DOI maneuver sequence
	return 0;
}

int LDPP::Mode6()
{
	//PDI
	return 0;
}

int LDPP::Mode7()
{
	//CSM prelaunch plane change maneuver
	return 0;
}

VECTOR3 LDPP::SAC(double h_W, bool J, EphemerisData sv_L) const
{
	//h_W: height wanted at an apsis, or circular altitude wanted
	//J: false = compute maneuver to yield h_W at an apsis 180° away from maneuver, true = compute maneuver to circularize at input altitude

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
	if (J == false)
	{
		//Height change
		R_A = opt.R_LS + h_W;
	}
	else
	{
		//Circularization
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

double LDPP::ArgLat(VECTOR3 R, VECTOR3 V) const
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

VECTOR3 LDPP::LATLON(double GMT) const
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

void LDPP::CHAPLA(EphemerisData sv_L, bool IWA, bool IGO, int I, double TH, double &t_m, VECTOR3 &DV) const
{
	//INPUTS:
	//sv_L: input state vector before landing site passage
	//IWA: false = the azimuth is not specified, true = the azimuth is specified
	//IGO: false = compute plane change as if it were the only maneuver being done, true = compute plane change as part of maneuver sequence
	//I: number of maneuvers in sequence
	//I_PC: number of the plane-change maneuver
	//TH: Threshold for flying over the landing site

	//OUTPUTS:
	//t_m: maneuver time
	//DV: maneuver delta V

	EphemerisData sv_TH, sv_CA, sv_A, sv_P;
	MATRIX3 Rot;
	double rmag, vmag, rtasc, decl, fpav, az;
	int ii;
	bool error;

	error = LSClosestApproach(sv_L, TH, sv_CA);

	if (error)
	{
		//TBD: Error message: Failed to converge on closest approach in PCPLCH. Processing continued.
	}
	
	//Convert state vector to MCT coordinates
	sv_P = sv_CA;
	pRTCC->ELVCNV(sv_CA.GMT, RTCC_COORDINATES_MCT, RTCC_COORDINATES_MCI, Rot);

	sv_P.R = tmul(Rot, sv_P.R);
	sv_P.V = tmul(Rot, sv_P.V);

	//Convert to spherical coordinates
	OrbMech::rv_from_adbar(sv_P.R, sv_P.V, rmag, vmag, rtasc, decl, fpav, az);

	//Replace azimuth with desired azimuth, if that option is used
	if (IWA == true)
	{
		az = opt.azi_nom;
	}

	//Convert spherical coordinates back to cartesian, keeping the radius, velocity and flight path angle the same, but replacing latitude and longitude with the landing site
	OrbMech::adbar_from_rv(rmag, vmag, opt.Lng_LS, opt.Lat_LS, fpav, az, sv_P.R, sv_P.V);

	//Convert back to MCI coordinates
	sv_P.R = mul(Rot, sv_P.R);
	sv_P.V = mul(Rot, sv_P.V);

	//Passive vehicle is now a state vector above the landing site, active vehicle is as input
	sv_A = sv_L;

	//Iteration loop to propagate backwards through maneuvers
	if (IGO)
	{
		ii = I;
		CELEMENTS elem_A, elem_P;

		do
		{
			//Propagate backwards to previous maneuver time
			sv_P = PMMLAEG(sv_P, 0, t_M[ii - 1], error);
			//Active vehicle 
			elem_A = OrbMech::GIMIKC(LDPP_SV_E[ii - 1][0][0], LDPP_SV_E[ii - 1][0][1], mu);
			elem_P = OrbMech::GIMIKC(sv_P.R, sv_P.V, mu);
			elem_P.a = elem_A.a;
			elem_P.e = elem_A.e;
			OrbMech::GIMKIC(elem_P, mu, sv_P.R, sv_P.V);
			sv_P.GMT = t_M[ii - 1];
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

		//Set active vehicle state vector to state before plane change burn
		sv_A = LoadElements(I_PC - 1, true);
	}
	else
	{
		ii = 1;
	}

	//Advance both vehicles to threshold time for plane change
	sv_A = PMMLAEG(sv_A, 0, opt.TH[I_PC - 1], error);
	sv_P = PMMLAEG(sv_P, 0, opt.TH[I_PC - 1], error);

	//??
	//I = ii;

	//Common node
	CNODE(sv_A, sv_P, t_m, DV);
}

void LDPP::CHAPLA_FixedTIG(EphemerisData sv_TIG, EphemerisData sv_L, double TH, double &deltaw_s, VECTOR3 &DV) const
{
	//INPUTS:
	//sv_TIG: State vector at plane change ignition
	//sv_L: State vector before landing site passage
	//TH: Threshold time for landing site passage
	EphemerisData sv_CA;
	VECTOR3 r_LS, r_P, Q, c, R_P, h_L;
	double nu, delta_nu, delta_w, u_CA, u_MAN, cos_delta_nu, v_L, dv_PC, dv_Z, dv_R, dv_H, u_DUM, u_SIGN;
	bool error;

	error = LSClosestApproach(sv_L, TH, sv_CA);
	if (error)
	{

	}

	//Argument of latitude at TIG and closest approach
	u_CA = ArgLat(sv_CA.R, sv_CA.V);
	u_MAN = ArgLat(sv_TIG.R, sv_TIG.V);

	//Solve for wedge angle between desired orbit and actual orbit
	r_LS = unit(LATLON(sv_CA.GMT));
	h_L = unit(crossp(sv_CA.R, sv_CA.V));
	Q = crossp(r_LS, h_L);
	c = unit(Q);
	R_P = crossp(h_L, c); //landing site projected into orbital plane
	r_P = unit(R_P);
	nu = acos2(dotp(r_LS, r_P)); //angle between landing site and projected landing site (out-of-plane angle)
	delta_nu = acos2(dotp(r_LS, h_L));
	delta_w = sin(nu) / (cos(nu)*cos(u_CA - u_MAN));
	cos_delta_nu = cos(delta_nu);
	delta_w = delta_w * cos_delta_nu / abs(cos_delta_nu);

	//Update wedge angle
	delta_w = deltaw_s + delta_w;

	//Calculate plane change
	v_L = length(sv_TIG.V);
	dv_PC = 2.0*v_L*sin(abs(delta_w) / 2.0);
	dv_Z = dv_PC * cos(abs(delta_w) / 2.0);
	dv_R = 0.0;
	dv_H = -dv_PC * sin(abs(delta_w) / 2.0);
	u_DUM = u_CA - u_MAN;
	u_SIGN = u_DUM;
	OrbMech::normalizeAngle(u_SIGN, true);
	if (delta_nu < PI05)
	{
		dv_Z = -dv_Z;
	}
	if (u_SIGN >= PI)
	{
		dv_Z = -dv_Z;
	}

	DV.x = dv_H;
	DV.y = dv_Z;
	DV.z = dv_R;

	//Save new wedge angle
	deltaw_s = delta_w;
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
	int ii;

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

	//Set up iteration
	ii = 0;

	do
	{
		ii++;
		if (ii > 10)
		{
			sv_out = sv0;
			return true;
		}

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

void LDPP::CNODE(EphemerisData sv_A, EphemerisData sv_P, double &t_m, VECTOR3 &dV_LVLH) const
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

EphemerisData LDPP::PMMLAEG(EphemerisData sv0, int opt, double param, bool &error, double DN) const
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

EphemerisData LDPP::PositionMatch(EphemerisData sv_A, EphemerisData sv_P, double mu) const
{
	//Wrapper to call PositionMatch with EphemerisData instead of SV
	SV sv_A1, sv_P1, sv_P2;

	sv_A1 = pRTCC->ConvertEphemDatatoSV(sv_A);
	sv_P1 = pRTCC->ConvertEphemDatatoSV(sv_P);

	sv_P2 = OrbMech::PositionMatch(pRTCC->SystemParameters.AGCEpoch, sv_A1, sv_P1, mu);

	return pRTCC->ConvertSVtoEphemData(sv_P2);
}

double LDPP::P29TimeOfLongitude(VECTOR3 R0, VECTOR3 V0, double GMT, double phi_d) const
{
	//Wrapper to call P29TimeOfLongitude with GMT instead of MJD
	double GMTBase, MJD, MJD_lng;

	GMTBase = pRTCC->GetGMTBase();
	MJD = OrbMech::MJDfromGET(GMT, GMTBase);

	MJD_lng = OrbMech::P29TimeOfLongitude(pRTCC->SystemParameters.MAT_J2000_BRCS, R0, V0, MJD, hMoon, phi_d);

	return OrbMech::GETfromMJD(MJD_lng, GMTBase);
}

EphemerisData LDPP::SaveElements(EphemerisData sv, int n, VECTOR3 DV)
{
	//Save time
	t_M[n] = sv.GMT;
	//Save state before maneuver
	LDPP_SV_E[n][0][0] = sv.R;
	LDPP_SV_E[n][0][1] = sv.V;
	//Apply DV
	sv = APPLY(sv, DV);
	//Save state after maneuver
	LDPP_SV_E[n][1][0] = sv.R;
	LDPP_SV_E[n][1][1] = sv.V;
	//Save DV
	DeltaV_LVLH[n] = DV;
	//Return state vector after maneuver, if required
	return sv;
}

EphemerisData LDPP::LoadElements(int n, bool before) const
{
	EphemerisData sv;
	int bef;

	if (before)
	{
		bef = 0;
	}
	else
	{
		bef = 1;
	}

	sv.R = LDPP_SV_E[n][bef][0];
	sv.V = LDPP_SV_E[n][bef][1];
	sv.GMT = t_M[n];
	sv.RBI = BODY_MOON;

	return sv;
}

double LDPP::OutOfPlaneError(EphemerisData sv) const
{
	//sv is at time when the check is desired

	VECTOR3 h_LM, R_LS, r_LS, Q, c, R_P, r_P;
	double xi;

	//Orbital plane of the LM
	h_LM = unit(crossp(sv.R, sv.V));
	//Inertial landing site vector
	R_LS = LATLON(sv.GMT);
	r_LS = unit(R_LS);
	//Compute a unit vector through the point of closest approach
	Q = crossp(r_LS, h_LM);
	c = unit(Q);
	R_P = crossp(h_LM, c);
	r_P = unit(R_P);
	//Out-of-plane error
	xi = acos(dotp(r_P, r_LS));

	return xi;
}

double LDPP::OrbitalPeriod(EphemerisData sv) const
{
	//Orbital period adjusted for J2
	MATRIX3 Rot;
	VECTOR3 R_MCT;
	double r, v, ainv, sin_lat, eta;

	pRTCC->ELVCNV(sv.GMT, RTCC_COORDINATES_MCI, RTCC_COORDINATES_MCT, Rot);
	R_MCT = mul(Rot, sv.R);

	r = length(sv.R);
	v = length(sv.V);
	ainv = 2.0 / r - v * v / mu;
	sin_lat = R_MCT.z / length(R_MCT);
	ainv = ainv + OrbMech::J2_Moon*pow(OrbMech::R_Moon, 2) / pow(r, 3)*(1.0 - 3.0*pow(sin_lat, 2));
	eta = sqrt(mu * pow(ainv, 3));

	return PI2 / eta;
}

bool LDPP::LSClosestApproach(EphemerisData sv, double TH, EphemerisData &sv_CA) const
{
	EphemerisData sv_TH;
	VECTOR3 RR_LS, rr_LS, rr_L, vv_L, H_L, h_L, Q, c, R_p, rr_p, H_d, h_d;
	double dt1, dt2, n_L, theta, s1, s2, dt3;
	int ii;

	//Take state vector to threshold time for flying over the landing site
	dt1 = TH - sv.GMT;
	sv_TH = pRTCC->coast(sv, dt1);
	//Calculate time of landing site passage after threshold time
	sv_CA.GMT = P29TimeOfLongitude(sv_TH.R, sv_TH.V, sv_TH.GMT, opt.Lng_LS);
	dt2 = sv_CA.GMT - sv_TH.GMT;
	//Take state vector to approximate time of landing site passage
	sv_CA = pRTCC->coast(sv_TH, dt2);
	//Mean motion of orbit
	n_L = PI2 / OrbMech::period(sv_CA.R, sv_CA.V, mu);

	//Iteration loop to calculate closest approach (not identical to LS longitude!)
	ii = 0;
	do
	{
		RR_LS = LATLON(sv_CA.GMT);
		rr_LS = unit(RR_LS);

		rr_L = unit(sv_CA.R);
		vv_L = unit(sv_CA.V);
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
			sv_CA = pRTCC->coast(sv_CA, dt3);
		}
		ii++;
		//Stop after 10 iterations
		if (ii > 10) return true;
	} while (abs(dt3) > zeta_t);

	return false;
}

int LDPP::DOIManeuver(int i_DOI)
{
	//Function uses sv_CSM, TH[3] and H_DP and outputs elemets, sv_CSM at DOI time, sv_LM at touchdown time and t_DOI, t_IGN and t_TD

	VECTOR3 DV_apo;
	double dt;
	int err;

	//DOI
	//Advance to threshold
	dt = opt.TH[3] - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	//Calculate DOI time
	err = LLTPR(opt.TH[3], sv_CSM, t_DOI, t_IGN, t_TD);
	//Advance to CSM
	dt = t_DOI - sv_CSM.GMT;
	sv_CSM = pRTCC->coast(sv_CSM, dt);
	//Assign CSM vector and time to LM
	sv_LM = sv_CSM;
	//Compute DOI
	DV_apo = SAC(opt.H_DP, 0, sv_LM);
	//Save state vector and apply DV
	sv_LM = SaveElements(sv_LM, i_DOI, DV_apo);
	//Advance to time of touchdown
	dt = t_TD - sv_LM.GMT;
	sv_LM = pRTCC->coast(sv_LM, dt);
	return 0;
}

void LDPP::OutputCalculations()
{
	if (opt.MODE != 7)
	{
		//Compute new LM weight
		opt.W_LM = opt.W_LM / exp(length(DeltaV_LVLH[I_Num - 1]) / opt.I_SP);
		if (opt.I_PD == false)
		{
			double T_PD;
			if (opt.I_TPD)
			{
				//Input time to begin powered descent T_PD
				T_PD = opt.T_PD;
			}
			else
			{
				//Allow program to compute time to begin powered descent T_PD
				T_PD = 0.0; //TBD
			}
			//TBD: Powered descent guidance subroutine
		}
	}
	//Compute display quantites and output
	for (int ii = 0; ii < 4; ii++)
	{
		outp.DeltaV_LVLH[ii] = DeltaV_LVLH[ii];
		outp.T_M[ii] = t_M[ii];
		outp.sv_before[ii] = LoadElements(ii, true);
		outp.V_after[ii] = LDPP_SV_E[ii][1][0];
	}
	if (opt.MODE == 7)
	{
		outp.azi = opt.azi_nom;
		outp.t_Land = 0.0;
		outp.t_PDI = 0.0;
		//outp.T_M[1] = GMT_LS_CA; //Show as second maneuver time
	}
	else
	{
		//For now, from the old DOI calculation
		double dt4;
		OrbMech::time_theta(sv_LM.R, sv_LM.V, opt.theta_D - 15.0*RAD, mu, dt4);
		outp.azi = opt.azi_nom;
		outp.t_Land = 0.0;// t_L + dt4;
		outp.t_PDI = 0.0;// t_IGN + dt4;
	}
	outp.i = I_Num;
}