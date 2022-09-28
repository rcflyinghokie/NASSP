/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2018

RTCC Calculations for Mission C

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

#include "Orbitersdk.h"
#include "soundlib.h"
#include "apolloguidance.h"
#include "saturn.h"
#include "saturn1b.h"
#include "sivb.h"
#include "iu.h"
#include "LVDC.h"
#include "../src_rtccmfd/OrbMech.h"
#include "mcc.h"
#include "rtcc.h"

bool RTCC::CalculationMTP_C(int fcn, LPVOID &pad, char * upString, char * upDesc, char * upMessage)
{
	char uplinkdata[1024 * 3];
	bool preliminary = true;
	bool scrubbed = false;

	switch (fcn) {
	case 100: //MISSION INITIALIZATION
	{
		char Buff[128];

		//P80 MED: mission initialization
		sprintf_s(Buff, "P80,1,CSM,%d,%d,%d;", GZGENCSN.MonthofLiftoff, GZGENCSN.DayofLiftoff, GZGENCSN.Year);
		GMGMED(Buff);

		//P10 MED: Enter actual liftoff time
		double TEPHEM0, tephem_scal;
		Saturn *cm = (Saturn *)calcParams.src;

		//Get TEPHEM
		TEPHEM0 = 40038.;
		tephem_scal = GetTEPHEMFromAGC(&cm->agc.vagc);
		double LaunchMJD = (tephem_scal / 8640000.) + TEPHEM0;
		LaunchMJD = (LaunchMJD - SystemParameters.GMTBASE)*24.0;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(LaunchMJD*3600.0, hh, mm, ss);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P12: CSM GRR and Azimuth
		Saturn1b *Sat1b = (Saturn1b*)cm;
		LVDC1B *lvdc = (LVDC1B*)Sat1b->iu->GetLVDC();
		double Azi = lvdc->Azimuth*DEG;
		double T_GRR = lvdc->T_GRR;

		sprintf_s(Buff, "P12,CSM,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//P15: CMC clock zero
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P12: IU GRR and Azimuth
		OrbMech::SStoHHMMSS(T_GRR, hh, mm, ss);
		sprintf_s(Buff, "P12,IU1,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//Get actual liftoff REFSMMAT from telemetry
		BZSTLM.CMC_REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		BZSTLM.CMCRefsPresent = true;
		EMSGSUPP(1, 1);
		//Make telemetry matrix current
		GMGMED("G00,CSM,TLM,CSM,CUR;");

		//Initialize config, areas and weights
		med_m55.Table = RTCC_MPT_CSM;
		med_m55.ConfigCode = "C";
		PMMWTC(55);

		med_m55.Table = RTCC_MPT_LM;
		med_m55.ConfigCode = "S";
		PMMWTC(55);

		med_m51.Table = RTCC_MPT_CSM;
		med_m51.CSMArea = 129.4*0.3048*0.3048;
		med_m51.LMAscentArea = med_m51.LMDescentArea = med_m51.SIVBArea = 0.0;
		med_m51.KFactor = 1.0;
		PMMWTC(51);

		med_m51.Table = RTCC_MPT_LM;
		med_m51.SIVBArea = 365.0*0.3048*0.3048;
		med_m51.LMAscentArea = med_m51.LMDescentArea = med_m51.CSMArea = 0.0;
		med_m51.KFactor = 1.0;
		PMMWTC(51);

		med_m50.Table = RTCC_MPT_CSM;
		med_m50.CSMWT = 36300.0* 0.453592;
		PMMWTC(50);

		med_m50.Table = RTCC_MPT_LM;
		med_m50.SIVBWT = 30029.0* 0.453592;
		PMMWTC(50);
	}
	break;
	case 1: // MISSION C PHASING BURN
	{
		AP7ManPADOpt opt;
		SV sv_A, sv_P;
		double GET_TIG;

		AP7MNV * form = (AP7MNV *)pad;

		sv_A = StateVectorCalc(calcParams.src);
		sv_P = StateVectorCalc(calcParams.tgt);

		GET_TIG = OrbMech::HHMMSSToSS(3, 20, 0);

		opt.GETbase = CalcGETBase();
		opt.vessel = calcParams.src;
		opt.TIG = GET_TIG;
		opt.dV_LVLH = _V(-1.8, 0.0, 0.0)*0.3048; //TBD: Change to -5.7 when full drag is simulated
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSMINUS4;
		opt.HeadsUp = false;
		opt.sxtstardtime = 0;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.navcheckGET = 0;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "PHASING BURN");
		sprintf(form->remarks, "heads down, retrograde, -X thrusters");
	}
	break;
	case 101: //S-IVB STATE VECTOR UPLINK
	{
		void *uplink = NULL;
		DCSSLVNAVUPDATE upl;

		SIVB *iuv = (SIVB *)calcParams.tgt;
		IU *iu = iuv->GetIU();

		EphemerisData sv1 = StateVectorCalcEphem(calcParams.tgt);
		EphemerisData sv2 = coast(sv1, GMTfromGET(17460.0) - sv1.GMT); //TBD: Take drag into account?
		CMMSLVNAV(sv2.R, sv2.V, sv2.GMT);

		upl.PosS = CZNAVSLV.PosS;
		upl.DotS = CZNAVSLV.DotS;
		upl.NUPTIM = CZNAVSLV.NUPTIM;

		uplink = &upl;
		bool uplinkaccepted = iu->DCSUplink(DCSUPLINK_SLV_NAVIGATION_UPDATE, uplink);

		sprintf(upMessage, "S-IVB Navigation Update");
	}
	break;
	case 2: // MISSION C CONTINGENCY DEORBIT (6-4) TARGETING
	{
		AP7MNV * form = (AP7MNV *)pad;

		MATRIX3 REFSMMAT;
		double CSMmass, gmt_guess, gmt_min, gmt_max;
		AP7ManPADOpt opt;
		EphemerisData sv;
		EMSMISSInputTable intab;
		EphemerisDataTable2 tab; 
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];

		sv = StateVectorCalcEphem(calcParams.src);
		CSMmass = calcParams.src->GetMass();

		//Generate epehemeris for recovery target selection
		gmt_guess = GMTfromGET(8.0*3600.0 + 55.0*60.0);
		gmt_min = gmt_guess;
		gmt_max = gmt_guess + 2.75*60.0*60.0;

		intab.AnchorVector = sv;
		intab.EphemerisBuildIndicator = true;
		intab.ECIEphemerisIndicator = true;
		intab.ECIEphemTableIndicator = &tab;
		intab.EphemerisLeftLimitGMT = gmt_min;
		intab.EphemerisRightLimitGMT = gmt_max;
		intab.ManCutoffIndicator = false;
		intab.VehicleCode = RTCC_MPT_CSM;

		NewEMSMISS(&intab);
		tab.Header.TUP = 1;

		//Run recovery target selection
		RMDRTSD(tab, 1, gmt_guess, -163.0*RAD);

		//Select first entry
		RZJCTTC.R32_lat_T = RZDRTSD.table[0].Latitude*RAD;
		RZJCTTC.R32_lng_T = RZDRTSD.table[0].Longitude*RAD;
		RZJCTTC.R32_GETI = RZDRTSD.table[0].GET - 20.0*60.0;

		//MEDs
		RZJCTTC.R32_Code = 1;
		RZJCTTC.Type = 1;

		RZJCTTC.R31_Thruster = RTCC_ENGINETYPE_CSMSPS;
		RZJCTTC.R31_GuidanceMode = 4;
		RZJCTTC.R31_BurnMode = 3;
		RZJCTTC.R31_dt = 0.0;
		RZJCTTC.R31_dv = 0.0;
		RZJCTTC.R31_AttitudeMode = 1;
		RZJCTTC.R31_LVLHAttitude = _V(0.0, -48.5*RAD, PI);
		RZJCTTC.R31_UllageTime = 15.0;
		RZJCTTC.R31_Use4UllageThrusters = true;
		RZJCTTC.R31_REFSMMAT = RTCC_REFSMMAT_TYPE_CUR;
		RZJCTTC.R31_GimbalIndicator = -1;
		RZJCTTC.R31_InitialBankAngle = 0.0;
		RZJCTTC.R31_GLevel = 0.2;
		RZJCTTC.R31_FinalBankAngle = 55.0*RAD;

		RMSDBMP(sv, CSMmass);

		//Save data
		TimeofIgnition = RZRFDP.data[2].GETI;
		SplashLatitude = RZRFDP.data[2].lat_T*RAD;
		SplashLongitude = RZRFDP.data[2].lng_T*RAD;
		DeltaV_LVLH = RZRFTT.Manual.DeltaV;

		REFSMMAT = RZRFDP.data[2].REFSMMAT; //REFSMMAT for uplink

		//Save REFSMMAT in DOD slot
		GMGMED("G11,CSM,DOM;");
		//Move REFSMMAT to current
		GMGMED("G00,CSM,DOD,CSM,CUR;");

		opt.vessel = calcParams.src;
		opt.GETbase = CalcGETBase();
		opt.TIG = TimeofIgnition;
		opt.dV_LVLH = DeltaV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = true;
		opt.sxtstardtime = -25 * 60;
		opt.REFSMMAT = REFSMMAT;
		opt.navcheckGET = 8 * 60 * 60 + 17 * 60;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "6-4 DEORBIT");

		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv);
		CMCRetrofireExternalDeltaVUpdate(buffer2, SplashLatitude, SplashLongitude, TimeofIgnition, DeltaV_LVLH);
		AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

		sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load, REFSMMAT");
		}
	}
	break;
	case 102: //MANUAL RETRO ATTITUDE ORIENTATION TEST
	{
		AP7RETRORIENTPAD * form = (AP7RETRORIENTPAD *)pad;

		RTACFGOSTInput in;
		RTACFGOSTOutput out;

		in.get = OrbMech::HHMMSSToSS(6, 10, 0);
		in.LVLHRoll = 0.0;
		in.LVLHYaw = PI;
		in.option = 4;
		in.REFSMMAT = EZJGMTX1.data[0].REFSMMAT;
		in.sv = StateVectorCalcEphem(calcParams.src);
		in.Weight = calcParams.src->GetMass();

		RTACFGuidanceOpticsSupportTable(in, out);

		form->GET_Day = in.get;
		form->RetroAtt_Day = out.IMUAtt*DEG;
		form->RetroAtt_Day.x = round(form->RetroAtt_Day.x);
		form->RetroAtt_Day.y = round(form->RetroAtt_Day.y);
		form->RetroAtt_Day.z = round(form->RetroAtt_Day.z);

		in.get = OrbMech::HHMMSSToSS(6, 50, 0);
		RTACFGuidanceOpticsSupportTable(in, out);

		form->GET_Night = in.get;
		form->RetroAtt_Night = out.IMUAtt*DEG;
		form->RetroAtt_Night.x = round(form->RetroAtt_Night.x);
		form->RetroAtt_Night.y = round(form->RetroAtt_Night.y);
		form->RetroAtt_Night.z = round(form->RetroAtt_Night.z);
	}
	break;
	case 3: //MISSION C BLOCK DATA UPDATE 2
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 136.7*RAD, -16.2*RAD, -22.0*RAD, -33.0*RAD, -28.2*RAD, -62.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(13,29,36),OrbMech::HHMMSSToSS(14,19,12),OrbMech::HHMMSSToSS(15,54,48),OrbMech::HHMMSSToSS(17,28,48),OrbMech::HHMMSSToSS(19,8,6),OrbMech::HHMMSSToSS(20,34,3) };
		std::string area[] = { "009-3B", "010-AC", "011-AC", "012-AC", "013-2A", "014-1B" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 4: //MISSION C 2ND PHASING MANEUVER
	{
		EphemerisData sv_A, sv_P, sv_A1, sv_A1_apo, sv_A2, sv_P2;
		double GET_TIG, GMT1, GMT2;
		MATRIX3 Q_Xx;
		VECTOR3 R_c_u, R_t_u, H_t_u, dV_LVLH;
		double r_t, theta1, DR, DR_act, dv, erro, dvo, err;
		double c_I, p_I, tol;
		int s_I;

		AP7MNV * form = (AP7MNV *)pad;

		//Update masses
		med_m50.Table = RTCC_MPT_CSM;
		med_m50.CSMWT = calcParams.src->GetMass();
		PMMWTC(50);

		med_m50.Table = RTCC_MPT_LM;
		med_m50.SIVBWT = calcParams.tgt->GetMass();
		PMMWTC(50);

		//Iterator variable
		c_I = 0.0;
		s_I = 0;

		//TIG GET
		GET_TIG = OrbMech::HHMMSSToSS(15, 52, 0);
		//TIG GMT
		GMT1 = GMTfromGET(GET_TIG);
		//Time of NCC1 maneuver
		GMT2 = GMTfromGET(OrbMech::HHMMSSToSS(26, 25, 0));
		//Desired leading distance at NCC1
		DR = 76.5 * 1852;
		//Initial guess for DV
		dv = 0.0;
		//Tolerance
		tol = 10.0;

		//Get state vectors
		sv_A = StateVectorCalcEphem(calcParams.src);
		sv_P = StateVectorCalcEphem(calcParams.tgt);

		//Coast to TIG
		sv_A1 = coast(sv_A, GMT1 - sv_A.GMT, RTCC_MPT_CSM);
		sv_A1_apo = sv_A1;
		Q_Xx = OrbMech::LVLH_Matrix(sv_A1.R, sv_A1.V);

		//Coast to NCC1
		sv_P2 = coast(sv_P, GMT2 - sv_P.GMT, RTCC_MPT_LM);
		r_t = length(sv_P2.R);
		R_t_u = unit(sv_P2.R);
		H_t_u = unit(crossp(sv_P2.R, sv_P2.V));
		
		do
		{
			//Simulate maneuver
			dV_LVLH = _V(dv, 0, 0);
			sv_A1_apo.V = sv_A1.V + tmul(Q_Xx, dV_LVLH);

			//Coast to NCC1
			sv_A2 = coast(sv_A1_apo, GMT2 - sv_A1.GMT, RTCC_MPT_CSM);

			//Calculate actual downrange distance
			R_c_u = unit(sv_A2.R);
			theta1 = atan2(dotp(H_t_u, crossp(R_t_u, R_c_u)), dotp(R_c_u, R_t_u));
			DR_act = r_t * theta1;

			//Calculate err
			err = DR_act - DR;

			//Iterate
			if (abs(err) > tol)
			{
				OrbMech::ITER(c_I, s_I, err, p_I, dv, erro, dvo);
			}
		} while (abs(err) > tol);

		if (length(dV_LVLH) < 1.0*0.3048)
		{
			scrubbed = true;
		}

		if (scrubbed)
		{
			sprintf(upMessage, "Second Phasing Maneuver not necessary.");
		}
		else
		{
			AP7ManPADOpt opt;

			opt.GETbase = CalcGETBase();
			opt.vessel = calcParams.src;
			opt.TIG = GET_TIG;
			opt.dV_LVLH = dV_LVLH;
			opt.enginetype = SPSRCSDecision(SPS_THRUST / calcParams.src->GetMass(), dV_LVLH);
			opt.HeadsUp = true;
			opt.sxtstardtime = 0;
			opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			opt.navcheckGET = 0;
			opt.UllageDT = 15.0;
			opt.UllageThrusterOpt = true;

			AP7ManeuverPAD(&opt, *form);
			sprintf(form->purpose, "PHASING BURN");

			if (opt.enginetype == RTCC_ENGINETYPE_CSMRCSPLUS4)
			{
				sprintf(form->remarks, "heads up, +X thrusters");
			}
			else
			{
				sprintf(form->remarks, "heads up, 4 jets, 15 seconds ullage");
			}
		}
	}
	break;
	case 5: //MISSION C BLOCK DATA UPDATE 3
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 7;
		double lng[] = { -62.9*RAD, -63.0*RAD, -62.9*RAD, -68.5*RAD, -162.4*RAD, -162.3*RAD, -163.3*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(22,10,42),OrbMech::HHMMSSToSS(23,46,41),OrbMech::HHMMSSToSS(25,22,18),OrbMech::HHMMSSToSS(26,56,28),OrbMech::HHMMSSToSS(29,43,42),OrbMech::HHMMSSToSS(31,18,29), OrbMech::HHMMSSToSS(32,53,56) };
		std::string area[] = { "015-1A", "016-1B", "017-1A", "018-1A", "019-4A", "020-4A", "021-4A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 7://MISSION C FINAL NCC1 MANEUVER
	{
		preliminary = false;
	}
	case 6: //MISSION C PRELIMINARY NCC1 MANEUVER
	{
		TwoImpulseOpt lambert;
		TwoImpulseResuls res;
		AP7ManPADOpt opt;
		PMMMPTInput in;
		VECTOR3 dV_LVLH;
		double GET_TIG_imp, GMT_TIG_imp, P30TIG, GETBase;
		EphemerisData sv_A, sv_P;//, sv_A1, sv_P1;
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];

		AP7MNV * form = (AP7MNV *)pad;

		//Update masses
		med_m50.Table = RTCC_MPT_CSM;
		med_m50.CSMWT = calcParams.src->GetMass();
		PMMWTC(50);

		med_m50.Table = RTCC_MPT_LM;
		med_m50.SIVBWT = calcParams.tgt->GetMass();
		PMMWTC(50);

		GETBase = CalcGETBase();

		GET_TIG_imp = OrbMech::HHMMSSToSS(26, 25, 0);
		GMT_TIG_imp = GMTfromGET(GET_TIG_imp);

		//Get state vectors
		sv_A = StateVectorCalcEphem(calcParams.src);
		sv_P = StateVectorCalcEphem(calcParams.tgt);

		//Propagate to time tags
		//sv_A1 = coast(sv_A, GMT_TIG_imp - 12.0*60.0 - sv_A.GMT, RTCC_MPT_CSM);
		//sv_P1 = coast(sv_P, GMT_TIG_imp + 12.0*60.0 - sv_P.GMT, RTCC_MPT_LM);

		lambert.mode = 5;
		lambert.T1 = GMT_TIG_imp;
		lambert.T2 = GMTfromGET(OrbMech::HHMMSSToSS(28, 1, 0));
		lambert.ChaserVehicle = RTCC_MPT_CSM;
		lambert.sv_A = sv_A;
		lambert.sv_P = sv_P;
		lambert.DH = 8.0*1852.0;
		lambert.PhaseAngle = 1.32*RAD;

		PMSTICN(lambert, res);

		in.CONFIG = 1; //CSM
		in.CSMWeight = med_m50.CSMWT;
		in.sv_before = res.sv_tig;
		in.V_aft = res.sv_tig.V + res.dV;
		in.DETU = 15.0; //Ullage
		in.UT = true; //4 jets
		in.IgnitionTimeOption = false;
		in.IterationFlag = true;
		in.LMWeight = 0.0;
		in.Thruster = RTCC_ENGINETYPE_CSMSPS;
		in.VC = RTCC_MANVEHICLE_CSM;
		in.VehicleArea = PZMPTCSM.ConfigurationArea;

		double GMT_TIG;
		PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
		P30TIG = GETfromGMT(GMT_TIG);

		opt.GETbase = GETBase;
		opt.vessel = calcParams.src;
		opt.TIG = P30TIG;
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = true;
		opt.sxtstardtime = -30 * 60;
		opt.UllageDT = 15.0;
		opt.UllageThrusterOpt = true;

		if (preliminary)
		{
			REFSMMATOpt refsopt;

			refsopt.GETbase = GETBase;
			refsopt.REFSMMATopt = 2;
			refsopt.REFSMMATTime = 23 * 60 * 60 + 24 * 60 + 8;
			refsopt.vessel = calcParams.src;

			opt.REFSMMAT = REFSMMATCalc(&refsopt);
			opt.navcheckGET = 25 * 60 * 60 + 41 * 60 + 55;
		}
		else
		{
			opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			opt.navcheckGET = 25 * 60 * 60 + 42 * 60;
			sprintf(form->remarks, "posigrade, heads up");
		}

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "NCC1");

		AGCStateVectorUpdate(buffer1, 1, 1, sv_A);
		AGCStateVectorUpdate(buffer2, 1, 3, sv_P);
		CMCExternalDeltaVUpdate(buffer3, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM and S-IVB state vectors, target load");
		}
	}
	break;
	case 8: //MISSION C NCC2 MANEUVER
	{
		TwoImpulseOpt lambert;
		TwoImpulseResuls res;
		AP7ManPADOpt opt;
		EphemerisData sv_A, sv_P;
		double GETBase, GET_TIG_imp, P30TIG, CSMmass;

		AP7MNV * form = (AP7MNV *)pad;

		sv_A = StateVectorCalcEphem(calcParams.src);
		sv_P = StateVectorCalcEphem(calcParams.tgt);
		CSMmass = calcParams.src->GetMass();

		GETBase = CalcGETBase();
		GET_TIG_imp = OrbMech::HHMMSSToSS(27, 30, 0);

		lambert.mode = 5;
		lambert.T1 = GET_TIG_imp;
		lambert.T2 = OrbMech::HHMMSSToSS(28, 1, 0);
		lambert.ChaserVehicle = RTCC_MPT_CSM;
		lambert.sv_A = sv_A;
		lambert.sv_P = sv_P;
		lambert.DH = 8.0*1852.0;
		lambert.PhaseAngle = 1.32*RAD;

		PMSTICN(lambert, res);

		if (length(res.dV) < 10.0*0.3048)
		{
			scrubbed = true;
		}

		if (scrubbed)
		{
			sprintf(upMessage, "NCC2 has been scrubbed.");
		}
		else
		{
			PMMMPTInput in;
			char buffer1[1000];
			VECTOR3 dV_LVLH;
			int enginetype;
			enginetype = SPSRCSDecision(SPS_THRUST / CSMmass, res.dV);

			in.CONFIG = 1; //CSM
			in.CSMWeight = med_m50.CSMWT;
			in.sv_before = res.sv_tig;
			in.V_aft = res.sv_tig.V + res.dV;
			in.DETU = 15.0; //Ullage
			in.UT = true; //4 jets
			in.IgnitionTimeOption = false;
			in.IterationFlag = true;
			in.LMWeight = 0.0;
			in.Thruster = enginetype;
			in.VC = RTCC_MANVEHICLE_CSM;
			in.VehicleArea = PZMPTCSM.ConfigurationArea;

			double GMT_TIG;
			PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
			P30TIG = GETfromGMT(GMT_TIG);

			opt.GETbase = GETBase;
			opt.vessel = calcParams.src;
			opt.TIG = P30TIG;
			opt.dV_LVLH = dV_LVLH;
			opt.enginetype = enginetype;
			opt.HeadsUp = false;
			opt.sxtstardtime = 0;
			opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			opt.navcheckGET = 0;
			opt.UllageDT = 15.0;
			opt.UllageThrusterOpt = true;

			AP7ManeuverPAD(&opt, *form);
			sprintf(form->purpose, "NCC2");

			CMCExternalDeltaVUpdate(buffer1, P30TIG, dV_LVLH);

			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "Target load");
			}
		}
	}
	break;
	case 9: //MISSION C NSR MANEUVER
	{
		SPQOpt spqopt;
		AP7ManPADOpt opt;
		PMMMPTInput in;
		SPQResults res;
		double P30TIG, GETbase;
		VECTOR3 dV_LVLH;
		SV sv_A, sv_P;
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];

		sv_A = StateVectorCalc(calcParams.src); //State vector for uplink
		sv_P = StateVectorCalc(calcParams.tgt); //State vector for uplink
		GETbase = CalcGETBase();

		AP7MNV * form = (AP7MNV *)pad;

		spqopt.E = 27.45*RAD;
		spqopt.GETbase = GETbase;
		spqopt.sv_A = sv_A;
		spqopt.sv_P = sv_P;
		spqopt.t_CSI = -1;
		spqopt.t_CDH = FindDH(sv_A, sv_P, GETbase, 28.0*3600.0 + 1.0*60.0, 8.0*1852.0);

		ConcentricRendezvousProcessor(spqopt, res);

		in.CONFIG = 1; //CSM
		in.CSMWeight = sv_A.mass;
		in.sv_before = ConvertSVtoEphemData(res.sv_C[0]);
		in.V_aft = res.sv_C_apo[0].V;
		in.DETU = 15.0; //Ullage
		in.UT = true; //4 jets
		in.IgnitionTimeOption = false;
		in.IterationFlag = false;
		in.LMWeight = 0.0;
		in.Thruster = RTCC_ENGINETYPE_CSMSPS;
		in.VC = RTCC_MANVEHICLE_CSM;
		in.VehicleArea = PZMPTCSM.ConfigurationArea;
		in.HeadsUpIndicator = false;

		double GMT_TIG;
		PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
		P30TIG = GETfromGMT(GMT_TIG);

		opt.GETbase = GETbase;
		opt.vessel = calcParams.src;
		opt.TIG = P30TIG;
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = false;
		opt.sxtstardtime = 0;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.navcheckGET = 27 * 60 * 60 + 17 * 60;
		opt.UllageDT = 15.0;
		opt.UllageThrusterOpt = true;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "NSR");
		sprintf(form->remarks, "heads down, retrograde");

		AGCStateVectorUpdate(buffer1, sv_A, true, GETbase);
		AGCStateVectorUpdate(buffer2, sv_P, false, GETbase);
		CMCExternalDeltaVUpdate(buffer3, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM and S-IVB state vectors, target load");
		}
	}
	break;
	case 10: //MISSION C TPI MANEUVER
	{
		TwoImpulseOpt lambert;
		TwoImpulseResuls res;
		AP7TPIPADOpt opt;
		EphemerisData sv_A, sv_P;
		double GETbase;

		AP7TPI * form = (AP7TPI *)pad;

		sv_A = StateVectorCalcEphem(calcParams.src);
		sv_P = StateVectorCalcEphem(calcParams.tgt);
		GETbase = CalcGETBase();

		lambert.mode = 5;
		lambert.T1 = -1.0;
		lambert.T2 = -1.0;
		lambert.ChaserVehicle = RTCC_MPT_CSM;
		lambert.sv_A = sv_A;
		lambert.sv_P = sv_P;
		lambert.Elev = 27.45*RAD;
		lambert.WT = 140.0*RAD;

		PMSTICN(lambert, res);

		opt.dV_LVLH = res.dV_LVLH;
		opt.GETbase = GETbase;
		opt.TIG = res.T1;
		opt.sv_A = sv_A;
		opt.sv_P = sv_P;
		opt.mass = calcParams.src->GetMass();

		AP7TPIPAD(opt, *form);
	}
	break;
	case 11: //MISSION C FINAL SEPARATION MANEUVER
	{
		AP7ManPADOpt opt;

		AP7MNV * form = (AP7MNV *)pad;

		opt.dV_LVLH = _V(2.0*0.3048, 0.0, 0.0);
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSMINUS4;
		opt.GETbase = CalcGETBase();
		opt.HeadsUp = false;
		opt.navcheckGET = 0;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.sxtstardtime = 0;
		opt.TIG = OrbMech::HHMMSSToSS(30.0, 20.0, 0.0);
		opt.vessel = calcParams.src;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "PHASING BURN");
		sprintf(form->remarks, "posigrade, heads down, -X Thrusters");
	}
	break;
	case 12: //MISSION C BLOCK DATA 4
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 7;
		double lng[] = { -163.3*RAD, 138.8*RAD, 138.5*RAD, 135.6*RAD, -18.0*RAD, -24.0*RAD, -25.4*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(32,53,42),OrbMech::HHMMSSToSS(34,13,54),OrbMech::HHMMSSToSS(35,49,27),OrbMech::HHMMSSToSS(37,24,28),OrbMech::HHMMSSToSS(38,14,11),OrbMech::HHMMSSToSS(39,49,27), OrbMech::HHMMSSToSS(41,26,13) };
		std::string area[] = { "021-4A", "022-3B", "023-3A", "024-3B", "025-AC", "026-AC", "027-2B" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 13: //MISSION C BLOCK DATA 5
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -25.4*RAD, -28.8*RAD, -63.3*RAD, -66.2*RAD, -66.2*RAD, -66.2*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(41,26,13),OrbMech::HHMMSSToSS(43,2,5),OrbMech::HHMMSSToSS(44,27,52),OrbMech::HHMMSSToSS(46,3,34),OrbMech::HHMMSSToSS(47,39,29),OrbMech::HHMMSSToSS(49,15,9) };
		std::string area[] = { "027-2B", "028-2B", "029-1B", "030-1A", "031-1B", "032-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 14: //MISSION C BLOCK DATA 6
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -145.0*RAD, 150.0*RAD, 139.0*RAD, -165.9*RAD, 139.0*RAD, 137.4*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(52,5,9),OrbMech::HHMMSSToSS(53,21,42),OrbMech::HHMMSSToSS(54,55,7),OrbMech::HHMMSSToSS(56,46,40),OrbMech::HHMMSSToSS(58,7,17),OrbMech::HHMMSSToSS(59,42,35) };
		std::string area[] = { "033-4C", "034-3C", "035-3B", "036-4A", "037-3A", "038-3A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 15: //MISSION C BLOCK DATA 7
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 134.5*RAD, -19.9*RAD, -22.9*RAD, -26.4*RAD, -54.9*RAD, -64.9*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(61,17,53),OrbMech::HHMMSSToSS(62,7,40),OrbMech::HHMMSSToSS(63,43,46),OrbMech::HHMMSSToSS(65,19,43),OrbMech::HHMMSSToSS(66,47,22),OrbMech::HHMMSSToSS(68,20,59) };
		std::string area[] = { "039-3B", "040-AC", "041-AC", "042-2A", "043-1C", "044-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 16: //MISSION C BLOCK DATA 8
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -63.8*RAD, -63.8*RAD, -64.9*RAD, -165.0*RAD, -165.0*RAD, -137.1*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(69,57,34),OrbMech::HHMMSSToSS(71,33,18),OrbMech::HHMMSSToSS(73,8,47),OrbMech::HHMMSSToSS(75,52,32),OrbMech::HHMMSSToSS(77,28,29),OrbMech::HHMMSSToSS(78,47,51) };
		std::string area[] = { "045-1A", "046-1A", "047-1A", "048-4A", "049-4B", "050-3A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 17: //MISSION C SPS-3: SCS MANEUVER AND SLOSH DAMPING TEST
	{
		AP7MNV * form = (AP7MNV *)pad;

		GMPOpt orbopt;
		REFSMMATOpt refsopt;
		AP7ManPADOpt manopt;
		VECTOR3 dV_LVLH, dV_imp;
		double P30TIG, GETbase, TIG_imp;
		MATRIX3 REFSMMAT;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();
		sv = StateVectorCalc(calcParams.src);

		orbopt.AltRef = 1;
		orbopt.H_A = 160.1*1852.0;
		orbopt.H_P = 90.3*1852.0;
		//Over Carnarvon
		orbopt.long_D = 113.72595*RAD;
		orbopt.ManeuverCode = RTCC_GMP_NHL;
		//Gives DV we had previously, needs fixing eventually
		orbopt.dLAN = 0.3*RAD;
		orbopt.sv_in = ConvertSVtoEphemData(sv);
		orbopt.TIG_GET = OrbMech::HHMMSSToSS(75, 18, 0);

		GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);
		PoweredFlightProcessor(sv, GETbase, TIG_imp, RTCC_ENGINETYPE_CSMSPS, 0.0, dV_imp, false, P30TIG, dV_LVLH);

		refsopt.dV_LVLH = dV_LVLH;
		refsopt.GETbase = GETbase;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.REFSMMATopt = 0;
		refsopt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&refsopt);

		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.GETbase = GETbase;
		manopt.HeadsUp = true;
		manopt.navcheckGET = OrbMech::HHMMSSToSS(75, 5, 0);
		manopt.REFSMMAT = REFSMMAT;
		manopt.TIG = P30TIG;
		manopt.vessel = calcParams.src;
		manopt.sxtstardtime = 0.0;

		AP7ManeuverPAD(&manopt, *form);
		sprintf(form->purpose, "SPS-3");

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load");
		}
	}
	break;
	case 18: //MISSION C BLOCK DATA 9
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 138.0*RAD, 138.0*RAD, 137.0*RAD, -15.0*RAD, -22.0*RAD, -30.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(80,23,36),OrbMech::HHMMSSToSS(82,0,15),OrbMech::HHMMSSToSS(83,36,27),OrbMech::HHMMSSToSS(84,22,7),OrbMech::HHMMSSToSS(85,55,7),OrbMech::HHMMSSToSS(87,28,31) };
		std::string area[] = { "051-3B", "052-3B", "053-3A", "054-AC", "55-AC", "056-AC" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 19: //MISSION C BLOCK DATA 10
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -27.0*RAD, -60.0*RAD, -64.0*RAD, -64.4*RAD, -64.5*RAD, -64.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(89,6,20),OrbMech::HHMMSSToSS(90,30,41),OrbMech::HHMMSSToSS(92,6,54),OrbMech::HHMMSSToSS(93,43,29),OrbMech::HHMMSSToSS(95,20,0),OrbMech::HHMMSSToSS(96,52,38) };
		std::string area[] = { "057-2A", "058-1C", "059-1A", "060-1A", "61-1A", "062-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 20: //MISSION C BLOCK DATA 11
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -159.9*RAD, -160.0*RAD, -160.0*RAD, 136.3*RAD, 136.2*RAD, 134.4*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(99,36,59),OrbMech::HHMMSSToSS(101,13,24),OrbMech::HHMMSSToSS(102,46,4),OrbMech::HHMMSSToSS(104,4,38),OrbMech::HHMMSSToSS(105,41,4),OrbMech::HHMMSSToSS(107,13,10) };
		std::string area[] = { "063-4A", "064-4A", "065-4A", "066-3A", "067-3A", "068-3B" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 21: //MISSION C BLOCK DATA 12
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 130.0*RAD, -23.0*RAD, -32.0*RAD, -27.0*RAD, -61.5*RAD, -64.5*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(108,47,28),OrbMech::HHMMSSToSS(109,37,43),OrbMech::HHMMSSToSS(111,10,33),OrbMech::HHMMSSToSS(112,48,12),OrbMech::HHMMSSToSS(114,13,4),OrbMech::HHMMSSToSS(115,48,12) };
		std::string area[] = { "069-3C", "070-AC", "071-AC", "072-2A", "073-1B", "074-1B" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 22: //MISSION C BLOCK DATA 13
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -65.0*RAD, -65.0*RAD, -63.0*RAD, -160.0*RAD, -160.0*RAD, -161.1*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(117,24,4),OrbMech::HHMMSSToSS(119,0,11),OrbMech::HHMMSSToSS(120,33,36),OrbMech::HHMMSSToSS(123,17,25),OrbMech::HHMMSSToSS(124,53,43),OrbMech::HHMMSSToSS(126,27,32) };
		std::string area[] = { "075-1A", "076-1A", "077-1A", "078-4A", "079-4A", "080-4A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 23: //MISSION C SPS-4: MINIMUM IMPULSE
	{
		AP7MNV * form = (AP7MNV *)pad;
		AP7ManPADOpt opt;
		REFSMMATOpt refsopt;
		double t_burn, F, m, dv, GETbase, P30TIG;
		VECTOR3 dV_LVLH;
		MATRIX3 REFSMMAT;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();
		F = SPS_THRUST;
		t_burn = 0.5;
		m = calcParams.src->GetMass();
		dv = F / m * t_burn;

		sv = StateVectorCalc(calcParams.src);

		dV_LVLH = _V(dv, 0.0, 0.0);
		GETbase = CalcGETBase();
		P30TIG = OrbMech::HHMMSSToSS(120, 43, 0);

		refsopt.GETbase = GETbase;
		refsopt.REFSMMATopt = 2;
		refsopt.REFSMMATTime = OrbMech::HHMMSSToSS(120, 43, 0);
		refsopt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&refsopt);

		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.GETbase = GETbase;
		opt.HeadsUp = true;
		opt.navcheckGET = 120.0*3600.0;
		opt.REFSMMAT = REFSMMAT;
		opt.sxtstardtime = -25.0*60.0;
		opt.TIG = P30TIG;
		opt.vessel = calcParams.src;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "SPS-4");

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load");
		}
	}
	break;
	case 24: //MISSION C BLOCK DATA 14
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 136.0*RAD, 136.0*RAD, 134.0*RAD, 170.0*RAD, -22.0*RAD, -25.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(127,45,11),OrbMech::HHMMSSToSS(129,21,34),OrbMech::HHMMSSToSS(130,53,56),OrbMech::HHMMSSToSS(132,33,15),OrbMech::HHMMSSToSS(133,19,17),OrbMech::HHMMSSToSS(134,53,55) };
		std::string area[] = { "081-3A", "082-3A", "083-3B", "084-CC", "085-AC", "086-2C" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 25: //MISSION C BLOCK DATA 15
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -27.0*RAD, -60.0*RAD, -62.2*RAD, -62.0*RAD, -62.2*RAD, -63.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(136,29,19),OrbMech::HHMMSSToSS(137,54,53),OrbMech::HHMMSSToSS(139,30,6),OrbMech::HHMMSSToSS(141,6,7),OrbMech::HHMMSSToSS(142,42,26),OrbMech::HHMMSSToSS(144,16,25) };
		std::string area[] = { "087-2A", "088-1B", "089-1A", "090-1B", "091-1A", "092-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 26: //MISSION C BLOCK DATA 16
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -162.0*RAD, -161.9*RAD, -163.0*RAD, 133.9*RAD, 133.9*RAD, 141.9*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(146,58,14),OrbMech::HHMMSSToSS(148,34,16),OrbMech::HHMMSSToSS(150,9,20),OrbMech::HHMMSSToSS(151,25,41),OrbMech::HHMMSSToSS(153,1,35),OrbMech::HHMMSSToSS(154,38,44) };
		std::string area[] = { "093-4A", "094-4A", "095-4A", "096-3A", "097-3A", "098-3C" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 27: //MISSION C BLOCK DATA 17
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -18.0*RAD, -24.0*RAD, -23.9*RAD, -27.0*RAD, -61.6*RAD, -62.7*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(155,27,54),OrbMech::HHMMSSToSS(157,0,51),OrbMech::HHMMSSToSS(158,35,56),OrbMech::HHMMSSToSS(160,10,26),OrbMech::HHMMSSToSS(161,35,40),OrbMech::HHMMSSToSS(163,10,40) };
		std::string area[] = { "099-AC", "100-AC", "101-2C", "102-2A", "103-1B", "104-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 28: //MISSION C SPS-5: PUGS TEST AND MTVC
	{
		AP7MNV * form = (AP7MNV *)pad;

		GMPOpt orbopt;
		REFSMMATOpt refsopt;
		AP7ManPADOpt manopt;
		VECTOR3 dV_LVLH, dV_imp;
		double P30TIG, GETbase, TIG_imp;
		MATRIX3 REFSMMAT;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();
		sv = StateVectorCalc(calcParams.src);

		orbopt.AltRef = 1;
		orbopt.H_A = 240.6*1852.0;
		orbopt.H_P = 89.8*1852.0;
		//Eastern Test Range
		orbopt.long_D = -88.455*RAD;
		orbopt.dLAN = -6.7*RAD;
		orbopt.ManeuverCode = RTCC_GMP_NHL;
		orbopt.sv_in = ConvertSVtoEphemData(sv);
		orbopt.TIG_GET = OrbMech::HHMMSSToSS(164, 30, 0);

		GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);
		PoweredFlightProcessor(sv, GETbase, TIG_imp, RTCC_ENGINETYPE_CSMSPS, 0.0, dV_imp, false, P30TIG, dV_LVLH);

		refsopt.dV_LVLH = dV_LVLH;
		refsopt.GETbase = GETbase;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.REFSMMATopt = 0;
		refsopt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&refsopt);

		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.GETbase = GETbase;
		manopt.HeadsUp = true;
		manopt.navcheckGET = OrbMech::HHMMSSToSS(164, 18, 0);
		manopt.REFSMMAT = REFSMMAT;
		manopt.TIG = P30TIG;
		manopt.vessel = calcParams.src;
		manopt.sxtstardtime = -20.0*60.0;

		AP7ManeuverPAD(&manopt, *form);
		sprintf(form->purpose, "SPS-5");

		form->Vc += 100.0;
		sprintf(form->remarks, "MTVC takeover at TIG+%.0f seconds, manual cutoff at DV counter equal 100 ft/s.", form->burntime - 30.0);

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load");
		}
	}
	break;
	case 29: //MISSION C BLOCK DATA 18
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -62.7*RAD, -63.1*RAD, -162.5*RAD, -162.5*RAD, -162.5*RAD, 139.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(164,46,6),OrbMech::HHMMSSToSS(166,21,55),OrbMech::HHMMSSToSS(168,59,3),OrbMech::HHMMSSToSS(170,40,38),OrbMech::HHMMSSToSS(172,22,48),OrbMech::HHMMSSToSS(173,34,54) };
		std::string area[] = { "105-1A", "106-1A", "107-4A", "108-4A", "109-4A", "110-3A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 30: //MISSION C BLOCK DATA 19
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { 138.9*RAD, 152.0*RAD, -9.0*RAD, -23.9*RAD, -31.0*RAD, -26.9*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(175,17,19),OrbMech::HHMMSSToSS(177,0,44),OrbMech::HHMMSSToSS(177,42,42),OrbMech::HHMMSSToSS(179,14,47),OrbMech::HHMMSSToSS(180,48,41),OrbMech::HHMMSSToSS(182,26,21) };
		std::string area[] = { "111-3A", "112-3C", "113-AC", "114-AC", "115-AC", "116-2A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 31: //MISSION C BLOCK DATA 20
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -55.2*RAD, -60.0*RAD, -60.0*RAD, -70.2*RAD, -63.0*RAD, -162.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(183,54,59),OrbMech::HHMMSSToSS(185,31,45),OrbMech::HHMMSSToSS(187,12,18),OrbMech::HHMMSSToSS(188,54,8),OrbMech::HHMMSSToSS(190,35,19),OrbMech::HHMMSSToSS(193,9,9) };
		std::string area[] = { "117-1C", "118-1A", "119-1B", "120-1A", "121-1A", "122-4A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 32: //MISSION C BLOCK DATA 21
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -162.0*RAD, -163.5*RAD, -166.0*RAD, -10.0*RAD, -12.0*RAD, -18.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(194,50,14),OrbMech::HHMMSSToSS(196,31,45),OrbMech::HHMMSSToSS(198,9,57),OrbMech::HHMMSSToSS(198,43,50),OrbMech::HHMMSSToSS(200,17,18),OrbMech::HHMMSSToSS(201,50,35) };
		std::string area[] = { "123-4A", "124-4A", "125-CC", "126-AC", "127-AC", "128-AC" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 33: //MISSION C BLOCK DATA 22
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -25.0*RAD, -27.0*RAD, -27.1*RAD, -62.0*RAD, -60.0*RAD, -60.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(203,23,55),OrbMech::HHMMSSToSS(204,58,45),OrbMech::HHMMSSToSS(206,35,31),OrbMech::HHMMSSToSS(208,2,22),OrbMech::HHMMSSToSS(209,40,53),OrbMech::HHMMSSToSS(211,20,47) };
		std::string area[] = { "129-AC", "130-2A", "131-2C", "132-1C", "133-1A", "134-1A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 34: //MISSION C SPS-6: MINIMUM IMPULSE
	{
		AP7MNV * form = (AP7MNV *)pad;
		AP7ManPADOpt opt;
		REFSMMATOpt refsopt;
		double t_burn, F, m, dv, GETbase, P30TIG;
		VECTOR3 dV_LVLH;
		MATRIX3 REFSMMAT;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();
		F = SPS_THRUST;
		t_burn = 0.5;
		m = calcParams.src->GetMass();
		dv = F / m * t_burn;

		sv = StateVectorCalc(calcParams.src);
		dV_LVLH = _V(0.0, dv, 0.0);
		GETbase = CalcGETBase();
		P30TIG = OrbMech::HHMMSSToSS(210, 8, 0);

		refsopt.dV_LVLH = dV_LVLH;
		refsopt.GETbase = GETbase;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.REFSMMATopt = 0;
		refsopt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&refsopt);

		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.GETbase = GETbase;
		opt.HeadsUp = true;
		opt.navcheckGET = OrbMech::HHMMSSToSS(209, 20, 0);
		opt.REFSMMAT = REFSMMAT;
		opt.sxtstardtime = 0.0;
		opt.TIG = P30TIG;
		opt.vessel = calcParams.src;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "SPS-6");

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load");
		}
	}
	break;
	case 35: //MISSION C BLOCK DATA 23
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -63.0*RAD, -161.8*RAD, -162.0*RAD, -161.7*RAD, -164.0*RAD, -5.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(213,0,32),OrbMech::HHMMSSToSS(215,38,45),OrbMech::HHMMSSToSS(217,17,27),OrbMech::HHMMSSToSS(218,57,54),OrbMech::HHMMSSToSS(220,39,3),OrbMech::HHMMSSToSS(221,19,6) };
		std::string area[] = { "135-1A", "136-4A", "137-4B", "138-4A", "139-4B", "140-AC" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 36: //MISSION C SV PAD
	{
		P27PAD * form = (P27PAD *)pad;
		P27Opt opt;

		opt.GETbase = CalcGETBase();
		opt.navcheckGET = OrbMech::HHMMSSToSS(215, 44, 0);
		opt.SVGET = OrbMech::HHMMSSToSS(216, 14, 0);
		opt.vessel = calcParams.src;

		P27PADCalc(&opt, *form);
	}
	break;
	case 37: //MISSION C BLOCK DATA 24
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -10.0*RAD, -8.0*RAD, -20.0*RAD, -31.0*RAD, -27.0*RAD, -27.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(222,51,52),OrbMech::HHMMSSToSS(224,26,0),OrbMech::HHMMSSToSS(225,58,13),OrbMech::HHMMSSToSS(227,30,42),OrbMech::HHMMSSToSS(229,6,36),OrbMech::HHMMSSToSS(230,43,18) };
		std::string area[] = { "141-AC", "142-AC", "143-AC", "144-AC", "145-2A", "146-2C" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 38: //MISSION C BLOCK DATA 25
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -63.0*RAD, -64.5*RAD, -64.4*RAD, -63.0*RAD, -161.5*RAD, -161.5*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(232,9,47),OrbMech::HHMMSSToSS(233,46,42),OrbMech::HHMMSSToSS(235,25,39),OrbMech::HHMMSSToSS(237,7,5),OrbMech::HHMMSSToSS(239,48,35),OrbMech::HHMMSSToSS(241,29,11) };
		std::string area[] = { "147-1B", "148-1A", "149-1A", "150-1A", "151-4A", "152-4A" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 39: //MISSION C SPS-7: SCS MANEUVER
	{
		AP7MNV * form = (AP7MNV *)pad;

		GMPOpt orbopt;
		REFSMMATOpt refsopt;
		AP7ManPADOpt manopt;
		VECTOR3 dV_LVLH, dV_imp;
		double P30TIG, GETbase, TIG_imp;
		MATRIX3 REFSMMAT;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();
		sv = StateVectorCalc(calcParams.src);

		orbopt.dLOA = -22.5*RAD;
		orbopt.ManeuverCode = RTCC_GMP_SAO;
		orbopt.sv_in = ConvertSVtoEphemData(sv);
		orbopt.TIG_GET = OrbMech::HHMMSSToSS(238, 35, 0);

		GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);
		PoweredFlightProcessor(sv, GETbase, TIG_imp, RTCC_ENGINETYPE_CSMSPS, 0.0, dV_imp, false, P30TIG, dV_LVLH);
		dV_LVLH.y = -100.0*0.3048;

		refsopt.dV_LVLH = dV_LVLH;
		refsopt.GETbase = GETbase;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.REFSMMATopt = 0;
		refsopt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&refsopt);

		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.GETbase = GETbase;
		manopt.HeadsUp = true;
		manopt.navcheckGET = OrbMech::HHMMSSToSS(238, 24, 0);
		manopt.REFSMMAT = REFSMMAT;
		manopt.TIG = P30TIG;
		manopt.vessel = calcParams.src;
		manopt.sxtstardtime = 0.0;

		AP7ManeuverPAD(&manopt, *form);
		sprintf(form->purpose, "SPS-7");

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load");
		}
	}
	break;
	case 40: //MISSION C BLOCK DATA 26
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 6;
		double lng[] = { -161.0*RAD, -161.0*RAD, -10.0*RAD, -11.0*RAD, -17.0*RAD, -25.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(243,11,5),OrbMech::HHMMSSToSS(244,47,45),OrbMech::HHMMSSToSS(245,22,27),OrbMech::HHMMSSToSS(246,55,49),OrbMech::HHMMSSToSS(248,28,57),OrbMech::HHMMSSToSS(250,2,0) };
		std::string area[] = { "153-4A", "154-4C", "155-AC", "156-AC", "157-AC", "158-AC" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 41: //MISSION C BLOCK DATA 27
	{
		AP7BLK * form = (AP7BLK *)pad;
		AP7BLKOpt opt;

		int n = 7;
		double lng[] = { -33.0*RAD, -26.5*RAD, -62.0*RAD, -64.2*RAD, -64.5*RAD, -64.2*RAD, -67.0*RAD };
		double GETI[] = { OrbMech::HHMMSSToSS(251,35,18),OrbMech::HHMMSSToSS(253,13,19),OrbMech::HHMMSSToSS(254,39,51),OrbMech::HHMMSSToSS(256,16,31),OrbMech::HHMMSSToSS(257,55,28),OrbMech::HHMMSSToSS(259,39,18),OrbMech::HHMMSSToSS(261,16,45) };
		std::string area[] = { "159-AC", "160-2A", "161-1B", "162-1A", "163-1A", "164-1A", "165-1B" };

		opt.area.assign(area, area + n);
		opt.GETI.assign(GETI, GETI + n);
		opt.lng.assign(lng, lng + n);
		opt.n = n;

		AP7BlockData(&opt, *form);
	}
	break;
	case 42: //MISSION C NOMINAL DEORBIT MANEUVER PAD
	{
		AP7MNV * form = (AP7MNV *)pad;

		EarthEntryOpt entopt;
		EntryResults res;
		AP7ManPADOpt opt;
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		double GETbase;
		SV sv;
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];

		sv = StateVectorCalc(calcParams.src); //State vector for uplink
		GETbase = CalcGETBase();

		entopt.vessel = calcParams.src;
		entopt.GETbase = GETbase;
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.lng = -64.17*RAD;
		entopt.nominal = true;
		entopt.ReA = -2.062*RAD;
		entopt.TIGguess = OrbMech::HHMMSSToSS(259, 39, 16);
		entopt.entrylongmanual = true;
		entopt.useSV = false;

		BlockDataProcessor(&entopt, &res);//dV_LVLH, P30TIG, latitude, longitude, RET, RTGO, VIO, ReA, prec); //Target Load for uplink

		TimeofIgnition = res.P30TIG;
		SplashLatitude = res.latitude;
		SplashLongitude = res.longitude;
		DeltaV_LVLH = res.dV_LVLH;

		refsopt.vessel = calcParams.src;
		refsopt.GETbase = GETbase;
		refsopt.dV_LVLH = res.dV_LVLH;
		refsopt.REFSMMATTime = res.P30TIG;
		refsopt.REFSMMATopt = 1;

		REFSMMAT = REFSMMATCalc(&refsopt); //REFSMMAT for uplink

		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.GETbase = GETbase;
		opt.HeadsUp = true;
		opt.navcheckGET = res.P30TIG - 40.0*60.0;
		opt.REFSMMAT = REFSMMAT;
		opt.sxtstardtime = -20.0*60.0;
		opt.TIG = res.P30TIG;
		opt.vessel = calcParams.src;

		AP7ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "164-1A RETROFIRE");

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);
		CMCRetrofireExternalDeltaVUpdate(buffer2, res.latitude, res.longitude, res.P30TIG, res.dV_LVLH);
		AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

		sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector, target load, Retrofire REFSMMAT");
		}
	}
	break;
	case 43: //MISSION C NOMINAL DEORBIT ENTRY PAD
	{
		AP7ENT * form = (AP7ENT *)pad;

		EarthEntryPADOpt opt;
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;

		refsopt.vessel = calcParams.src;
		refsopt.GETbase = CalcGETBase();
		refsopt.dV_LVLH = DeltaV_LVLH;
		refsopt.REFSMMATTime = TimeofIgnition;
		refsopt.REFSMMATopt = 1;

		REFSMMAT = REFSMMATCalc(&refsopt);

		opt.dV_LVLH = DeltaV_LVLH;
		opt.P30TIG = TimeofIgnition;
		opt.REFSMMAT = REFSMMAT;
		opt.sv0 = StateVectorCalc(calcParams.src);
		opt.preburn = true;
		opt.lat = SplashLatitude;
		opt.lng = SplashLongitude;

		EarthOrbitEntry(opt, *form);
		if (mcc->AbortMode == 0)
		{
			sprintf(form->Area[0], "164-1A");
		}
		else
		{
			sprintf(form->Area[0], "Abort");
		}
		form->Lat[0] = SplashLatitude * DEG;
		form->Lng[0] = SplashLongitude * DEG;
	}
	break;
	case 44: //MISSION C POSTBURN ENTRY PAD
	{
		AP7ENT * form = (AP7ENT *)pad;

		EarthEntryPADOpt opt;

		opt.dV_LVLH = DeltaV_LVLH;
		opt.lat = SplashLatitude;
		opt.lng = SplashLongitude;
		opt.P30TIG = TimeofIgnition;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.preburn = false;
		opt.sv0 = StateVectorCalc(calcParams.src);

		EarthOrbitEntry(opt, *form);
	}
	break;
	case 50: //GENERIC CSM STATE VECTOR UPDATE
	{
		SV sv;
		double GETbase;
		char buffer1[1000];

		sv = StateVectorCalc(calcParams.src); //State vector for uplink
		GETbase = CalcGETBase();

		AGCStateVectorUpdate(buffer1, sv, true, GETbase);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector");
		}
	}
	break;
	case 51: //GENERIC CSM AND TARGET STATE VECTOR UPDATE
	{
		SV sv_A, sv_P;
		double GETbase;
		char buffer1[1000];
		char buffer2[1000];

		sv_A = StateVectorCalc(calcParams.src); //State vector for uplink
		sv_P = StateVectorCalc(calcParams.tgt); //State vector for uplink
		GETbase = CalcGETBase();

		AGCStateVectorUpdate(buffer1, sv_A, true, GETbase);
		AGCStateVectorUpdate(buffer2, sv_P, false, GETbase);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM and S-IVB state vectors");
		}
	}
	break;
	case 52: //CSM STATE VECTOR UPDATE AND NAV CHECK PAD
	{
		AP7NAV * form = (AP7NAV *)pad;

		SV sv;
		double GETbase;
		char buffer1[1000];

		GETbase = CalcGETBase();
		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		NavCheckPAD(sv, *form, GETbase);
		AGCStateVectorUpdate(buffer1, sv, true, GETbase);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector");
		}
	}
	break;
	case 53: //GENERIC CSM AND TARGET STATE VECTOR UPDATE AND CSM NAV CHECK PAD
	{
		AP7NAV * form = (AP7NAV *)pad;

		SV sv_A, sv_P;
		double GETbase;
		char buffer1[1000];
		char buffer2[1000];

		GETbase = CalcGETBase();

		sv_A = StateVectorCalc(calcParams.src); //State vector for uplink
		sv_P = StateVectorCalc(calcParams.tgt); //State vector for uplink

		NavCheckPAD(sv_A, *form, GETbase);
		AGCStateVectorUpdate(buffer1, sv_A, true, GETbase);
		AGCStateVectorUpdate(buffer2, sv_P, false, GETbase);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM and S-IVB state vectors");
		}
	}
	break;
	case 54: //GENERIC SV PAD
	{
		P27PAD * form = (P27PAD *)pad;
		P27Opt opt;

		opt.GETbase = CalcGETBase();
		opt.SVGET = (oapiGetSimMJD() - opt.GETbase)*24.0*3600.0;
		opt.navcheckGET = opt.SVGET + 30 * 60;
		opt.vessel = calcParams.src;

		P27PADCalc(&opt, *form);
	}
	break;
	}

	return scrubbed;
}