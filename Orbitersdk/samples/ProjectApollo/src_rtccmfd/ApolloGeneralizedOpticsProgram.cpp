/****************************************************************************
This file is part of Project Apollo - NASSP

Apollo Generalized Optics Program

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

#include "ApolloGeneralizedOpticsProgram.h"
#include "rtcc.h"

AGOP::AGOP(RTCC *r) :RTCCModule(r)
{

}

void AGOP::Calc(const AGOPInputs &in, AGOPOutputs &out)
{
	out.output_text.clear();
	out.errormessage.clear();

	switch (in.Option)
	{
	case 1:
		CislunarNavigation(in, out);
		break;
	case 2:
		ReferenceBodyComputation(in, out);
		break;
	case 3:
		StarCatalog(in, out);
		break;
	case 4:
		AntennaPointing(in, out);
		break;
	case 5:
		PassiveThermalControl(in, out);
		break;
	case 6:
		HorizonAngles(in, out);
		break;
	case 7:
		OpticalSupportTable(in, out);
		break;
	}
}

void AGOP::CislunarNavigation(const AGOPInputs &in, AGOPOutputs &out)
{
	EphemerisData sv;
	MATRIX3 SMNB;
	VECTOR3 U_S, R_L, R_ZC, R_CL, U_CL, Vec1, Vec2, Vec3;
	double TA, SA;

	// Get star unit vector in BRCS
	U_S = GetStarUnitVector(in, 0);

	out.output_text.push_back("                  OST CISLUNAR NAVIGATION");
	out.output_text.push_back("   GET STAR ID HORZ OPTICS ANGLES INERTIAL ATTITUDE");
	out.output_text.push_back("HR:MIN:SEC DEC/OCT N-F   SFT     TRN     R      P      Y");
	//out.output_text.push_back("XXX:XX:XX XXX/XXX XXXX +XXX.XX +XX.XXX XXX.XX XXX.XX XXX.XX");

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		//Convert position vector to desired SOI
		R_ZC = sv.R;

		if (in.Mode == 1 || in.Mode == 3)
		{
			if (sv.RBI != BODY_EARTH)
			{
				//Convert to Earth
				int err = pRTCC->ELVCNV(R_ZC, sv.GMT, 1, RTCC_COORDINATES_MCI, RTCC_COORDINATES_ECI, R_ZC);
				if (err)
				{
					WriteError(out, 1);
					return;
				}
			}
		}
		else
		{
			if (sv.RBI != BODY_MOON)
			{
				//Convert to Moon
				int err = pRTCC->ELVCNV(R_ZC, sv.GMT, 1, RTCC_COORDINATES_ECI, RTCC_COORDINATES_MCI, R_ZC);
				if (err)
				{
					WriteError(out, 1);
					return;
				}
			}
		}

		// Get horizon/landmark vector in BRCS
		if (in.Mode == 1 || in.Mode == 2)
		{
			// Horizon

			MATRIX3 M;
			VECTOR3 U_Z, u0, u1, u2, R_H, U_sH, t[2];
			double a_H, b_H, x_H, y_H, A, alpha, beta, AA[2];

			U_Z = _V(0, 0, 1);

			u2 = unit(crossp(U_S, R_ZC));
			u0 = unit(crossp(U_Z, u2));
			u1 = crossp(u2, u0);

			M = _M(u0.x, u0.y, u0.z, u1.x, u1.y, u1.z, u2.x, u2.y, u2.z);

			if (in.Mode == 1)
			{
				// Earth
				double SINL, r_F, h;

				SINL = dotp(u1, U_Z);

				r_F = OrbMech::R_Earth; //TBD
				h = 28000.0; //TBD
				a_H = r_F + h; //TBD
				b_H = r_F + h; //TBD
			}
			else
			{
				// Moon

				a_H = OrbMech::R_Moon;
				b_H = OrbMech::R_Moon;
			}

			R_H = mul(M, R_ZC);
			U_sH = mul(M, U_S);

			x_H = R_H.x;
			y_H = R_H.y;

			A = x_H * x_H / a_H / a_H + y_H * y_H / b_H / b_H;

			alpha = a_H / b_H * y_H*sqrt(A - 1.0);
			beta = b_H / a_H * x_H*sqrt(A - 1.0);

			t[0] = _V(x_H + alpha, y_H - beta, 0.0) / A;
			t[1] = _V(x_H - alpha, y_H + beta, 0.0) / A;

			AA[0] = dotp(U_sH, unit(t[0] - R_H));
			AA[1] = dotp(U_sH, unit(t[1] - R_H));

			VECTOR3 t_n, t_f;

			if (AA[1] > AA[0])
			{
				// 1 is near horizon
				t_n = t[1];
				t_f = t[0];
			}
			else
			{
				// 0 is near horizon
				t_n = t[0];
				t_f = t[1];
			}

			VECTOR3 R_L_v[2]; //0 = near, 1 = far horizon
			VECTOR3 R_EM, V_EM, R_ES;

			R_L_v[0] = tmul(M, t_n);
			R_L_v[1] = tmul(M, t_f);

			// Check which site has the higher sun elevation angle

			pRTCC->PLEFEM(1, sv.GMT / 3600.0, 0, &R_EM, &V_EM, &R_ES, NULL);

			VECTOR3 N, rho, rho_apo;
			double sinang[2];

			for (unsigned i = 0; i < 2; i++)
			{
				//Unit horizon vector
				N = unit(R_L_v[i]);

				if (in.Mode == 1 || in.Mode == 3)
				{
					// Earth
					rho = R_ES - R_L_v[i];
				}
				else
				{
					// Moon
					rho = (R_ES - R_EM) - R_L_v[i];
				}

				//Unit vector from horizon vector to sun
				rho_apo = unit(rho);
				sinang[i] = dotp(rho_apo, N);
			}

			if (sinang[0] > sinang[1])
			{
				out.bIsNearHorizon = true;
			}
			else
			{
				out.bIsNearHorizon = false;
			}

			if (out.bIsNearHorizon)
			{
				//Use near horizon solution
				R_L = R_L_v[0];
			}
			else
			{
				//Use far horizon solution
				R_L = R_L_v[1];
			}
		}
		else
		{
			// Landmark

			// Convert to inertial
			if (in.Mode == 3)
			{
				bool err = GetInertialLandmarkVector(in.lmk_lat, in.lmk_lng, in.lmk_alt, sv.GMT, true, R_L);
				if (err)
				{
					WriteError(out, 1);
					return;
				}
			}
			else
			{
				bool err = GetInertialLandmarkVector(in.lmk_lat, in.lmk_lng, in.lmk_alt, sv.GMT, false, R_L);
				if (err)
				{
					WriteError(out, 1);
					return;
				}
			}
		}

		R_CL = R_L - R_ZC;
		U_CL = unit(R_CL);

		Vec3 = U_CL;
		Vec2 = unit(crossp(U_S, U_CL));
		Vec1 = unit(crossp(Vec2, U_CL));
		SMNB = mul(OrbMech::SBNBMatrix(), _M(Vec1.x, Vec1.y, Vec1.z, Vec2.x, Vec2.y, Vec2.z, Vec3.x, Vec3.y, Vec3.z));

		out.IMUAttitude = OrbMech::CALCGAR(in.CSM_REFSMMAT, SMNB);

		OrbMech::CALCSXA(SMNB, U_S, TA, SA);

		//Write line
		std::string line;
		char Buffer[128];

		OrbMech::format_time_HHMMSS(Buffer, pRTCC->GETfromGMT(sv.GMT));
		line.assign(Buffer);

		sprintf(Buffer, " %03d/%03o ", in.StarIDs[0], in.StarIDs[0]);
		line.append(Buffer);

		if (in.Mode == 1 || in.Mode == 2)
		{
			if (out.bIsNearHorizon)
			{
				line.append("NEAR ");
			}
			else
			{
				line.append(" FAR ");
			}
		}
		else
		{
			line.append("     ");
		}

		sprintf(Buffer, "%+07.2lf %+07.3lf %06.2lf %06.2lf %06.2lf", SA*DEG, TA*DEG, out.IMUAttitude.x*DEG, out.IMUAttitude.y*DEG, out.IMUAttitude.z*DEG);
		line.append(Buffer);

		out.output_text.push_back(line);
	}
}

void AGOP::ReferenceBodyComputation(const AGOPInputs &in, AGOPOutputs &out)
{
	std::string line;
	EphemerisData sv;
	VECTOR3 R_EM, V_EM, R_ES;
	char Buffer[128];

	sprintf(Buffer, "MODE %d   REFERENCE BODY COMPUTATION", in.Mode);
	out.output_text.push_back(Buffer);

	if (in.Mode != 1)
	{
		out.output_text.push_back("   GET         RA         DEC          UNIT VECTOR        ");
		out.output_text.push_back("HR:MIN:SEC HR:MIN:SEC HR:MIN:SEC                          ");
		//out.output_text.push_back("XXX:XX:XX  XXX:XX:XX  +XX:XX:XX +0.XXXXX +0.XXXXX +0.XXXXX");
	}

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		bool err = pRTCC->PLEFEM(1, sv.GMT / 3600.0, 0, &R_EM, &V_EM, &R_ES, NULL);

		if (err)
		{
			WriteError(out, 2);
			return;
		}

		OrbMech::format_time_HHMMSS(Buffer, pRTCC->GETfromGMT(sv.GMT));
		line.assign(Buffer);
		line += " ";

		if (in.Mode == 1)
		{
			//Calculate RA and declination of SC wrt the Earth, calculate RA and declination of Earth, Moon, Sun wrt the SC

			out.output_text.push_back("   GET         SPACECRAFT             EARTH    ");
			out.output_text.push_back("HR:MIN:SEC    RA        DEC       RA        DEC");
			//out.output_text.push_back("XXX:XX:XX XXX:XX:XX +XX:XX:XX XXX:XX:XX +XX:XX:XX");

			VECTOR3 R_EV; //Vector of vehicle wrt the Earth
			VECTOR3 u_EV; //Unit vector from Earth to vehicle
			VECTOR3 u_VE; //Unit vector from vehicle to Earth
			VECTOR3 u_VM; //Unit vector from vehicle to Moon
			VECTOR3 u_VS; //Unit vector from vehicle to Sun

			if (sv.RBI == BODY_EARTH)
			{
				R_EV = sv.R;
			}
			else
			{
				R_EV = sv.R + R_EM;
			}

			u_EV = unit(sv.R);
			u_VE = -u_EV;

			u_VM = unit(R_EM - sv.R);
			u_VS = unit(R_ES - sv.R);

			double decl, ra;

			OrbMech::latlong_from_r(u_EV, decl, ra);
			RightAscension_Display(Buffer, ra*DEG);
			line.append(Buffer);
			line += " ";
			Declination_Display(Buffer, decl*DEG);
			line.append(Buffer);
			line += " ";

			OrbMech::latlong_from_r(u_VE, decl, ra);
			RightAscension_Display(Buffer, ra*DEG);
			line.append(Buffer);
			line += " ";
			Declination_Display(Buffer, decl*DEG);
			line.append(Buffer);
			line += " ";
			out.output_text.push_back(line);
			out.output_text.push_back("");

			out.output_text.push_back("                  MOON                 SUN       ");
			out.output_text.push_back("              RA        DEC       RA        DEC  ");
			//out.output_text.push_back("          XXX:XX:XX +XX:XX:XX XXX:XX:XX +XX:XX:XX");

			line = "          ";
			OrbMech::latlong_from_r(u_VM, decl, ra);
			RightAscension_Display(Buffer, ra*DEG);
			line.append(Buffer);
			line += " ";
			Declination_Display(Buffer, decl*DEG);
			line.append(Buffer);
			line += " ";

			OrbMech::latlong_from_r(u_VS, decl, ra);
			RightAscension_Display(Buffer, ra*DEG);
			line.append(Buffer);
			line += " ";
			Declination_Display(Buffer, decl*DEG);
			line.append(Buffer);
			
			out.output_text.push_back(line);

			//Only for the first state vector
			return;
		}
		else
		{
			VECTOR3 u;
			double ra, decl;

			if (in.Mode == 2)
			{
				//Compute RA, declination, unit vector from spaceraft to center of Earth

				VECTOR3 R_EV;

				if (sv.RBI == BODY_EARTH)
				{
					R_EV = sv.R;
				}
				else
				{
					R_EV = sv.R + R_EM;
				}
				u = -unit(R_EV);
			}
			else if (in.Mode == 3)
			{
				//Compute RA, declination, unit vector from spaceraft to center of Moon

				VECTOR3 R_VM;

				if (sv.RBI == BODY_EARTH)
				{
					R_VM = R_EM - sv.R;
				}
				else
				{
					R_VM = -sv.R;
				}

				u = unit(R_VM);
			}
			else if (in.Mode == 4)
			{
				//Compute RA, declination, unit vector from spaceraft to center of Sun

				VECTOR3 R_VS;

				if (sv.RBI == BODY_EARTH)
				{
					R_VS = R_ES - sv.R;
				}
				else
				{
					R_VS = R_ES - (sv.R + R_EM);
				}

				u = unit(R_VS);
			}
			else if (in.Mode == 5)
			{
				//Compute RA, declination, unit vector from spaceraft to Earth landmark

				VECTOR3 R_L;

				bool err = GetInertialLandmarkVector(in.lmk_lat, in.lmk_lng, in.lmk_alt, sv.GMT, true, R_L);
				if (err)
				{
					WriteError(out, 1);
					return;
				}

				if (sv.RBI == BODY_EARTH)
				{
					u = unit(R_L - sv.R);
				}
				else
				{
					u = unit(R_L - (sv.R + R_EM));
				}
			}
			else
			{
				//Compute RA, declination, unit vector from spaceraft to Moon landmark

				VECTOR3 R_L;

				bool err = GetInertialLandmarkVector(in.lmk_lat, in.lmk_lng, in.lmk_alt, sv.GMT, false, R_L);
				if (err)
				{
					WriteError(out, 1);
					return;
				}

				if (sv.RBI == BODY_EARTH)
				{
					u = unit(R_L - (sv.R - R_EM));
				}
				else
				{
					u = unit(R_L - sv.R);
				}
			}

			OrbMech::latlong_from_r(u, decl, ra);
			RightAscension_Display(Buffer, ra*DEG);
			line.append(Buffer);
			line += "  ";
			Declination_Display(Buffer, decl*DEG);
			line.append(Buffer);
			line += "  ";

			for (unsigned i = 0; i < 3; i++)
			{
				sprintf(Buffer, "%+.5lf ", u.data[i]);
				line.append(Buffer);
			}

			out.output_text.push_back(line);
		}
	}
}

void AGOP::StarCatalog(const AGOPInputs &in, AGOPOutputs &out)
{
	std::string line;
	char Buffer[128];
	VECTOR3 u;
	double ra, decl;

	u = in.startable[in.StarIDs[0] - 1];

	out.output_text.push_back("                 STAR CATALOG");
	out.output_text.push_back("STAR ID     RA        DEC            UNIT VECTOR");
	out.output_text.push_back("DEC/OCT HR:MIN:SEC HR:MIN:SEC");
	//out.output_text.push_back("XXX/XXX XXX:XX:XX  +XX:XX:XX  +0.XXXXX +0.XXXXX +0.XXXXX");

	sprintf(Buffer, "%03d/%03o ", in.StarIDs[0], in.StarIDs[0]);
	line.assign(Buffer);

	OrbMech::latlong_from_r(u, decl, ra);
	RightAscension_Display(Buffer, ra*DEG);
	line.append(Buffer);
	line += "  ";
	Declination_Display(Buffer, decl*DEG);
	line.append(Buffer);
	line += "  ";

	for (unsigned i = 0; i < 3; i++)
	{
		sprintf(Buffer, "%+.5lf ", u.data[i]);
		line.append(Buffer);
	}

	out.output_text.push_back(line);
}

void AGOP::AntennaPointing(const AGOPInputs &in, AGOPOutputs &out)
{
	EphemerisData sv;
	MATRIX3 SMNB;
	VECTOR3 R_LMK, R, CSM_Att, LM_Att;
	double lat, lng, alt, CSM_PCH, CSM_YAW, LM_PCH, LM_YAW, SLANT_RANGE, AZM, ELV;
	std::string line;
	char Buffer[128];

	out.output_text.push_back("    STEERABLE ANTENNA POINTING PROGRAM");
	sprintf(Buffer, "MODE %d ", in.Mode);
	line.assign(Buffer);
	if (in.Mode == 1 || in.Mode == 4)
	{
		line.append("ACTIVE VEH CSM ");
	}
	else
	{
		line.append("ACTIVE VEH LEM ");
	}

	if (in.AttIsCSM)
	{
		line.append("POINTING VEH CSM");
	}
	else
	{
		line.append("POINTING VEH LEM");
	}
	out.output_text.push_back(line);

	out.output_text.push_back("          ********CSM********  *********LM********");
	out.output_text.push_back("    GET   PCH YAW OGA IGA MGA  PCH YAW OGA IGA MGA");
	//out.output_text.push_back("XXX:XX:XX XXX XXX XXX XXX XXX  XXX XXX XXX XXX XXX");

	CSM_PCH = CSM_YAW = LM_PCH = LM_YAW = SLANT_RANGE = AZM = ELV = 0.0;
	CSM_Att = LM_Att = _V(0, 0, 0);

	//Find ground station
	if (in.GroundStationID == "")
	{
		lat = in.lmk_lat;
		lng = in.lmk_lng;
		alt = in.lmk_alt;
	}
	else
	{
		StationData stat;
		int j;

		bool found = false;
		for (j = 0; j < pRTCC->SystemParameters.MKRBKS; j++)
		{
			if (in.GroundStationID == pRTCC->GZSTCH[j].data.code)
			{
				found = true;
				break;
			}
		}
		if (found == false)
		{
			WriteError(out, 3);
			return;
		}
		stat = pRTCC->GZSTCH[j].data;

		lat = stat.lat_geoc;
		lng = stat.lng;
		alt = stat.H;
	}

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		GetInertialLandmarkVector(lat, lng, alt, sv.GMT, true, R_LMK);

		//Get vector from spacecraft to ground station
		if (sv.RBI == BODY_EARTH)
		{
			R = R_LMK - sv.R;
		}
		else
		{
			VECTOR3 R_EM, V_EM;
			bool err = pRTCC->PLEFEM(4, sv.GMT / 3600.0, 0, &R_EM, &V_EM, NULL, NULL);

			if (err)
			{
				WriteError(out, 2);
				return;
			}

			R = R_LMK - (R_EM + sv.R);
		}

		SLANT_RANGE = length(R);

		if (in.Mode <= 3)
		{
			if (in.AttIsCSM)
			{
				CSM_Att = in.IMUAttitude[0];
			}
			else
			{
				LM_Att = in.IMUAttitude[0];
			}
		}

		if (in.Mode == 1)
		{
			// S-Band HGA (movable)

			if (!in.AttIsCSM)
			{
				// Convert attitude to CSM
				CSM_Att = LMIMUtoCMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, in.IMUAttitude[0], in.DockingAngle);
				SMNB = OrbMech::CALCSMSC(CSM_Att);
			}
			else
			{
				SMNB = OrbMech::CALCSMSC(in.IMUAttitude[0]);
			}

			CSMHGAngles(R, SMNB, in.CSM_REFSMMAT, CSM_PCH, CSM_YAW);

			out.pitch = CSM_PCH;
			out.yaw = CSM_YAW;
		}
		else if (in.Mode == 2)
		{
			// S-Band Steerable (movable)

			if (in.AttIsCSM)
			{
				// Convert attitude to LM
				LM_Att = CSMIMUtoLMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, in.IMUAttitude[0], in.DockingAngle);
				SMNB = OrbMech::CALCSMSC(LM_Att);
			}
			else
			{
				SMNB = OrbMech::CALCSMSC(in.IMUAttitude[0]);
			}

			LMSteerableAngles(R, SMNB, in.LM_REFSMMAT, LM_PCH, LM_YAW);

			out.pitch = LM_PCH;
			out.yaw = LM_YAW;
		}
		else if (in.Mode == 3)
		{
			// RR (movable)

			if (in.AttIsCSM)
			{
				// Convert attitude to LM
				LM_Att = CSMIMUtoLMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, in.IMUAttitude[0], in.DockingAngle);
				SMNB = OrbMech::CALCSMSC(LM_Att);
			}
			else
			{
				SMNB = OrbMech::CALCSMSC(in.IMUAttitude[0]);
			}

			RRAngles(R, SMNB, in.LM_REFSMMAT, LM_PCH, LM_YAW);
		}
		else if (in.Mode == 4)
		{
			// S-Band HGA (fixed)

			VECTOR3 SCAXIS;

			CSM_PCH = in.AntennaPitch;
			CSM_YAW = in.AntennaYaw;

			SCAXIS = GetBodyFixedHGAVector(CSM_PCH, CSM_YAW);
			SMNB = ThreeAxisPointing(SCAXIS, unit(R), sv.R, sv.V, in.HeadsUp ? 0.0 : PI);

			CSM_Att = OrbMech::CALCGAR(in.CSM_REFSMMAT, SMNB);

			if (in.AttIsCSM == false)
			{
				//Convert to LM
				LM_Att = CSMIMUtoLMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, CSM_Att, in.DockingAngle);
			}
		}
		else if (in.Mode == 5)
		{
			// S-Band Steerable (fixed)

			VECTOR3 SCAXIS;

			LM_PCH = in.AntennaPitch;
			LM_YAW = in.AntennaYaw;

			SCAXIS = GetBodyFixedSteerableAntennaVector(LM_PCH, LM_YAW);
			SMNB = ThreeAxisPointing(SCAXIS, unit(R), sv.R, sv.V, in.HeadsUp ? 0.0 : PI);

			LM_Att = OrbMech::CALCGAR(in.LM_REFSMMAT, SMNB);

			if (in.AttIsCSM)
			{
				//Convert to CSM
				CSM_Att = LMIMUtoCMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, LM_Att, in.DockingAngle);
			}
		}
		else
		{
			// RR (fixed)

			VECTOR3 SCAXIS;

			LM_PCH = in.AntennaPitch;
			LM_YAW = in.AntennaYaw;

			SCAXIS = GetBodyFixedRRVector(LM_YAW, LM_PCH);
			SMNB = ThreeAxisPointing(SCAXIS, unit(R), sv.R, sv.V, in.HeadsUp ? 0.0 : PI);

			LM_Att = OrbMech::CALCGAR(in.LM_REFSMMAT, SMNB);

			if (in.AttIsCSM)
			{
				//Convert to CSM
				CSM_Att = LMIMUtoCMIMUAngles(in.CSM_REFSMMAT, in.LM_REFSMMAT, LM_Att, in.DockingAngle);
			}
		}

		//Write line
		OrbMech::format_time_HHMMSS(Buffer, pRTCC->GETfromGMT(sv.GMT));
		line.assign(Buffer);
		line += " ";

		sprintf(Buffer, "%03.0lf %03.0lf %03.0lf %03.0lf %03.0lf  %03.0lf %03.0lf %03.0lf %03.0lf %03.0lf", CSM_PCH*DEG, CSM_YAW*DEG, CSM_Att.x*DEG, CSM_Att.y*DEG, CSM_Att.z*DEG,
			LM_PCH*DEG, LM_YAW*DEG, LM_Att.x*DEG, LM_Att.y*DEG, LM_Att.z*DEG);
		line.append(Buffer);
		out.output_text.push_back(line);
	}
}

void AGOP::PassiveThermalControl(const AGOPInputs &in, AGOPOutputs &out)
{
	EphemerisData sv;
	MATRIX3 M_NB;
	VECTOR3 R_EV, R_EM, V_EM, R_ES, X_NB, Y_NB, Z_NB, u_VE, u_VS;
	std::string line;
	char Buffer[128];

	out.output_text.push_back("     PASSIVE THERMAL CONTROL     ");
	out.output_text.push_back("   GET            ATTITUDE       ");
	out.output_text.push_back("HR:MIN:SEC  OGA     IGA     MGA  ");
	//out.output_text.push_back("XXX:XX:XX +XXX.XX +XXX.XX +XXX.XX");

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		bool err = pRTCC->PLEFEM(1, sv.GMT / 3600.0, 0, &R_EM, &V_EM, &R_ES, NULL);

		if (err)
		{
			WriteError(out, 2);
			return;
		}

		if (sv.RBI == BODY_EARTH)
		{
			R_EV = sv.R;
		}
		else
		{
			R_EV = R_EM + sv.R;
		}

		u_VE = -unit(R_EV);
		u_VS = unit(R_ES - R_EV);

		X_NB = unit(crossp(u_VE, u_VS));
		Y_NB = -crossp(X_NB, u_VE);
		Z_NB = crossp(X_NB, Y_NB);

		M_NB = _M(X_NB.x, X_NB.y, X_NB.z, Y_NB.x, Y_NB.y, Y_NB.z, Z_NB.x, Z_NB.y, Z_NB.z);

		out.IMUAttitude = OrbMech::CALCGAR(in.CSM_REFSMMAT, M_NB);
		out.REFSMMAT = M_NB;

		//Write line
		OrbMech::format_time_HHMMSS(Buffer, pRTCC->GETfromGMT(sv.GMT));
		line.assign(Buffer);
		line += " ";
		sprintf(Buffer, "%+07.2lf %+07.2lf %+07.2lf", out.IMUAttitude.x*DEG, out.IMUAttitude.y*DEG, out.IMUAttitude.z*DEG);
		line.append(Buffer);
		out.output_text.push_back(line);
	}
}

void AGOP::HorizonAngles(const AGOPInputs &in, AGOPOutputs &out)
{
	EphemerisData sv;
	MATRIX3 M_NB;
	VECTOR3 LVLHAtt;
	double R;
	std::string line;
	char Buffer[128];

	out.output_text.push_back("        HORIZON ALIGNMENT        ");
	out.output_text.push_back("   GET            ATTITUDE       ");
	out.output_text.push_back("HR:MIN:SEC  OGA     IGA     MGA  ");
	//out.output_text.push_back("XXX:XX:XX +XXX.XX +XXX.XX +XXX.XX");

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		if (sv.RBI == BODY_EARTH)
		{
			R = OrbMech::R_Earth;
		}
		else
		{
			R = pRTCC->BZLAND.rad[0];
		}

		LVLHAtt.y = -acos(R / length(sv.R));

		if (in.HeadsUp)
		{
			LVLHAtt.x = 0.0;
			LVLHAtt.y -= pRTCC->SystemParameters.MCGHZA;
		}
		else
		{
			LVLHAtt.x = PI;
			LVLHAtt.y += pRTCC->SystemParameters.MCGHZA;
		}

		if (in.Mode == 1)
		{
			LVLHAtt.z = 0.0;
		}
		else
		{
			LVLHAtt.z = PI;
		}

		M_NB = LVLHAttitude(LVLHAtt, sv.R, sv.V);

		out.IMUAttitude = OrbMech::CALCGAR(in.CSM_REFSMMAT, M_NB);

		//Write line
		OrbMech::format_time_HHMMSS(Buffer, pRTCC->GETfromGMT(sv.GMT));
		line.assign(Buffer);
		line += " ";
		sprintf(Buffer, "%+07.2lf %+07.2lf %+07.2lf", out.IMUAttitude.x*DEG, out.IMUAttitude.y*DEG, out.IMUAttitude.z*DEG);
		line.append(Buffer);
		out.output_text.push_back(line);
	}
}

void AGOP::OpticalSupportTable(const AGOPInputs &in, AGOPOutputs &out)
{
	if (in.Mode == 1)
	{
		LMHorizonCheck(in, out);
	}
	else if (in.Mode == 2)
	{
		OSTAlignmentManeuverCheck(in, out);
	}
	else if (in.Mode == 3)
	{
		OSTComputeREFSMMAT(in, out);
	}
	else if (in.Mode == 4)
	{
		DockingAlignment(in, out);
	}
	else if (in.Mode == 5)
	{
		PointAOTWithCSM(in, out);
	}
	else if (in.Mode == 6)
	{
		//Attitude for preferred REFSMMAT?
	}
	else if (in.Mode == 7)
	{
		//Crescent align?
	}
}

void AGOP::LMHorizonCheck(const AGOPInputs &in, AGOPOutputs &out)
{
	EphemerisData sv;
	MATRIX3 M_BRCS_SM, M_SM_NB, M_BRCS_NB, M, M_NB;
	VECTOR3 u_dir, u_zero, u0, u1, u2, U_Z, R_H, t[2], R_L[2], u_horiz[2], X_NB, Y_NB, Z_NB;
	double a_H, b_H, x_H, y_H, A, alpha, beta;

	U_Z = _V(0, 0, 1);

	//Calculate burn direction (roll, pitch)
	M_BRCS_SM = in.LM_REFSMMAT;
	M_SM_NB = OrbMech::CALCSMSC(_V(0.0, in.IMUAttitude[0].y, in.IMUAttitude[0].z));
	M_BRCS_NB = mul(M_SM_NB, M_BRCS_SM);
	u_dir = tmul(M_BRCS_NB, _V(1, 0, 0));
	u_zero = tmul(M_BRCS_NB, _V(0, 0, 1));

	for (unsigned i = 0; i < in.sv_arr.size(); i++)
	{
		sv = in.sv_arr[i];

		u2 = u_dir;
		u0 = unit(crossp(U_Z, u2));
		u1 = crossp(u2, u0);

		M = _M(u0.x, u0.y, u0.z, u1.x, u1.y, u1.z, u2.x, u2.y, u2.z);

		if (sv.RBI == BODY_EARTH)
		{
			// Earth
			double SINL, r_F, h;

			SINL = dotp(u1, U_Z);

			r_F = OrbMech::R_Earth; //TBD
			h = 28000.0; //TBD
			a_H = r_F + h; //TBD
			b_H = r_F + h; //TBD
		}
		else
		{
			// Moon

			a_H = OrbMech::R_Moon;
			b_H = OrbMech::R_Moon;
		}

		R_H = mul(M, sv.R);

		x_H = R_H.x;
		y_H = R_H.y;

		A = x_H * x_H / a_H / a_H + y_H * y_H / b_H / b_H;

		alpha = a_H / b_H * y_H*sqrt(A - 1.0);
		beta = b_H / a_H * x_H*sqrt(A - 1.0);

		t[0] = _V(x_H + alpha, y_H - beta, 0.0) / A;
		t[1] = _V(x_H - alpha, y_H + beta, 0.0) / A;

		R_L[0] = tmul(M, t[0]);
		R_L[1] = tmul(M, t[1]);

		//Spacecraft +Z axis
		u_horiz[0] = unit(R_L[0] - sv.R);
		u_horiz[1] = unit(R_L[1] - sv.R);

		//TBD: choose solution

		X_NB = u_dir;
		Z_NB = u_horiz[0];
		Y_NB = crossp(Z_NB, X_NB);

		M_NB = _M(X_NB.x, X_NB.y, X_NB.z, Y_NB.x, Y_NB.y, Y_NB.z, Z_NB.x, Z_NB.y, Z_NB.z);

		out.IMUAttitude = OrbMech::CALCGAR(in.LM_REFSMMAT, M_NB);
	}
}

void AGOP::OSTAlignmentManeuverCheck(const AGOPInputs &in, AGOPOutputs &out)
{
	//Options here include
	//Sextant, LM COAS, AOT
	//Star search or input
	//Starting star for star search

	//Initial calculations
	MATRIX3 M_SM_NB, M_BRCS_NB, M_BRCS_SM;
	VECTOR3 u_BRCS, S_SM, S_NB, u_LOS;
	double limit, pitch, yaw;
	unsigned int star, num, search;

	if (in.AttIsCSM)
	{
		M_BRCS_SM = in.CSM_REFSMMAT;
	}
	else
	{
		M_BRCS_SM = in.LM_REFSMMAT;
	}

	M_SM_NB = OrbMech::CALCSMSC(in.IMUAttitude[0]);
	M_BRCS_NB = mul(M_SM_NB, M_BRCS_SM);

	if (in.Instrument == 0)
	{
		u_LOS = mul(OrbMech::SBNBMatrix(), _V(0, 0, 1));
		limit = 50.0*RAD;
	}
	else if (in.Instrument == 1)
	{
		if (in.AttIsCSM)
		{
			u_LOS = _V(1, 0, 0);
		}
		else
		{
			if (in.LMCOASAxis)
			{
				u_LOS = _V(0, 0, 1);
			}
			else
			{
				u_LOS = _V(1, 0, 0);
			}
		}
		limit = 50.0*RAD;
	}
	else
	{
		//AOT
		double AZ, EL;

		AZ = pRTCC->SystemParameters.MDGTCD[in.AOTDetent - 1];
		EL = pRTCC->SystemParameters.MDGETA[in.AOTDetent - 1];

		u_LOS = OrbMech::AOTNavigationBase(AZ, EL);
		limit = 50.0*RAD;
	}

	//Select first star
	if (in.AdditionalOption == 0)
	{
		star = in.StartingStar;
	}
	else
	{
		//TBD: Input stars
		star = in.StarIDs[0];
	}

	num = 0; //Number of valid stars found
	search = 1; //Counter for input stars

	do
	{
		//Search for up to 10 stars

		//Get star vector in BRCS
		u_BRCS = in.startable[star - 1];

		//Convert to stable member
		S_SM = mul(M_BRCS_SM, u_BRCS);
		//Convert to navigation base
		S_NB = mul(M_SM_NB, S_SM);

		//Check if the star is visible
		if (acos(dotp(S_NB, u_LOS)) < limit)
		{
			//TBD: Calculate first AOS/LOS time
			//TBD: Is star available in time span

			//Calculate instrument angles
			if (in.Instrument == 0)
			{
				//Sextant
				OrbMech::CALCSXA(M_SM_NB, S_SM, pitch, yaw);
			}
			else if (in.Instrument == 1)
			{
				//LM COAS
			}
			else
			{
				//AOT
			}

			num++;
		}

		//Check next star
		if (in.AdditionalOption == 0)
		{
			star++;
			if (star > 400) break;
		}
		else
		{
			search++;
			if (search > 10) break;
			star = in.StarIDs[search - 1];
		}
	} while (num < 10);

	//TBD
}

void AGOP::OSTComputeREFSMMAT(const AGOPInputs &in, AGOPOutputs &out)
{
	MATRIX3 M_SM_NB_A, M_SM_NB_B;
	VECTOR3 U_CBA, U_CBB, U_CBA_apo, U_CBB_apo, U_NBA_apo, U_NBB_apo;

	M_SM_NB_A = OrbMech::CALCSMSC(in.IMUAttitude[0]);
	M_SM_NB_B = OrbMech::CALCSMSC(in.IMUAttitude[1]);

	U_CBA = GetStarUnitVector(in, 0);
	U_CBB = GetStarUnitVector(in, 1);

	if (in.Instrument == 0)
	{
		// Sextant
		U_NBA_apo = OrbMech::SXTNB(in.TrunnionShaftAngles[0], in.SextantShaftAngles[0]);
		U_NBB_apo = OrbMech::SXTNB(in.TrunnionShaftAngles[1], in.SextantShaftAngles[1]);
	}
	else if (in.Instrument == 1)
	{
		if (in.AttIsCSM)
		{
			// CSM COAS
			U_NBA_apo = GetCSMCOASVector(in.COASElevationAngle[0], in.COASPositionAngle[0]);
			U_NBB_apo = GetCSMCOASVector(in.COASElevationAngle[1], in.COASPositionAngle[1]);
		}
		else
		{
			// LM COAS
			U_NBA_apo = GetLMCOASVector(in.COASElevationAngle[0], in.COASPositionAngle[0], in.LMCOASAxis);
			U_NBB_apo = GetLMCOASVector(in.COASElevationAngle[1], in.COASPositionAngle[1], in.LMCOASAxis);
		}
	}
	else if (in.Instrument == 2)
	{
		// AOT

		double AZ, EL;

		AZ = pRTCC->SystemParameters.MDGTCD[in.AOTDetent - 1];
		EL = pRTCC->SystemParameters.MDGETA[in.AOTDetent - 1];
		U_NBA_apo = GetAOTNBVector(EL, AZ, in.AOTReticleAngle[0], in.AOTSpiraleAngle[0], in.AOTLineID[0]);
		U_NBB_apo = GetAOTNBVector(EL, AZ, in.AOTReticleAngle[1], in.AOTSpiraleAngle[1], in.AOTLineID[1]);
	}

	U_CBA_apo = tmul(M_SM_NB_A, U_NBA_apo);
	U_CBB_apo = tmul(M_SM_NB_B, U_NBB_apo);

	out.REFSMMAT = OrbMech::AXISGEN(U_CBA_apo, U_CBB_apo, U_CBA, U_CBB);
}

void AGOP::DockingAlignment(const AGOPInputs &in, AGOPOutputs &out)
{

}

void AGOP::PointAOTWithCSM(const AGOPInputs &in, AGOPOutputs &out)
{
	MATRIX3 M_NBCSM_NBLM;
	VECTOR3 SCAXIS, u_LOS;
	double AZ, EL;

	AZ = pRTCC->SystemParameters.MDGTCD[in.AOTDetent - 1];
	EL = pRTCC->SystemParameters.MDGETA[in.AOTDetent - 1];

	SCAXIS = OrbMech::AOTNavigationBase(AZ, EL);

	u_LOS = GetStarUnitVector(in, 0);

	M_NBCSM_NBLM = OrbMech::CSMBodyToLMBody(in.DockingAngle);

	//TBD
}

//Option 8
void AGOP::StarSightingTable(const AGOPInputs &in, AGOPOutputs &out)
{
	if (in.Mode == 1 || in.Mode == 2)
	{
		// Landmark

		//Search for AOS
	}

	//TBD
}

void AGOP::WriteError(AGOPOutputs &out, int err)
{
	switch (err)
	{
	case 1:
		out.errormessage = "UNABLE TO CONVERT VECTORS";
		break;
	case 2:
		out.errormessage = "EPHEMERIDES NOT AVAILABLE";
		break;
	case 3:
		out.errormessage = "GROUND STATION NOT FOUND";
		break;
	}
}

void AGOP::RightAscension_Display(char *Buff, double angle)
{
	double angle2 = abs(round(angle*3600.0));
	sprintf_s(Buff, 32, "%03.0f:%02.0f:%02.0f", floor(angle2 / 3600.0), floor(fmod(angle2, 3600.0) / 60.0), fmod(angle2, 60.0));
}

void AGOP::Declination_Display(char *Buff, double angle)
{
	double angle2 = abs(round(angle*3600.0));
	if (angle >= 0)
	{
		sprintf_s(Buff, 32, "+%02.0f:%02.0f:%02.0f", floor(angle2 / 3600.0), floor(fmod(angle2, 3600.0) / 60.0), fmod(angle2, 60.0));
	}
	else
	{
		sprintf_s(Buff, 32, "-%02.0f:%02.0f:%02.0f", floor(angle2 / 3600.0), floor(fmod(angle2, 3600.0) / 60.0), fmod(angle2, 60.0));
	}
}

bool AGOP::GetInertialLandmarkVector(double lat, double lng, double alt, double GMT, bool isEarth, VECTOR3 &R_LMK)
{
	//Returns landmark position in inertial (ECI or MCI) coordinates

	VECTOR3 R_L_equ;
	if (isEarth)
	{
		R_L_equ = OrbMech::r_from_latlong(lat, lng + OrbMech::w_Earth*GMT, OrbMech::R_Earth + alt);

		int err = pRTCC->ELVCNV(R_L_equ, GMT, 1, RTCC_COORDINATES_ECT, RTCC_COORDINATES_ECI, R_LMK);
		if (err) return true;
	}
	else
	{
		R_L_equ = OrbMech::r_from_latlong(lat, lng, pRTCC->BZLAND.rad[0] + alt);

		int err = pRTCC->ELVCNV(R_L_equ, GMT, 1, RTCC_COORDINATES_MCT, RTCC_COORDINATES_MCI, R_LMK);
		if (err) return true;
	}

	return false;
}

void AGOP::CSMHGAngles(VECTOR3 R, MATRIX3 SMNB, MATRIX3 REFSMMAT, double &pitch, double &yaw)
{
	//R = vector from CM to ground station

	VECTOR3 U_X, U_Y, U_Z, RP, U_RP, U_R;
	double x;

	R = mul(SMNB, mul(REFSMMAT, R)); //R in CM body axes

	U_R = unit(R);
	U_X = _V(1, 0, 0);
	U_Y = _V(0, 1, 0);
	U_Z = _V(0, 0, 1);

	RP = R - U_Z * dotp(R, U_Z);
	U_RP = unit(RP);

	yaw = acos(dotp(U_RP, U_X));
	x = dotp(U_RP, U_Y);
	if (x < 0)
	{
		yaw = PI2 - yaw;
	}
	pitch = acos(dotp(U_R, U_Z)) - PI05;
}

void AGOP::LMSteerableAngles(VECTOR3 R, MATRIX3 SMNB, MATRIX3 REFSMMAT, double &pitch, double &yaw)
{
	//R = vector from LM to ground station

	MATRIX3 NBSA;
	VECTOR3 U_X, U_Y, U_Z, RP, U_RP, U_R, X, Z, YAW;
	double Y;

	NBSA = _M(cos(45.0*RAD), sin(45.0*RAD), 0, -sin(45.0*RAD), cos(45.0*RAD), 0, 0, 0, 1);
	R = mul(NBSA, mul(SMNB, mul(REFSMMAT, R))); //R in LM body axes

	U_R = unit(R);
	U_X = _V(1, 0, 0);
	U_Y = _V(0, 1, 0);
	U_Z = _V(0, 0, 1);

	RP = R - U_Y * dotp(R, U_Y);
	U_RP = unit(RP);

	X = crossp(U_Z, U_RP);
	pitch = asin(length(X)*OrbMech::sign(dotp(X, U_Y)));

	Y = dotp(U_RP, U_Z);
	if (Y < 0.0)
	{
		pitch = PI - pitch;
	}

	Z = crossp(U_R, U_RP);
	YAW = U_X * cos(pitch) - U_Z * sin(pitch);
	yaw = asin(length(Z)*OrbMech::sign(dotp(YAW, Z)));
}

void AGOP::RRAngles(VECTOR3 R, MATRIX3 SMNB, MATRIX3 REFSMMAT, double &trunnion, double &shaft)
{
	VECTOR3 u_D, u_P;

	//Pointing vector in body axes
	u_D = mul(SMNB, mul(REFSMMAT, unit(R)));

	u_P = unit(_V(u_D.x, 0, u_D.z));

	trunnion = -asin(u_D.y);
	if (trunnion < 0)
	{
		trunnion += PI2;
	}

	//Displayed trunnion the reverse of CDU trunnion
	trunnion = PI2 - trunnion;

	shaft = atan2(u_P.x, u_P.z);
	if (shaft < 0)
	{
		shaft += PI2;
	}
}

VECTOR3 AGOP::CSMIMUtoLMIMUAngles(MATRIX3 CSM_REFSMMAT, MATRIX3 LM_REFSMMAT, VECTOR3 CSMIMUAngles, double DockingAngle)
{
	MATRIX3 M_NBCSM_NBLM, M_SMCSM_NBCSM, M_BRCS_SMCSM, M_BRCS_SMLM, M_BRCS_NBLM;

	M_NBCSM_NBLM = OrbMech::CSMBodyToLMBody(DockingAngle);
	M_SMCSM_NBCSM = OrbMech::CALCSMSC(CSMIMUAngles);
	M_BRCS_SMCSM = CSM_REFSMMAT;
	M_BRCS_SMLM = LM_REFSMMAT;
	M_BRCS_NBLM = mul(mul(M_NBCSM_NBLM, M_SMCSM_NBCSM), M_BRCS_SMCSM);

	return OrbMech::CALCGAR(M_BRCS_SMLM, M_BRCS_NBLM);
}

VECTOR3 AGOP::LMIMUtoCMIMUAngles(MATRIX3 CSM_REFSMMAT, MATRIX3 LM_REFSMMAT, VECTOR3 LMIMUAngles, double DockingAngle)
{
	MATRIX3 M_NBCSM_NBLM, M_SMLM_NBLM, M_BRCS_SMCSM, M_BRCS_SMLM, M_BRCS_NBCSM;

	M_NBCSM_NBLM = OrbMech::CSMBodyToLMBody(DockingAngle);
	M_SMLM_NBLM = OrbMech::CALCSMSC(LMIMUAngles);
	M_BRCS_SMCSM = CSM_REFSMMAT;
	M_BRCS_SMLM = LM_REFSMMAT;
	M_BRCS_NBCSM = mul(OrbMech::tmat(M_NBCSM_NBLM), mul(M_SMLM_NBLM, M_BRCS_SMLM));

	return OrbMech::CALCGAR(M_BRCS_SMCSM, M_BRCS_NBCSM);
}

MATRIX3 AGOP::LVLHAttitude(VECTOR3 LVLHAtt, VECTOR3 R, VECTOR3 V)
{
	double SINP, SINY, SINR, COSP, COSY, COSR;
	SINP = sin(LVLHAtt.y);
	SINY = sin(LVLHAtt.z);
	SINR = sin(LVLHAtt.x);
	COSP = cos(LVLHAtt.y);
	COSY = cos(LVLHAtt.z);
	COSR = cos(LVLHAtt.x);

	VECTOR3 Z_P, Y_P, X_P;
	Z_P = -unit(R);
	Y_P = -unit(crossp(R, V));
	X_P = crossp(Y_P, Z_P);

	double AL, BE, a1, a2, a3, b1, b2, b3, c1, c2, c3;
	AL = SINP * SINR;
	BE = SINP * COSR;
	a1 = COSY * COSP;
	a2 = SINY * COSP;
	a3 = -SINP;
	b1 = AL * COSY - SINY * COSR;
	b2 = AL * SINY + COSY * COSR;
	b3 = COSP * SINR;
	c1 = BE * COSY + SINY * SINR;
	c2 = BE * SINY - COSY * SINR;
	c3 = COSP * COSR;

	VECTOR3 X_B, Y_B, Z_B;
	X_B = X_P * a1 + Y_P * a2 + Z_P * a3;
	Y_B = X_P * b1 + Y_P * b2 + Z_P * b3;
	Z_B = X_P * c1 + Y_P * c2 + Z_P * c3;

	return _M(X_B.x, X_B.y, X_B.z, Y_B.x, Y_B.y, Y_B.z, Z_B.x, Z_B.y, Z_B.z);
}

MATRIX3 AGOP::ThreeAxisPointing(VECTOR3 SCAXIS, VECTOR3 U_LOS, VECTOR3 R, VECTOR3 V, double OMICRON)
{
	//INPUT:
	//SCAXIS: Pointing direction in navigation base coordinates
	//U_LOS: Pointing direction in inertial coordinates
	//R: position vector in inertial coordinates
	//V: velocity vector in inertial coordinates
	//OMICRON: essentially LVLH roll angle

	//OUTPUT:
	//Navigation base orientation matrix

	//Math from Artemis 72 code

	VECTOR3 unitY, UTSB, UTSBP, UTSAP, POINTVSM, UTSA, UTUYP, UTUZP, UTUY, UTUZ;

	unitY = _V(0, 1, 0);

	UTSB = U_LOS;
	UTSBP = SCAXIS;
	UTSAP = unit(crossp(UTSBP, unitY)); //Error?
	POINTVSM = unit(crossp(V, R));
	POINTVSM = unit(crossp(UTSB, POINTVSM));

	UTSA = POINTVSM * cos(OMICRON) + unit(crossp(UTSB, POINTVSM))*sin(OMICRON);
	UTUYP = unit(crossp(UTSAP, UTSBP));
	UTUZP = crossp(UTSAP, UTUYP);
	UTUY = unit(crossp(UTSA, UTSB));
	UTUZ = crossp(UTSA, UTUY);

	return OrbMech::AXISGEN(UTUYP, UTUZP, UTUY, UTUZ);
}

VECTOR3 AGOP::GetBodyFixedHGAVector(double pitch, double yaw) const
{
	return _V(cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch));
}

VECTOR3 AGOP::GetBodyFixedSteerableAntennaVector(double pitch, double yaw) const
{
	MATRIX3 NBSA;

	NBSA = _M(cos(45.0*RAD), sin(45.0*RAD), 0, -sin(45.0*RAD), cos(45.0*RAD), 0, 0, 0, 1);

	return tmul(NBSA, _V(cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch)));
}

VECTOR3 AGOP::GetBodyFixedRRVector(double trunnion, double shaft) const
{
	//Displayed trunnion the reverse of CDU trunnion
	trunnion = PI2 - trunnion;

	return _V(sin(shaft)*cos(trunnion), -sin(trunnion), cos(shaft)*cos(trunnion));
}

VECTOR3 AGOP::GetStarUnitVector(const AGOPInputs &in, unsigned num)
{
	unsigned int ID;

	ID = in.StarIDs[num];

	if (ID <= 400U)
	{
		//From table
		return in.startable[ID - 1];
	}

	//Manual input
	return OrbMech::r_from_latlong(in.StarDeclination, in.StarRightAscension);
}

VECTOR3 AGOP::GetCSMCOASVector(double SPA, double SXP)
{
	//In navigation base coordinates
	return unit(_V(cos(SPA)*cos(SXP), sin(SXP), sin(SPA)*cos(SXP)));
}

VECTOR3 AGOP::GetLMCOASVector(double EL, double SXP, bool IsZAxis)
{
	//In navigation base coordinates
	if (IsZAxis)
	{
		return unit(_V(sin(SXP), -sin(EL)*cos(SXP), cos(EL)*cos(SXP)));
	}

	//X-axis
	return unit(_V(cos(EL)*cos(SXP), sin(SXP), sin(EL)*cos(SXP)));
}

VECTOR3 AGOP::GetAOTNBVector(double EL, double AZ, double ReticleAngle, double SpiraleAngle, int axis)
{
	VECTOR3 u_OAN, u_XPN_apo, u_YPN_apo, u_XPN, u_YPN;
	double RN;

	u_OAN = _V(sin(EL), cos(EL)*sin(AZ), cos(EL)*cos(AZ));
	u_YPN_apo = _V(0.0, cos(AZ), -sin(AZ));
	u_XPN_apo = crossp(u_YPN_apo, u_OAN);
	RN = 0.0 - AZ;
	u_XPN = u_XPN_apo * cos(RN) + u_YPN_apo * sin(RN);
	u_YPN = -u_XPN_apo * sin(RN) + u_YPN_apo * cos(RN);

	if (ReticleAngle == 0.0 && SpiraleAngle == 0.0)
	{
		// P52 technique
		
		return unit(crossp(u_XPN, u_YPN));
	}
	else
	{
		// P57 technique

		VECTOR3 u_YPN_aapo;
		double YROT, SROT, SEP;

		//1 = +Y-axis, 2 = +X-axis, 3 = -Y-axis, 4 = -X-axis
		switch (axis)
		{
		case 2: //+X
			YROT = ReticleAngle + 270.0*RAD;
			break;
		case 3: //-Y
			YROT = ReticleAngle + 180.0*RAD;
			break;
		case 4: //-X
			YROT = ReticleAngle + 90.0*RAD;
			break;
		default: //+Y
			YROT = ReticleAngle;
			break;
		}

		SROT = SpiraleAngle;

		SEP = (PI2 + SROT - YROT) / 12.0;

		u_YPN_aapo = -u_XPN * sin(YROT) + u_YPN * cos(YROT);
		return u_OAN * cos(SEP) + crossp(u_YPN_aapo, u_OAN)*sin(SEP);
	}
}