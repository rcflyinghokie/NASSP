/****************************************************************************
  This file is part of Project Apollo - NASSP

  Mission Control Center Vessel

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

#include "mccvessel.h"
#include "rtcc.h"
#include "mcc.h"
#include "paCBGmessageID.h"

#define ORBITER_MODULE

static int refcount = 0;

DLLCLBK void InitModule(HINSTANCE hDLL)
{
}

DLLCLBK void ExitModule(HINSTANCE hDLL)
{
}

DLLCLBK VESSEL* ovcInit(OBJHANDLE hVessel, int iFlightModel)
{
	if (!refcount++) {
		LoadMeshes();
	}

	return new MCCVessel(hVessel, iFlightModel);
}

DLLCLBK void ovcExit(VESSEL* pVessel)
{
    --refcount;

	delete static_cast<MCCVessel*>(pVessel);
}

MCCVessel::MCCVessel(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel)
{
	//Vessel data
	CSMName[0] = 0;
	LEMName[0] = 0;
	LVName[0] = 0;
	rtcc = new RTCC();
	mcc = NULL;
}

MCCVessel::~MCCVessel()
{
	if (rtcc)
	{
		delete rtcc;
		rtcc = NULL;
	}
	if (mcc)
	{
		delete mcc;
		mcc = NULL;
	}
}

void MCCVessel::clbkPreStep(double simt, double simdt, double mjd)
{
	// Update Ground Data
	if (mcc) mcc->TimeStep(simdt);
	if (rtcc) rtcc->Timestep(simt, simdt, mjd);
}

int MCCVessel::clbkGeneric(int msgid, int prm, void* context)
{
	if (!mcc) { return 0; }

	if (msgid == paCBGmessageID::messageID::RF_PROPERTIES) {

		RFCALC_RFProperties *RFtemp = (RFCALC_RFProperties*)context;

		if (prm == paCBGmessageID::parameterID::GetCM || prm == paCBGmessageID::parameterID::GetLM)
		{
			int slot;

			if (prm == paCBGmessageID::parameterID::GetCM)
			{
				slot = MCC::TrackingSlot::SlotCM;
			}
			else
			{
				slot = MCC::TrackingSlot::SlotLM;
			}

			RFtemp->GlobalPosition = (mcc->TransmittingGroundStationVector[slot]);

			if (mcc->GroundStations[mcc->TransmittingGroundStation[slot]].SBandAntenna & GSSA_26METER) {
				RFtemp->Gain = pow(10.0, (50.0 / 10.0));
				RFtemp->Power = 20.0E3;
			}
			else if (mcc->GroundStations[mcc->TransmittingGroundStation[slot]].SBandAntenna & GSSA_9METER) {
				RFtemp->Gain = pow(10.0, (43.0 / 10.0));
				RFtemp->Power = 10E3;
			}
			else if (mcc->GroundStations[mcc->TransmittingGroundStation[slot]].SBandAntenna & GSSA_3PT7METER) {
				RFtemp->Gain = pow(10.0, (37.0 / 10.0));
				RFtemp->Power = 22.9;
			}
			else {
				RFtemp->Gain = 1.0;
				RFtemp->Power = 0.0;
			}

			return 1;
		}
	}

	return 0;

}

void MCCVessel::clbkSaveState(FILEHANDLE scn)
{
	VESSEL4::clbkSaveState(scn);

	if (CSMName[0])
		oapiWriteScenario_string(scn, "CSMNAME", CSMName);

	if (LVName[0])
		oapiWriteScenario_string(scn, "LVNAME", LVName);

	if (LEMName[0])
		oapiWriteScenario_string(scn, "LEMNAME", LEMName);

	if (mcc) mcc->SaveState(scn);
	if (rtcc) rtcc->SaveState(scn);
}

void MCCVessel::clbkLoadStateEx(FILEHANDLE scn, void *status)
{
	char *line;

	while (oapiReadScenario_nextline(scn, line)) {
		if (!_strnicmp(line, "MISSIONTRACKING", 15)) {
			int i;
			sscanf(line + 15, "%d", &i);
			if (i)
			{
				CreateMCC();
				mcc->enableMissionTracking();
			}
		}
		else if (!_strnicmp(line, "CSMNAME", 7))
		{
			strncpy(CSMName, line + 8, 64);
		}
		else if (!_strnicmp(line, "LVNAME", 6))
		{
			strncpy(LVName, line + 7, 64);
		}
		else if (!_strnicmp(line, "LEMNAME", 7))
		{
			strncpy(LEMName, line + 8, 64);
		}
		else if (!_strnicmp(line, MCC_START_STRING, sizeof(MCC_START_STRING)))
		{
			CreateMCC();
			if (mcc) mcc->LoadState(scn);
		}
		else if (!_strnicmp(line, RTCC_START_STRING, sizeof(RTCC_START_STRING))) {
			if (rtcc) rtcc->LoadState(scn);
		}
		else ParseScenarioLineEx(line, status);
	}
}

void MCCVessel::clbkSetClassCaps(FILEHANDLE cfg)
{
	LoadVC();
}

void MCCVessel::clbkPostCreation()
{
	//CSM
	if (CSMName[0])
	{
		if (mcc) mcc->SetCSM(CSMName);
	}
	//S-IVB
	if (LVName[0])
	{
		if (mcc) mcc->SetLV(LVName);
	}
	//LEM
	if (LEMName[0])
	{
		if (mcc) mcc->SetLM(LEMName);
	}
	if (rtcc) rtcc->clbkPostCreation();
}

void MCCVessel::CreateMCC()
{
	if (mcc == NULL)
	{
		mcc = new MCC(rtcc);
	}
}