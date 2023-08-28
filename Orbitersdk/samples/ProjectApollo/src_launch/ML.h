/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  ML vessel

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

#pragma once

#include "soundlib.h"
#include "IUUmbilicalInterface.h"
#include "TailUmbilicalInterface.h"
#include "LCCPadInterface.h"

class Saturn;
class IUUmbilical;
class TailUmbilical;
class IUSV_ESE;
class SI_ESE;
class RCA110AM;

const double ML_SIC_INTERTANK_ARM_CONNECTING_SPEED = 1.0 / 300.0;
const double ML_SIC_INTERTANK_ARM_RETRACT_SPEED = 1.0 / 13.0;
const double ML_SIC_FORWARD_ARM_CONNECTING_SPEED = 1.0 / 300.0;
const double ML_SIC_FORWARD_ARM_RETRACT_SPEED = 1.0 / 5.2;
const double ML_SWINGARM_CONNECTING_SPEED = 1.0 / 200.0;
const double ML_SWINGARM_RETRACT_SPEED = 1.0 / 5.0;
const double ML_TAIL_SERVICE_MAST_CONNECTING_SPEED = 1.0 / 100.0;
const double ML_TAIL_SERVICE_MAST_RETRACT_SPEED = 1.0 / 2.0;
const double ML_TAIL_SERVICE_MAST_COVERS_CONNECTING_SPEED = 1.0 / 100.0;
const double ML_TAIL_SERVICE_MAST_COVERS_RETRACT_SPEED = 1.0 / 1;
const double DAMPERARM_CONNECTING_SPEED = 1.0 / 1000.0;
const double DAMPERARM_RETRACT_SPEED = 1.0 / 30.0;

///
/// \ingroup Ground
///
class ML: public VESSEL2, public IUUmbilicalInterface, public TailUmbilicalInterface, public LCCPadInterface {

public:
	ML(OBJHANDLE hObj, int fmodel);
	virtual ~ML();

	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkPostCreation();
	void clbkLoadStateEx(FILEHANDLE scn, void *status);
	void clbkSaveState(FILEHANDLE scn);
	int clbkConsumeDirectKey(char *kstate);
	int clbkConsumeBufferedKey(DWORD key, bool down, char *kstate);
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkPostStep(double simt, double simdt, double mjd);

	virtual void SetVABBuildState();
	virtual void SetVABReadyState();
	virtual bool Detach();
	virtual bool Attach();
	virtual bool IsInVAB(); 

	// ML/IU Interface
	bool ESEGetCommandVehicleLiftoffIndicationInhibit();
	bool ESEGetSICOutboardEnginesCantInhibit();
	bool ESEGetSICOutboardEnginesCantSimulate();
	bool ESEGetExcessiveRollRateAutoAbortInhibit(int n);
	bool ESEGetExcessivePitchYawRateAutoAbortInhibit(int n);
	bool ESEGetTwoEngineOutAutoAbortInhibit(int n);
	bool ESEGetGSEOverrateSimulate(int n);
	bool ESEGetEDSPowerInhibit();
	bool ESEPadAbortRequest();
	bool ESEGetThrustOKIndicateEnableInhibitA();
	bool ESEGetThrustOKIndicateEnableInhibitB();
	bool ESEEDSLiftoffInhibitA();
	bool ESEEDSLiftoffInhibitB();
	bool ESEGetSIBurnModeSubstitute();
	bool ESEGetGuidanceReferenceRelease();
	bool ESEGetQBallSimulateCmd();
	bool ESEGetEDSAutoAbortSimulate(int n);
	bool ESEGetEDSLVCutoffSimulate(int n);

	//ML/S-IC Interface
	bool ESEGetSIThrustOKSimulate(int eng, int n);

	// LCC/ML Interface
	void SLCCCheckDiscreteInput(RCA110A *c);
	bool SLCCGetOutputSignal(size_t n);
	void ConnectGroundComputer(RCA110A *c);
	void IssueSwitchSelectorCmd(int stage, int chan);

protected:
	bool firstTimestepDone;
	int meshindexML;
	bool moveToPadA;
	bool moveToPadB;
	bool moveToVab;
	bool moveLVToPadA;
	bool moveLVToPadB;
	double touchdownPointHeight;
	char LVName[256];
	SoundLib soundlib;
	OBJHANDLE hLV;
	int state;
	bool IsSaturnV;

	UINT craneAnim;
	UINT cmarmAnim;
	UINT s2intermediatearmAnim;
	UINT s2aftarmAnim;
	UINT damperarmAnim;
	UINT s1cintertankarmAnim;
	UINT s1cforwardarmAnim;
	UINT swingarmAnim;
	UINT mastAnim;
	UINT mastcoversAnim;
	double craneProc;
	double cmarmProc;
	AnimState s2intermediatearmState;
	AnimState s2aftarmState;
	AnimState damperarmState;
	AnimState s1cintertankarmState;
	AnimState s1cforwardarmState;
	AnimState swingarmState;
	AnimState mastState;
	AnimState mastcoversState;

	PSTREAM_HANDLE liftoffStream[2];
	double liftoffStreamLevel;

	Saturn *sat;
	IUUmbilical *IuUmb;
	TailUmbilical *TailUmb;
	IUSV_ESE *IuESE;
	SI_ESE *SIESE;
	RCA110AM *rca110a;

	void DoFirstTimestep();
	double GetDistanceTo(double lon, double lat);
	void SetTouchdownPointHeight(double height);
	void DefineAnimations();
	void DefineSaturnIBAnimations(); //With milkstool
	void DefineSaturnVAnimations();
	void SetAnimations(double simdt);

	void OpenInflightSwingarms(); //At commit

	bool CutoffInterlock();
	bool Commit();
	void SaturnIBIgnitionSequence(double MissionTime);
	void SaturnVIgnitionSequence(double MissionTime);
	void HoldDownForce(double MissionTime);
	void LiftoffStream(double MissionTime);

	void MobileLauncherComputer(int mdo, bool on = true);

	void TerminalCountdownSequencer(double MissionTime);

	int TCSSequence;
	bool Hold;
	bool bCommit;
};
