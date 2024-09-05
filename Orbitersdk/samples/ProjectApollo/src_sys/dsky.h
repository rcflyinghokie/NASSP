/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005



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

//Network code slightly modified from here
//https://github.com/jchamet/cpp-send-udp

#pragma once

///
/// \ingroup DSKY
/// DSKY simulation.
/// \brief Generic DSKY simulation class.
///

#include "toggleswitch.h"
#include "apolloguidance.h"

class DSKY : public e_object

{
public:

	DSKY(SoundLib &s, ApolloGuidance &computer, int IOChannel = 015);
	virtual ~DSKY();

	void Init(
		e_object *statuslightpower, 
		e_object *segmentlightpower, 
		ContinuousRotationalSwitch *dimmer, 
		ContinuousRotationalSwitch *integralDimmer,
		ToggleSwitch *anunOverride,
		ToggleSwitch *integralOverride
	);
	void Reset();

	//
	// Light status.
	//

	bool UplinkLit()	{ return UplinkLight; };
	bool CompActyLit()	{ return CompActy; };
	bool NoAttLit()		{ return NoAttLight; };
	bool StbyLit()		{ return StbyLight; };
	bool KbRelLit()		{ return KbRelLight; };
	bool OprErrLit()	{ return OprErrLight; };
	bool TempLit()		{ return TempLight; };
	bool GimbalLockLit() { return GimbalLockLight; };
	bool ProgLit()		{ return ProgLight; };
	bool RestartLit()	{ return RestartLight; };
	bool TrackerLit()	{ return TrackerLight; };
	bool VelLit()		{ return VelLight; };
	bool AltLit()		{ return AltLight; };
	bool NoDAPLit()		{ return NoDAPLight; };
	bool PrioDispLit()	{ return PrioDispLight; };

	//
	// Set light status.
	//

	void LightStby()		{ StbyLight = true; };
	void LightRestart()		{ RestartLight = true; };

	void SetUplink(bool val)		{ UplinkLight = val; };
	void SetCompActy(bool val)		{ CompActy = val; };
	void SetNoAtt(bool val)			{ NoAttLight = val; };
	void SetStby(bool val)			{ StbyLight = val; };
	void SetKbRel(bool val)			{ KbRelLight = val; };
	void SetOprErr(bool val)		{ OprErrLight = val; };
	void SetTemp(bool val)			{ TempLight = val; };
	void SetGimbalLock(bool val)	{ GimbalLockLight = val; };
	void SetProg(bool val)			{ ProgLight = val; };
	void SetRestart(bool val)		{ RestartLight = val; };
	void SetTracker(bool val)		{ TrackerLight = val; };
	void SetVel(bool val)			{ VelLight = val; };
	void SetAlt(bool val)			{ AltLight = val; };
	void SetPrioDisp(bool val)		{ PrioDispLight = val; };
	void SetNoDAP(bool val)			{ NoDAPLight = val; };

	void ClearStby()		{ StbyLight = false; };
	void ClearRestart()		{ RestartLight = false; };

	//
	// Flashing status.
	//

	void SetVerbDisplayFlashing()	{ VerbFlashing = true; };
	void SetNounDisplayFlashing()	{ NounFlashing = true; };

	void ClearVerbDisplayFlashing() { VerbFlashing = false; };
	void ClearNounDisplayFlashing() { NounFlashing = false; };

	//
	// Timestep to run programs.
	//

	void Timestep(double simt);
	void SystemTimestep(double simdt);

	//
	// Keypad interface.
	//

	void KeyRel();
	void VerbPressed();
	void NounPressed();
	void EnterPressed();
	void ClearPressed();
	void ResetPressed();
	void ProceedPressed();
	void ProceedReleased();
	void PlusPressed();
	void MinusPressed();
	void NumberPressed(int n);

	void VerbCallback(PanelSwitchItem* s);
	void NounCallback(PanelSwitchItem* s);
	void EnterCallback(PanelSwitchItem* s);
	void ClearCallback(PanelSwitchItem* s);
	void ResetCallback(PanelSwitchItem* s);
	void ProceedCallback(PanelSwitchItem* s);
	void KeyRelCallback(PanelSwitchItem* s);
	void PlusCallback(PanelSwitchItem* s);
	void MinusCallback(PanelSwitchItem* s);
	void zeroCallback(PanelSwitchItem* s);
	void oneCallback(PanelSwitchItem* s);
	void twoCallback(PanelSwitchItem* s);
	void threeCallback(PanelSwitchItem* s);
	void fourCallback(PanelSwitchItem* s);
	void fiveCallback(PanelSwitchItem* s);
	void sixCallback(PanelSwitchItem* s);
	void sevenCallback(PanelSwitchItem* s);
	void eightCallback(PanelSwitchItem* s);
	void nineCallback(PanelSwitchItem* s);

	void ProcessKeyPress(int mx, int my);
	void ProcessKeyRelease(int mx, int my);
	void RenderLights(SURFHANDLE surf, SURFHANDLE lights, int xoffset = 0, int yoffset = 0, bool hasAltVel = true, bool hasDAPPrioDisp = false, int xTexMul = 1);
	void RenderData(SURFHANDLE surf, SURFHANDLE digits, SURFHANDLE disp, int xoffset = 0, int yoffset = 0, int xTexMul = 1);
	void RenderKeys(SURFHANDLE surf, SURFHANDLE keys, int xoffset = 0, int yoffset = 0);
	void ProcessChannel10(ChannelValue val);
	void ProcessChannel13(ChannelValue val);
	void ProcessChannel11Bit(int bit, bool val);
	void ProcessChannel11(ChannelValue val);
	void ProcessChannel163(ChannelValue val);

	void SaveState(FILEHANDLE scn, char *start_str, char *end_str);
	void LoadState(FILEHANDLE scn, char *end_str);

	void SendNetworkPacketDSKY();

	char *GetProg() { return Prog; };
	char *GetVerb() { return Verb; };
	char *GetNoun() { return Noun; };
	char *GetR1() { return R1; };
	char *GetR2() { return R2; };
	char *GetR3() { return R3; };

protected:

	bool IsStatusPowered();
	bool IsSegmentPowered();
	void SendKeyCode(int val);

	//
	// Lights.
	//

	void LightOprErrLight()		{ OprErrLight = true; };
	void ClearOprErrLight()		{ OprErrLight = false; };

	//
	// Light power consumption.
	//

	int LightsLit;
	int SegmentsLit;

	//
	// Lights state.
	//

	bool CompActy;
	bool UplinkLight;
	bool NoAttLight;
	bool StbyLight;
	bool KbRelLight;
	bool OprErrLight;
	bool TempLight;
	bool GimbalLockLight;
	bool ProgLight;
	bool RestartLight;
	bool TrackerLight;
	bool VelLight;
	bool AltLight;
	bool PrioDispLight;
	bool NoDAPLight;

	//
	// Keyboard state.
	//

	bool KbInUse;

	bool KeyDown_Verb;
	bool KeyDown_Noun;
	bool KeyDown_Plus;
	bool KeyDown_Minus;
	bool KeyDown_0;
	bool KeyDown_1;
	bool KeyDown_2;
	bool KeyDown_3;
	bool KeyDown_4;
	bool KeyDown_5;
	bool KeyDown_6;
	bool KeyDown_7;
	bool KeyDown_8;
	bool KeyDown_9;
	bool KeyDown_Clear;
	bool KeyDown_Prog;
	bool KeyDown_KeyRel;
	bool KeyDown_Enter;
	bool KeyDown_Reset;

	//
	// Current program state.
	//

	int CurrentVerb;
	int CurrentNoun;

	//
	// Internal variables.
	//

	char Prog[3];
	char Verb[3];
	char Noun[3];
	char R1[7];
	char R2[7];
	char R3[7];

	bool VerbFlashing;
	bool NounFlashing;
	bool ELOff;

	int	EnterPos;
	int EnterVal;

	//
	// AGC we're connected to.
	//

	ApolloGuidance &agc;

	//
	// I/O channel to use for key-codes.
	//

	int KeyCodeIOChannel;

	//
	// Sound library.
	//

	SoundLib &soundlib;
	Sound Sclick;

	bool FirstTimeStep;
	e_object *StatusPower;
	e_object *SegmentPower;
	ToggleSwitch *LtgORideAnunSwitch;
	ToggleSwitch *LtgORideIntegralSwitch;
	ContinuousRotationalSwitch *DimmerRotationalSwitch;
	ContinuousRotationalSwitch *IntegralRotationalSwitch;

	//
	//DSKY output stuff
	//

	bool DSKYOutEnabled;
	char DSKYOutIp[256];
	int DSKYOutPort;
	WSADATA wsaData;
	struct sockaddr_in serverAddr;
	int clientSock;
	
	//
	// Local helper functions.
	//

	char ValueChar(unsigned val);
	void KeyClick();
	void ResetKeyDown();

	void DSKYLightBlt(SURFHANDLE surf, SURFHANDLE lights, int dstx, int dsty, bool lit, int xOffset, int yOffset, int xTexMul);
	void DSKYKeyBlt(SURFHANDLE surf, SURFHANDLE keys, int dstx, int dsty, int srcx, int srcy, bool lit, int xOffset, int yOffset); 
	void RenderTwoDigitDisplay(SURFHANDLE surf, SURFHANDLE digits, int dstx, int dsty, char *Str, bool Flash, bool Off, int xTexMul = 1);
	void RenderSixDigitDisplay(SURFHANDLE surf, SURFHANDLE digits, int dstx, int dsty, char *Str, bool Off, int xTexMul = 1);
	int TwoDigitDisplaySegmentsLit(char *Str, bool Flash, bool Off);
	int SixDigitDisplaySegmentsLit(char *Str, bool Off);
};

//
// Strings for state saving.
//

#define DSKY_START_STRING	"DSKY_BEGIN"
#define DSKY_END_STRING		"DSKY_END"

#define DSKY2_START_STRING	"DSKY2_BEGIN"
#define DSKY2_END_STRING	"DSKY2_END"

inline const char* const B2S(bool b)
{
	return b ? "1" : "0";
}
