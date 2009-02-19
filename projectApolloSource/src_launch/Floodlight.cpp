/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  Floodlight vessel

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

  **************************** Revision History ****************************
  *	$Log$
  *	Revision 1.5  2008/04/11 11:49:10  tschachim
  *	Fixed BasicExcel for VC6, reduced VS2005 warnings, bugfixes.
  *	
  *	Revision 1.4  2007/08/13 16:06:03  tschachim
  *	Moved bitmaps to subdirectory.
  *	New VAGC mission time pad load handling.
  *	New telescope and sextant panels.
  *	Fixed CSM/LV separation speed.
  *	
  *	Revision 1.3  2007/06/06 15:02:07  tschachim
  *	OrbiterSound 3.5 support, various fixes and improvements.
  *	
  *	Revision 1.2  2006/07/24 19:24:50  tschachim
  *	Bugfix
  *	
  *	Revision 1.1  2006/07/17 19:33:36  tschachim
  *	Small improvements of LC39.
  *
  **************************************************************************/

#define ORBITER_MODULE

// To force orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "orbitersdk.h"
#include "stdio.h"
#include "math.h"
#include "nasspsound.h"
#include "OrbiterSoundSDK35.h"
#include "soundlib.h"
#include "tracer.h"
#include "Floodlight.h"
#include "papi.h"

HINSTANCE g_hDLL;
char trace_file[] = "ProjectApollo Floodlight.log";
 

DLLCLBK void InitModule(HINSTANCE hModule) {

	g_hDLL = hModule;
}


DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel) {

	return new Floodlight(hvessel, flightmodel);
}


DLLCLBK void ovcExit(VESSEL *vessel) {

	if (vessel) delete (Floodlight*)vessel;
}


Floodlight::Floodlight(OBJHANDLE hObj, int fmodel) : VESSEL2 (hObj, fmodel) {

	firstTimestepDone = false;
	touchdownPointHeight = 0;

	int i;
	for (i = 0; i < MAXEXHAUST; i++) {
		exhaust[i] = -1;
		exhaustPos[i] = _V(0, 0, 0);
		exhaustDir[i][0] = 0;
		exhaustDir[i][1] = 0;
		exhaustSize[i][0] = 50;
		exhaustSize[i][1] = 5;
	}
	exhausts = 0;
	exhaustsEnabled = false;
	currentExhaust = 0;
	configMode = 0;

	soundlib.InitSoundLib(hObj, SOUND_DIRECTORY);
}

Floodlight::~Floodlight() {
}

void Floodlight::clbkSetClassCaps(FILEHANDLE cfg) {

	SetEmptyMass(100);
	SetSize(1000);

    ClearMeshes();
	ClearThrusterDefinitions();
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	tex = oapiRegisterExhaustTexture("ProjectApollo\\Floodlight");
	ph = CreatePropellantResource(100.0);
	th = CreateThruster (_V(0, 0, 0), _V(0, 0, -1), 0.0 , ph);
	SetThrusterLevel(th, 1.0);
	SetPropellantMass(ph, 100.0);

	SetTouchdownPointHeight(touchdownPointHeight);
}

void Floodlight::clbkPostCreation() {
}

void Floodlight::clbkPreStep(double simt, double simdt, double mjd) {

	int i;

	if (!firstTimestepDone) DoFirstTimestep();
	
	SetThrusterLevel(th, 1.0);
	SetPropellantMass(ph, 100.0);

	// Day/night handling
	VECTOR3 globalPos, localPos, horizonPos;
	OBJHANDLE sun = oapiGetGbodyByName("Sun");
	oapiGetGlobalPos(sun, &globalPos);
	Global2Local(globalPos, localPos);
	HorizonRot(localPos, horizonPos);
	double angle = asin(horizonPos.y / length(horizonPos)); 

	// Enable lights during config mode
	if (configMode) angle = -1;
	// Turn lights on or off
	if (angle > 0 && exhaustsEnabled) {
		for (i = 0; i < exhausts; i++) {
			if (exhaust[i] != -1) 
				DelExhaust(exhaust[i]);
			exhaust[i] = -1;
		}
		exhaustsEnabled = false;
	}
	else if (angle <= 0 && !exhaustsEnabled) {
		for (i = 0; i < exhausts; i++) {
			SetExhaust(i);
		}
		exhaustsEnabled = true;
	}

	if (configMode)
		sprintf(oapiDebugString(), "Light %d [Usage: J Change, NUM 8/2/4/6/3/1 Move, NUM 7/9/+/- Direction, NUM 0/,/�/* Size]", currentExhaust);
}

void Floodlight::clbkPostStep (double simt, double simdt, double mjd) {

	if (!firstTimestepDone) return;

}

void Floodlight::DoFirstTimestep() {

	soundlib.SoundOptionOnOff(PLAYCOUNTDOWNWHENTAKEOFF, FALSE);
	soundlib.SoundOptionOnOff(PLAYCABINAIRCONDITIONING, FALSE);
	soundlib.SoundOptionOnOff(PLAYCABINRANDOMAMBIANCE, FALSE);
	soundlib.SoundOptionOnOff(PLAYRADARBIP, FALSE);
	soundlib.SoundOptionOnOff(DISPLAYTIMER, FALSE);

	firstTimestepDone = true;
}

void Floodlight::SetExhaust(int index) {

	if (exhaust[index] != -1) 
		DelExhaust(exhaust[index]);

	VECTOR3 dir = mul(GetRotationMatrixY(exhaustDir[index][0]), mul(GetRotationMatrixZ(exhaustDir[index][1]), _V(1, 0, 0)));
	exhaust[index] = AddExhaust(th, exhaustSize[index][0], exhaustSize[index][1], exhaustPos[index], dir, tex);
}

MATRIX3 Floodlight::GetRotationMatrixY(double angle) {
	// Returns the rotation matrix for a rotation of a given angle around the Y axis (Yaw)

	MATRIX3 RotMatrixY;
	
	RotMatrixY.m11 = cos(angle);
	RotMatrixY.m12 = 0;
	RotMatrixY.m13 = sin(angle);
	RotMatrixY.m21 = 0;
	RotMatrixY.m22 = 1;
	RotMatrixY.m23 = 0;
	RotMatrixY.m31 = -sin(angle);
	RotMatrixY.m32 = 0;
	RotMatrixY.m33 = cos(angle);
	
	return RotMatrixY;
}

MATRIX3 Floodlight::GetRotationMatrixZ(double angle) {
	// Returns the rotation matrix for a rotation of a given angle around the Z axis (Roll)

	MATRIX3 RotMatrixZ;
	
	RotMatrixZ.m11 = cos(angle);
	RotMatrixZ.m12 = -sin(angle);
	RotMatrixZ.m13 = 0;
	RotMatrixZ.m21 = sin(angle);
	RotMatrixZ.m22 = cos(angle);
	RotMatrixZ.m23 = 0;
	RotMatrixZ.m31 = 0;
	RotMatrixZ.m32 = 0;
	RotMatrixZ.m33 = 1;
	
	return RotMatrixZ;	
}

void Floodlight::SetTouchdownPointHeight(double height) {

	touchdownPointHeight = height;
	SetTouchdownPoints(_V(  0, touchdownPointHeight,  10), 
					   _V(-10, touchdownPointHeight, -10), 
					   _V( 10, touchdownPointHeight, -10));
}

void Floodlight::clbkLoadStateEx(FILEHANDLE scn, void *status) {

	int i;
	char *line;

	while (oapiReadScenario_nextline (scn, line)) {
		if (!strnicmp (line, "TOUCHDOWNPOINTHEIGHT", 20)) {
			sscanf (line + 20, "%lf", &touchdownPointHeight);
		}
		else if (!strnicmp (line, "CONFIGMODE", 10)) {
			sscanf (line + 10, "%d", &configMode);
		}
		else if (!strnicmp (line, "LIGHTS", 6)) {
			sscanf (line + 6, "%d", &exhausts);
		}
		else if (!strnicmp (line, "LIGHT", 5)) {
			sscanf (line + 5, "%d", &i);
			if (i < MAXEXHAUST) {
				sscanf (line + 5, "%d %lf %lf %lf %lf %lf %lf %lf", &i, 
					&exhaustPos[i].x, &exhaustPos[i].y, &exhaustPos[i].z,
					&exhaustDir[i][0], &exhaustDir[i][1],
					&exhaustSize[i][0], &exhaustSize[i][1]);
			}
		}
		else {
			ParseScenarioLineEx (line, status);
		}
	}
	SetTouchdownPointHeight(touchdownPointHeight);
	if (exhausts >= MAXEXHAUST)
		exhausts = MAXEXHAUST - 1;
}

void Floodlight::clbkSaveState(FILEHANDLE scn) {

	int i;
	char buffer[100];

	VESSEL2::clbkSaveState(scn);

	papiWriteScenario_double(scn, "TOUCHDOWNPOINTHEIGHT", touchdownPointHeight);
	oapiWriteScenario_int(scn, "CONFIGMODE", configMode);
	oapiWriteScenario_int(scn, "LIGHTS", exhausts);
	
	for (i = 0; i < exhausts; i++) {
		sprintf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf", i, 
			exhaustPos[i].x, exhaustPos[i].y, exhaustPos[i].z,
			exhaustDir[i][0], exhaustDir[i][1],
			exhaustSize[i][0], exhaustSize[i][1]); 
		oapiWriteScenario_string(scn, "LIGHT", buffer);
	}
}

int Floodlight::clbkConsumeDirectKey(char *kstate) {

	if (KEYMOD_SHIFT(kstate)) 
		return 0; 

	if (!configMode)
		return 0;
	
	double moveStep = 1.0;
	double turnStep = 1.0 * RAD;
	if (KEYMOD_CONTROL(kstate)) {
		moveStep = 0.1;
		turnStep = 0.1 * RAD;
	}

	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD2)) {
		exhaustPos[currentExhaust].x -= moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD2);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD4)) {			
		exhaustPos[currentExhaust].z += moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD4);			
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD6)) {			
		exhaustPos[currentExhaust].z -= moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD6);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD8)) {
		exhaustPos[currentExhaust].x += moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD8);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD1)) {
		exhaustPos[currentExhaust].y -= moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD1);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD3)) {
		exhaustPos[currentExhaust].y += moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD3);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD9)) {
		exhaustDir[currentExhaust][0] += turnStep;
		if (exhaustDir[currentExhaust][0] >= 2.0 * PI)
			exhaustDir[currentExhaust][0] -= 2.0 * PI;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD9);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD7)) {
		exhaustDir[currentExhaust][0] -= turnStep;
		if (exhaustDir[currentExhaust][0] < 0)
			exhaustDir[currentExhaust][0] += 2.0 * PI;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD7);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_ADD)) {
		exhaustDir[currentExhaust][1] += turnStep;
		if (exhaustDir[currentExhaust][1] > PI05)
			exhaustDir[currentExhaust][1] = PI05;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_ADD);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_SUBTRACT)) {
		exhaustDir[currentExhaust][1] -= turnStep;
		if (exhaustDir[currentExhaust][1] < -PI05)
			exhaustDir[currentExhaust][1] = -PI05;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_SUBTRACT);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_DECIMAL)) {
		exhaustSize[currentExhaust][0] += moveStep;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_DECIMAL);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD0)) {
		exhaustSize[currentExhaust][0] -= moveStep;
		if (exhaustSize[currentExhaust][0] < 0)
			exhaustSize[currentExhaust][0] = 0;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_NUMPAD0);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_MULTIPLY)) {
		exhaustSize[currentExhaust][1] += moveStep / 10.0;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_MULTIPLY);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_DIVIDE)) {
		exhaustSize[currentExhaust][1] -= moveStep / 10.0;
		if (exhaustSize[currentExhaust][1] < 0)
			exhaustSize[currentExhaust][1] = 0;
		SetExhaust(currentExhaust);
		RESETKEY(kstate, OAPI_KEY_DIVIDE);						
	}
	return 0;
}

int Floodlight::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate) {

	if (KEYMOD_SHIFT(kstate) || KEYMOD_CONTROL(kstate)) {
		return 0;
	}

	if (key == OAPI_KEY_J && down == true) {
		currentExhaust++;
		if (currentExhaust >= exhausts)
			currentExhaust = 0;
		return 1;
	}
	return 0;
}