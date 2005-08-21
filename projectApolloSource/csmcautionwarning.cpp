/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Mark Grant

  ORBITER vessel module: CSM caution and warning system code.

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
  *	Revision 1.4  2005/08/21 11:51:59  movieman523
  *	Initial version of CSM caution and warning lights: light test switch now works.
  *	
  *	Revision 1.3  2005/08/13 23:48:57  movieman523
  *	Added some documentation on caution and warning checks that I found on the web.
  *	
  *	Revision 1.2  2005/08/13 16:41:15  movieman523
  *	Fully wired up the CSM caution and warning switches.
  *	
  *	Revision 1.1  2005/08/13 14:59:24  movieman523
  *	Added initial null implementation of CSM caution and warning system, and removed 'master alarm' flag from Saturn class.
  *	
  **************************************************************************/

#include "Orbitersdk.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "OrbiterSoundSDK3.h"
#include "soundlib.h"

#include "resource.h"

#include "nasspdefs.h"
#include "nasspsound.h"

#include "toggleswitch.h"
#include "apolloguidance.h"
#include "dsky.h"
#include "csmcomputer.h"
#include "IMU.h"

#include "saturn.h"

CSMCautionWarningSystem::CSMCautionWarningSystem(Sound &s) : CautionWarningSystem(s)

{
	NextUpdateTime = MINUS_INFINITY;
	TimeStepCount = 0;
}

//
// Check status of a fuel cell.
//

bool CSMCautionWarningSystem::FuelCellBad(double temp)

{
	//
	// I don't know what the real temperature limits are.
	//

	if (temp < 150.0 || temp > 500.0)
		return true;

	return false;
}

//
// We'll only check the internal systems five times a second (x time acceleration). That should cut the overhead to
// a minimum.
//

void CSMCautionWarningSystem::TimeStep(double simt)

{
	CautionWarningSystem::TimeStep(simt);

	//
	// Skip the first few timesteps to give the internal systems time to
	// settle down after a scenario load. This should avoid spurious
	// warnings.
	//

	if (TimeStepCount < 10) {
		TimeStepCount++;
		return;
	}

	if (simt > NextUpdateTime) {

		Saturn *sat = (Saturn *) OurVessel;

		//
		// Check systems.
		//

		FuelCellStatus fc;

		sat->GetFuelCellStatus(fc);

		//
		// We should check more than temperature, once we find out what the
		// caution limits were for pressure, etc.
		//

		SetLight(CSM_CWS_FC1_LIGHT, FuelCellBad(fc.FC1Temp));
		SetLight(CSM_CWS_FC2_LIGHT, FuelCellBad(fc.FC2Temp));
		SetLight(CSM_CWS_FC3_LIGHT, FuelCellBad(fc.FC3Temp));

		//
		// LOX/LH2: "The caution and warning system will activate on alarm when oxygen pressure 
		// in either tank exceeds 950 psia or falls below 800 psia or when the hydrogen system 
		// pressure exceeds 270 psia or drops below 220 psia."
		//

		bool LightCryo = false;
		TankPressures press;

		sat->GetTankPressures(press);

		if (press.H2Tank1Pressure < 220.0 || press.H2Tank2Pressure < 220.0) {
			LightCryo = true;
		}
		else if (press.H2Tank1Pressure > 270.0 || press.H2Tank2Pressure > 270.0) {
			LightCryo = true;
		}
		else if (press.O2Tank1Pressure < 800.0 || press.O2Tank2Pressure < 800.0) {
			LightCryo = true;
		}
		else if (press.O2Tank1Pressure > 950.0 || press.O2Tank2Pressure > 950.0) {
			LightCryo = true;
		}

		SetLight(CSM_CWS_CRYO_PRESS_LIGHT, LightCryo);

		//
		// Inverter: "A temperature sensor with a range of 32 degrees to 248 degrees F is installed 
		// in each inverter and will illuminate a light in the caution and warning system at an 
		// inverter overtemperature of 190 degrees F"
		//

		//
		// Power Bus: "If voltage drops below 26.25 volts dc, the applicable dc undervoltage light 
		// on the caution and warning panel will illuminate."
		//

		//
		// Oxygen flow: "Flow rates of 1 pound per hour or more with a duration in excess of 16.5 
		// seconds will illuminate a light on the caution and warning panel to alert the crew to 
		// the fact that the oxygen flow rate is greater than is normally required."
		//

		//
		// CO2: "A carbon dioxide sensor is connected between the suit inlet and return manifold. It 
		// is connected to an indicator on the main display console, to telemetry, and to the caution 
		// and warning system and will activate a warning if the carbon dioxide partial pressure 
		// reaches 7.6 millimeters of mercury."
		//

		SetLight(CSM_CWS_CO2_LIGHT, (sat->GetCO2Level() >= 7.6));

		NextUpdateTime = simt + (0.2 * oapiGetTimeAcceleration());
	}
}

void CSMCautionWarningSystem::RenderLights(SURFHANDLE surf, SURFHANDLE lightsurf, bool leftpanel)

{
	if (leftpanel) {
		RenderLightPanel(surf, lightsurf, LeftLights, TestState == CWS_TEST_LIGHTS_LEFT, 6, 122);
	}
	else {
		RenderLightPanel(surf, lightsurf, RightLights, TestState == CWS_TEST_LIGHTS_RIGHT, 261, 122);
	}
}

void CSMCautionWarningSystem::RenderLightPanel(SURFHANDLE surf, SURFHANDLE lightsurf, bool *LightState, bool LightTest, int sdx, int sdy)

{
	int i = 0;
	int row, column;

	if (!IsPowered())
		return;

	for (row = 0; row < 6; row++) {
		for (column = 0; column < 4; column++) {
			if (LightTest || LightState[i]) {
				oapiBlt(surf, lightsurf, column * 53, row * 18, column * 53 + sdx, row * 18 + sdy, 50, 16);
			}
			i++;
		}
	}
}
