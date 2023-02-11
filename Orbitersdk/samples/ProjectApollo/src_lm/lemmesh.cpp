/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Jean-Luc Rocca-Serra, Mark Grant

  ORBITER vessel module: LEM mesh code

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

// To force Orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"
#include "stdio.h"
#include "math.h"
#include "lmresource.h"

#include "nasspdefs.h"
#include "nasspsound.h"

#include "soundlib.h"
#include "toggleswitch.h"
#include "apolloguidance.h"
#include "LEMcomputer.h"
#include "lm_channels.h"

#include "LEM.h"
#include "leva.h"
#include "Sat5LMDSC.h"
#include "LM_AscentStageResource.h"
#include "LM_DescentStageResource.h"
#include "LM_VC_Resource.h"
#include "Mission.h"

MESHHANDLE hLMDescent;
MESHHANDLE hLMDescentNoLeg;
MESHHANDLE hLMAscent;
MESHHANDLE hAstro1;
MESHHANDLE hLMVC;

static PARTICLESTREAMSPEC lunar_dust = {
	0,		// flag
	1,	    // size
	5,      // rate
	20,	    // velocity
	1,      // velocity distribution
	2.0,    // lifetime
	10,   	// growthrate
	2.0,    // atmslowdown 
	PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_LIN, 0, 1,
	PARTICLESTREAMSPEC::ATM_PLOG, -0.1, 0.1
};

VECTOR3 mesh_asc = _V(0.00, 0.99, 0.00);
VECTOR3 mesh_dsc = _V(0.00, -1.25, 0.00);

void LEM::ToggleEVA(bool isCDR)
{
	ToggleEva = false;

	int i = isCDR ? 0 : 1;
	
	if (EVA_IP[i]) {
		// Nothing for now, the EVA is ended, when the LEVA vessel calls StopEVA
	}
	else {
		VESSELSTATUS vs1;
		GetStatus(vs1);

		// The LM must be in landed state
		if (vs1.status != 1) return;

		EVA_IP[i] = true;

		OBJHANDLE hbody = GetGravityRef();
		double radius = oapiGetSize(hbody);
		vs1.vdata[0].x += 4.5 * sin(vs1.vdata[0].z) / radius;
		vs1.vdata[0].y += 4.5 * cos(vs1.vdata[0].z) / radius;

		char VName[256]="";
		strcpy (VName, GetName());
		if (isCDR)
		{
			strcat(VName, "-LEVA-CDR");
			SwitchFocusToLeva = 10;
		}
		else
		{
			strcat(VName, "-LEVA-LMP");
			SwitchFocusToLeva = -10;
		}
		hLEVA[i] = oapiCreateVessel(VName,"ProjectApollo/LEVA",vs1);

		LEVA *leva = (LEVA *) oapiGetVesselInterface(hLEVA[i]);
		if (leva) {
			LEVASettings evas;

			evas.MissionNo = ApolloNo;
			evas.isCDR = isCDR;
			leva->SetEVAStats(evas);
		}
	}
}

void LEM::StopEVA(bool isCDR)

{
	// Called by LEVA vessel during destruction
	if (isCDR)
	{
		EVA_IP[0] = false;
	}
	else
	{
		EVA_IP[1] = false;
	}
}

void LEM::SetLmVesselDockStage()

{
	ClearThrusterDefinitions();
	SetEmptyMass(AscentFuelMassKg + AscentEmptyMassKg + DescentEmptyMassKg);
	SetSize (6);
	SetVisibilityLimit(1e-3, 4.6401e-4);
	SetPMI(_V(2.5428, 2.2871, 2.7566));
	SetCrossSections (_V(24.53,21.92,24.40));
	CreateAirfoils();
	SetRotDrag (_V(0.7,0.7,0.7));
	SetPitchMomentScale (0);
	SetYawMomentScale (0);
	SetLiftCoeffFunc (0); 
	ClearBeacons();
	ClearExhaustRefs();
	ClearAttExhaustRefs();

	DefineTouchdownPoints(0);

	// Configure meshes if needed
	if (!pMission->LMHasLegs()) InsertMesh(hLMDescentNoLeg, dscidx, &mesh_dsc);
	SetLMMeshVis();

	if (!ph_Dsc)
	{
		ph_Dsc = CreatePropellantResource(DescentFuelMassKg); //2nd stage Propellant
	}

	SetDefaultPropellantResource(ph_Dsc); // display 2nd stage propellant level in generic HUD

	// 133.084001 kg is 293.4 pounds, which is the fuel + oxidizer capacity of one RCS tank.
	if (!ph_RCSA) {
		ph_RCSA = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}
	if (!ph_RCSB) {
		ph_RCSB = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}

	// orbiter main thrusters
	//Ascent stage mesh has RCS plane as reference, but it's shifted by 0.99 m up for center of full LM mesh
	//RCS plane is at 254 inches in LM coordinates. DPS gimbal plane is at 154 inches in LM coordinates
	//Therefore: 3.9116 m - (6.4516 m - 0.99 m) = -1.55 m for the DPS reference position
	th_hover[0] = CreateThruster(_V(0.0, -1.55, 0.0), _V(0, 1, 0), 46706.3, ph_Dsc, 3107);
	thg_hover = CreateThrusterGroup(th_hover, 1, THGROUP_USER);
	
	EXHAUSTSPEC es_hover[1] = {
		{ th_hover[0], NULL, NULL, NULL, 10.0, 1.5, 1.16, 0.1, exhaustTex, EXHAUST_CONSTANTPOS }
	};

	AddExhaust(es_hover);

	AddDust();

	status = 0;
	stage = 0;
	SetView();
	AddRCS_LMH(-5.4616); //254 inches minus the 0.99m offset from mesh_asc = 5.4616 m

	InitNavRadios (4);

	// Exterior lights
	SetTrackLight();
	SetDockingLights();
}

void LEM::SetLmVesselHoverStage()
{
	ClearThrusterDefinitions();

	SetEmptyMass(AscentFuelMassKg + AscentEmptyMassKg + DescentEmptyMassKg);

	SetSize (7);
	SetVisibilityLimit(1e-3, 5.4135e-4);
	SetPMI(_V(2.5428, 2.2871, 2.7566));
	SetCrossSections (_V(24.53,21.92,24.40));
	CreateAirfoils();
	SetRotDrag (_V(0.7,0.7,0.7));
	SetPitchMomentScale (0);
	SetYawMomentScale (0);
	SetLiftCoeffFunc (0);
	ClearBeacons();
	ClearExhaustRefs();
	ClearAttExhaustRefs();

	DefineTouchdownPoints(1);

	if (!ph_Dsc){  
		ph_Dsc  = CreatePropellantResource(DescentFuelMassKg); //2nd stage Propellant
	}
	else
	{
		SetPropellantMaxMass(ph_Dsc, DescentFuelMassKg);
	}

	SetDefaultPropellantResource (ph_Dsc); // display 2nd stage propellant level in generic HUD

	if (!ph_RCSA){
		ph_RCSA = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}
	if (!ph_RCSB){
		ph_RCSB = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}
	
	// orbiter main thrusters
	//Ascent stage mesh has RCS plane as reference, but it's shifted by 0.99 m up for center of full LM mesh
	//RCS plane is at 254 inches in LM coordinates. DPS gimbal plane is at 154 inches in LM coordinates
	//Therefore: 3.9116 m - (6.4516 m - 0.99 m) = -1.55 m for the DPS reference position
	th_hover[0] = CreateThruster(_V(0.0, -1.55, 0.0), _V(0, 1, 0), 46706.3, ph_Dsc, 3107);
	thg_hover = CreateThrusterGroup(th_hover, 1, THGROUP_USER);

	EXHAUSTSPEC es_hover[1] = {
		{ th_hover[0], NULL, NULL, NULL, 10.0, 1.5, 1.16, 0.1, exhaustTex, EXHAUST_CONSTANTPOS }
	};

	AddExhaust(es_hover);

	AddDust();

	status = 1;
	stage = 1;
	SetView();
	AddRCS_LMH(-5.4616); //254 inches minus the 0.99m offset from mesh_asc = 5.4616 m

	InitNavRadios (4);

	// Exterior lights
	SetTrackLight();
	SetDockingLights();
}

void LEM::SetLmAscentHoverStage()

{
	ClearThrusterDefinitions();
	ShiftCG(_V(0.0,1.75,0.0) - currentCoG);
	//We have shifted everything to the center of the mesh. If currentCoG gets used by the ascent stage it will be updated on the next timestep
	currentCoG = _V(0, 0, 0);
	LastFuelWeight = numeric_limits<double>::infinity(); // Ensure update at first opportunity
	SetSize (5);
	SetVisibilityLimit(1e-3, 3.8668e-4);
	SetEmptyMass (AscentEmptyMassKg);
	SetPMI(_V(2.8, 2.29, 2.37));
	SetCrossSections (_V(21,23,17));
	CreateAirfoils();
	SetRotDrag (_V(0.7,0.7,0.7));
	SetPitchMomentScale (0);
	SetYawMomentScale (0);
	SetLiftCoeffFunc (0); 
	ClearBeacons();
	ClearExhaustRefs();
	ClearAttExhaustRefs();
	eds.DeleteAnimations();
	DPS.DeleteAnimations();

	DefineTouchdownPoints(2);

	// Configure meshes if needed
	DelMesh(dscidx);
	dscidx = -1;

	if (!ph_Asc)
	{
		ph_Asc = CreatePropellantResource(AscentFuelMassKg);	// 2nd stage Propellant
	}
	else
	{
		SetPropellantMaxMass(ph_Asc, AscentFuelMassKg);
	}
	SetDefaultPropellantResource (ph_Asc);			// Display 2nd stage propellant level in generic HUD

													// 133.084001 kg is 293.4 pounds, which is the fuel + oxidizer capacity of one RCS tank.
	if (!ph_RCSA) {
		ph_RCSA = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}
	if (!ph_RCSB) {
		ph_RCSB = CreatePropellantResource(LM_RCS_FUEL_PER_TANK);
	}

	// orbiter main thrusters
	//Point of thrust application of the APS is at 232.96 inches in LM coordinates
	//Center of mesh is the RCS reference plane at 254 inches in LM coordinates
	//Mesh is shifted up by 0.99 meters when it is loaded and then 1.75 meters down with ShiftCG in this function
	//Therefore we need to calculate: 232.96 in - (254 in - 0.99 m + 1.75 m) = -1.294416 m
	//Point of thrust application is also shifted 3.75 in (0.09525 m) in the z-axis.
	//And it's canted at 1.5� to point through the CG which is located forward of the centerline
    th_hover[0] = CreateThruster (_V( 0.0,  -1.294416, 0.09525), _V( 0,cos(1.5*RAD),sin(1.5*RAD)), APS_THRUST, ph_Asc, APS_ISP);
	thg_hover = CreateThrusterGroup (th_hover, 1, THGROUP_USER);
	
	EXHAUSTSPEC es_hover[1] = {
		{ th_hover[0], NULL, NULL, NULL, 6.0, 0.8, 0.5, 0.1, exhaustTex, EXHAUST_CONSTANTPOS }
	};

	AddExhaust(es_hover);
	
	status = 2;
	stage = 2;
	SetView();
	AddRCS_LMH(-7.2116);  //254 inches minus the 0.99m offset from mesh_asc and plus 1.75 m from the ShiftCG = 7.2116 m

	if(ph_Dsc){
		DelPropellantResource(ph_Dsc);
		ph_Dsc = 0;
	}
	
	// CSM/LM interface is located at 312.5 inches in LM coordinates
	// Applying the same shift as for APS and RCS this gives: 312.5 in - (254 in - 0.99 m + 1.75 m) = 0.7259 m
	// TBD: Implement that
	SetLmDockingPort(0.85);
	InitNavRadios (4);

	// Exterior lights
	SetTrackLight();
	SetDockingLights();
}

void LEM::SeparateStage (UINT stage)

{
	ResetThrusters();

	if (docksla) {
		DelDock(docksla);
		docksla = NULL;
	}

	VESSELSTATUS2 vs2;
	memset(&vs2, 0, sizeof(vs2));
	vs2.version = 2;

	if (stage == 0) {
		GetStatusEx(&vs2);
		Local2Rel(OFS_LM_DSC - currentCoG, vs2.rpos);

		char VName[256];
		strcpy(VName, GetName()); strcat(VName, "-DESCENTSTG");
		hdsc = oapiCreateVesselEx(VName, "ProjectApollo/Sat5LMDSC", &vs2);

		Sat5LMDSC *dscstage = static_cast<Sat5LMDSC *> (oapiGetVesselInterface(hdsc));
		if (!pMission->LMHasLegs())
		{
			dscstage->SetState(0);
		}
		else
		{
			dscstage->SetState(1);
		}

		SetLmAscentHoverStage();
	}
	
	if (stage == 1)	{
		GetStatusEx(&vs2);
		
		if (vs2.status == 1) {
			vs2.vrot.x = 2.32;
			char VName[256];
			strcpy(VName, GetName()); strcat(VName, "-DESCENTSTG");
			hdsc = oapiCreateVesselEx(VName, "ProjectApollo/Sat5LMDSC", &vs2);
			
			Sat5LMDSC *dscstage = static_cast<Sat5LMDSC *> (oapiGetVesselInterface(hdsc));
			if (!pMission->LMHasLegs())
			{
				dscstage->SetState(0);
			}
			else if (Landed)
			{
				dscstage->SetState(3);
			}
			else
			{
				dscstage->SetState(2);
			}
			
			vs2.vrot.x = 5.32;
			DefSetStateEx(&vs2);
			SetLmAscentHoverStage();
		}
		else
		{
			Local2Rel(OFS_LM_DSC - currentCoG, vs2.rpos);

			char VName[256];
			strcpy(VName, GetName()); strcat(VName, "-DESCENTSTG");
			hdsc = oapiCreateVesselEx(VName, "ProjectApollo/Sat5LMDSC", &vs2);
			
			Sat5LMDSC *dscstage = static_cast<Sat5LMDSC *> (oapiGetVesselInterface(hdsc));
			if (!pMission->LMHasLegs())
			{
				dscstage->SetState(0);
			}
			else if (Landed)
			{
				dscstage->SetState(3);
			}
			else
			{
				dscstage->SetState(2);
			}

			SetLmAscentHoverStage();
		}
	}

	CheckDescentStageSystems();
}

void LEM::SetLmLandedMesh() {

	Landed = true;

	HideProbes();
}

void LEM::SetLMMeshVis() {

	SetLMMeshVisAsc();
	SetLMMeshVisVC();
	SetLMMeshVisDsc();
}

void LEM::SetLMMeshVisAsc() {

	if (ascidx == -1)
		return;

	if (!ExtView && InPanel && (PanelId == LMPANEL_AOTZOOM || PanelId == LMPANEL_UPPERHATCH)) {
		SetMeshVisibilityMode(ascidx, MESHVIS_ALWAYS);
	}
	else
	{
		SetMeshVisibilityMode(ascidx, MESHVIS_EXTERNAL);
	}
}

void LEM::SetLMMeshVisVC() {

	if (vcidx == -1)
		return;

	if (!ExtView && InPanel && PanelId == LMPANEL_LPDWINDOW) {
		SetMeshVisibilityMode(vcidx, MESHVIS_ALWAYS);
	}
	else
	{
		SetMeshVisibilityMode(vcidx, MESHVIS_VC);
	}
}

void LEM::SetLMMeshVisDsc() {
	
	if (dscidx == -1)
		return;

	if (!ExtView && InPanel && PanelId == LMPANEL_LPDWINDOW) {
		SetMeshVisibilityMode(dscidx, MESHVIS_ALWAYS);
	}
	else
	{
		SetMeshVisibilityMode(dscidx, MESHVIS_VCEXTERNAL);
	}
}

void LEM::SetCrewMesh() {

	static UINT meshgroup_LMP[11] = { AS_GRP_LMP1, AS_GRP_LMP2, AS_GRP_LMP3, AS_GRP_LMP4, AS_GRP_LMP5, AS_GRP_LMP6, AS_GRP_LMP7, AS_GRP_LMP8, AS_GRP_LMP9, AS_GRP_LMP91, AS_GRP_zLMP11 };
	static UINT meshgroup_CDR[11] = { AS_GRP_CDR1, AS_GRP_CDR2, AS_GRP_CDR3, AS_GRP_CDR4, AS_GRP_CDR5, AS_GRP_CDR6, AS_GRP_CDR7, AS_GRP_CDR8, AS_GRP_CDR9, AS_GRP_CDR91, AS_GRP_zCDR11 };
	GROUPEDITSPEC ges;

	if (cdrmesh) {
		if (Crewed && CDRSuited->number == 1) {
			ges.flags = (GRPEDIT_SETUSERFLAG);
			ges.UsrFlag = 0;
			for (int i = 0; i < 11; i++) oapiEditMeshGroup(cdrmesh, meshgroup_CDR[i], &ges);
		} else {
			ges.flags = (GRPEDIT_ADDUSERFLAG);
			ges.UsrFlag = 3;
			for (int i = 0; i < 11; i++) oapiEditMeshGroup(cdrmesh, meshgroup_CDR[i], &ges);
		}
	}
	if (lmpmesh) {
		if (Crewed && LMPSuited->number == 1) {
			ges.flags = (GRPEDIT_SETUSERFLAG);
			ges.UsrFlag = 0;
			for (int i = 0; i < 11; i++) oapiEditMeshGroup(cdrmesh, meshgroup_LMP[i], &ges);
		} else {
			ges.flags = (GRPEDIT_ADDUSERFLAG);
			ges.UsrFlag = 3;
			for (int i = 0; i < 11; i++) oapiEditMeshGroup(cdrmesh, meshgroup_LMP[i], &ges);
		}
	}
}

void LEM::DrogueVis() {

	if (!drogue)
		return;

	static UINT meshgroup_Drogue = AS_GRP_Drogue;
	GROUPEDITSPEC ges;

	if (OverheadHatch.IsOpen()) {
		ges.flags = (GRPEDIT_ADDUSERFLAG);
		ges.UsrFlag = 3;
		oapiEditMeshGroup(drogue, meshgroup_Drogue, &ges);
	}
	else
	{
		ges.flags = (GRPEDIT_SETUSERFLAG);
		ges.UsrFlag = 0;
		oapiEditMeshGroup(drogue, meshgroup_Drogue, &ges);
	}
}

void LEM::HideProbes() {

	if (!probes)
		return;

	if (Landed && pMission->LMHasLegs()) {
		static UINT meshgroup_Probes1[3] = { DS_GRP_Probes1Aft, DS_GRP_Probes1Left, DS_GRP_Probes1Right };
		static UINT meshgroup_Probes2[3] = { DS_GRP_Probes2Aft, DS_GRP_Probes2Left, DS_GRP_Probes2Right };
		GROUPEDITSPEC ges;

		for (int i = 0; i < 3; i++) {
			ges.flags = (GRPEDIT_ADDUSERFLAG);
			ges.UsrFlag = 3;
			oapiEditMeshGroup(probes, meshgroup_Probes1[i], &ges);
			oapiEditMeshGroup(probes, meshgroup_Probes2[i], &ges);
		}
	}
}

void LEM::SetTrackLight() {
	
	static VECTOR3 beaconCol = _V(1, 1, 1);
	trackLight.shape = BEACONSHAPE_STAR;

	if (stage == 2)
	{
		trackLightPos = _V(0.00, -0.38, 2.28);
	}
	else
	{
		trackLightPos = _V(0.00, 1.38, 2.28);
	}

	trackLight.pos = &trackLightPos;
	trackLight.col = &beaconCol;
	trackLight.size = 0.5;
	trackLight.falloff = 0.5;
	trackLight.period = 1.0;
	trackLight.duration = 0.1;
	trackLight.tofs = 0;
	trackLight.active = false;
	AddBeacon(&trackLight);
}

void LEM::SetDockingLights() {

	int i;
	double yoffset = -1.76;

	if (stage == 2)
	{
		dockingLightsPos[0] = { 0.28, 1.47 + yoffset, 2.27 };
		dockingLightsPos[1] = { 0.00, 1.85 + yoffset, -1.81 };
		dockingLightsPos[2] = { -0.28, 1.47 + yoffset, 2.27 };
		dockingLightsPos[3] = { -2.52, 0.59 + yoffset, 0.20 };
		dockingLightsPos[4] = { 1.91, 0.29 + yoffset, 0.22 };
	}
	else
	{
		dockingLightsPos[0] = { 0.28, 1.47, 2.27 };
		dockingLightsPos[1] = { 0.00, 1.85, -1.81 };
		dockingLightsPos[2] = { -0.28, 1.47, 2.27 };
		dockingLightsPos[3] = { -2.52, 0.59, 0.20 };
		dockingLightsPos[4] = { 1.91, 0.29, 0.22 };
	}

	static VECTOR3 beaconCol[4] = { { 1.0, 1.0, 1.0 },{ 1.0, 1.0, 0.5 },{ 1.0, 0.5, 0.5 },{ 0.5, 1.0, 0.5 } };
	for (i = 0; i < 5; i++) {
		dockingLights[i].shape = BEACONSHAPE_DIFFUSE;
		dockingLights[i].pos = &dockingLightsPos[i];
		dockingLights[i].col = (i < 2 ? beaconCol : i < 3 ? beaconCol+1 : i < 4 ? beaconCol+2 : beaconCol+3);
		dockingLights[i].size = 0.12;
		dockingLights[i].falloff = 0.8;
		dockingLights[i].period = 0.0;
		dockingLights[i].duration = 1.0;
		dockingLights[i].tofs = 0;
		dockingLights[i].active = false;
		AddBeacon(dockingLights+i);
	}
}

void LEM::SetCOAS() {

	if (!vcmesh)
		return;

	static UINT meshgroup_COASfwd[3] = { VC_GRP_COAS_1, VC_GRP_COAS_2, VC_GRP_zzzCOAS_Glass };
	static UINT meshgroup_COASovhd[3] = { VC_GRP_COAS_1ovhd, VC_GRP_COAS_2ovhd, VC_GRP_zzzCOAS_Glassovhd };
	static UINT meshgroup_ReticleFwd = VC_GRP_COAS_Reticle;
	static UINT meshgroup_ReticleOvhd = VC_GRP_COAS_Reticleovhd;

	GROUPEDITSPEC ges_on;
	ges_on.flags = (GRPEDIT_SETUSERFLAG);
	ges_on.UsrFlag = 0;
	GROUPEDITSPEC ges_off;
	ges_off.flags = (GRPEDIT_ADDUSERFLAG);
	ges_off.UsrFlag = 3;

	// FWD COAS
	if (LEMCoas2Enabled) {
		for (int i = 0; i < 3; i++) {
			oapiEditMeshGroup(vcmesh, meshgroup_COASfwd[i], &ges_on);
		}
		if (InVC && oapiCameraInternal() && viewpos == LMVIEW_CDR && COASreticlevisible == 1) {
			oapiEditMeshGroup(vcmesh, meshgroup_ReticleFwd, &ges_on);
		} else {
			oapiEditMeshGroup(vcmesh, meshgroup_ReticleFwd, &ges_off);
		}
	} else {
		for (int i = 0; i < 3; i++) {
			oapiEditMeshGroup(vcmesh, meshgroup_COASfwd[i], &ges_off);
		}
		oapiEditMeshGroup(vcmesh, meshgroup_ReticleFwd, &ges_off);
	}

	// OVHD COAS
	if (LEMCoas1Enabled) {
		for (int i = 0; i < 3; i++) {
			oapiEditMeshGroup(vcmesh, meshgroup_COASovhd[i], &ges_on);
		}
		if (InVC && oapiCameraInternal() && viewpos == LMVIEW_RDVZWIN && COASreticlevisible == 2) {
			oapiEditMeshGroup(vcmesh, meshgroup_ReticleOvhd, &ges_on);
		}
		else {
			oapiEditMeshGroup(vcmesh, meshgroup_ReticleOvhd, &ges_off);
		}
	}
	else {
		for (int i = 0; i < 3; i++) {
			oapiEditMeshGroup(vcmesh, meshgroup_COASovhd[i], &ges_off);
		}
		oapiEditMeshGroup(vcmesh, meshgroup_ReticleOvhd, &ges_off);
	}
}

void LEM::DefineTouchdownPoints(int s)
{
	//Touchdown Points
	double xtarget = -1.0;
	if (s == 0)
	{
		ConfigTouchdownPoints(7137.75, 3.5, 0.5, -3.7, 0, 3.86, xtarget); // landing gear retracted
	}
	else if (s == 1)
	{
		if (pMission->LMHasLegs())
		{
			ConfigTouchdownPoints(7137.75, 3.5, 4.25, -3.7, -5.41, 3.86, xtarget); // landing gear extended
		}
		else
		{
			ConfigTouchdownPoints(7137.75, 3.5, 0.5, -3.7, 0, 3.86, xtarget); // landing gear retracted
		}
	}
	else
	{
		ConfigTouchdownPoints(4495.0, 3, 3, -5.42, 0, 2.8, xtarget);
	}
}

void LEM::ConfigTouchdownPoints(double mass, double ro1, double ro2, double tdph, double probeh, double height, double x_target) {

	static DWORD ntdvtx = 12;
	static TOUCHDOWNVTX tdv[12];

	double stiffness = (-1)*(mass*9.80655) / (3 * x_target);
	double damping = 0.9*(2 * sqrt(mass*stiffness));
	for (int i = 0; i < 12; i++) {
		if (i < 9) {
			tdv[i].damping = damping;
			tdv[i].stiffness = stiffness;
		}
		else {
			tdv[i].damping = damping / 200;
			tdv[i].stiffness = stiffness / 200;
		}
		tdv[i].mu = 10;
		tdv[i].mu_lng = 10;
	}

	tdv[0].pos.x = 0;
	tdv[0].pos.y = tdph;
	tdv[0].pos.z = ro2;
	tdv[1].pos.x = -ro2;
	tdv[1].pos.y = tdph;
	tdv[1].pos.z = 0;
	tdv[2].pos.x = 0;
	tdv[2].pos.y = tdph;
	tdv[2].pos.z = -ro2;
	tdv[3].pos.x = ro2;
	tdv[3].pos.y = tdph;
	tdv[3].pos.z = 0;
	tdv[4].pos.x = 0;
	tdv[4].pos.y = 0;
	tdv[4].pos.z = ro1;
	tdv[5].pos.x = -ro1;
	tdv[5].pos.y = 0;
	tdv[5].pos.z = 0;
	tdv[6].pos.x = 0;
	tdv[6].pos.y = 0;
	tdv[6].pos.z = -ro1;
	tdv[7].pos.x = ro1;
	tdv[7].pos.y = 0;
	tdv[7].pos.z = 0;
	tdv[8].pos.x = 0;
	tdv[8].pos.y = height;
	tdv[8].pos.z = 0;
	tdv[9].pos.x = -ro2;
	tdv[9].pos.y = probeh;
	tdv[9].pos.z = 0;
	tdv[10].pos.x = 0;
	tdv[10].pos.y = probeh;
	tdv[10].pos.z = -ro2;
	tdv[11].pos.x = ro2;
	tdv[11].pos.y = probeh;
	tdv[11].pos.z = 0;

	for (int i = 0;i < 12;i++)
	{
		tdv[i].pos -= currentCoG;
	}

	SetTouchdownPoints(tdv, ntdvtx);
}

void LEM::AddDust() {

	// Simulate the dust kicked up near
	// the lunar surface
	int i;

	VECTOR3	s_exhaust_pos1 = _V(0, -15, 0);
	VECTOR3 s_exhaust_pos2 = _V(0, -15, 0);
	VECTOR3	s_exhaust_pos3 = _V(0, -15, 0);
	VECTOR3 s_exhaust_pos4 = _V(0, -15, 0);

	th_dust[0] = CreateThruster(s_exhaust_pos1, _V(-1, 0, 1), 0, ph_Dsc);
	th_dust[1] = CreateThruster(s_exhaust_pos2, _V(1, 0, 1), 0, ph_Dsc);
	th_dust[2] = CreateThruster(s_exhaust_pos3, _V(1, 0, -1), 0, ph_Dsc);
	th_dust[3] = CreateThruster(s_exhaust_pos4, _V(-1, 0, -1), 0, ph_Dsc);

	for (i = 0; i < 4; i++) {
		AddExhaustStream(th_dust[i], &lunar_dust);
	}
	thg_dust = CreateThrusterGroup(th_dust, 4, THGROUP_USER);
}

void LEM::SetMeshes() {

	// Ascent Stage Mesh
	ascidx = AddMesh(hLMAscent, &mesh_asc);

	// VC Mesh
	vcidx = AddMesh(hLMVC, &mesh_asc);

	// Descent Stage Mesh
	dscidx = AddMesh(hLMDescent, &mesh_dsc);
}

void LEM::CreateAirfoils()
{
	if (docksla && GetDockStatus(docksla))
	{
		SetCW(0, 0, 0, 0);
	}
	else
	{
		SetCW(0.2, 0.2, 0.2, 0.2); //TBD: Set to 4.0 when RTCC can take drag into account properly
	}
}

void LEMLoadMeshes()

{
	hLMDescent = oapiLoadMeshGlobal ("ProjectApollo/LM_DescentStage");
	hLMDescentNoLeg = oapiLoadMeshGlobal("ProjectApollo/LM_DescentStageNoLeg");
	hLMAscent = oapiLoadMeshGlobal ("ProjectApollo/LM_AscentStage");
	hAstro1= oapiLoadMeshGlobal ("ProjectApollo/Sat5AstroS");
	hLMVC = oapiLoadMeshGlobal("ProjectApollo/LM_VC");
	lunar_dust.tex = oapiRegisterParticleTexture("ProjectApollo/dust");
}

//
// Debug routine.
//

/*
void LEM::GetDockStatus()

{
	VESSELSTATUS2 vslm;
	VESSELSTATUS2::DOCKINFOSPEC dckinfo;

	memset(&vslm, 0, sizeof(vslm));
	memset(&dckinfo, 0, sizeof(dckinfo));

	vslm.flag = 0; // VS_DOCKINFOLIST;
	vslm.fuel = 0;
	vslm.thruster = 0;
	vslm.ndockinfo = 1;
	vslm.dockinfo = &dckinfo;
	vslm.version = 2;

//	GetStatusEx(&vslm);
}
*/
