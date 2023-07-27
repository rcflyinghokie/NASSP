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

#if !defined(_PA_CSMCOMPUTER_H)
#define _PA_CSMCOMPUTER_H

#include "thread.h"

class PanelSwitchItem;

// OPTICS CONFIGURATION DEFINES
// Step values in radians.
#define OCDU_SHAFT_STEP 0.000191747598876953125 
#define OCDU_TRUNNION_STEP 0.00004793689959716796875


///
/// AGC output channel five, used to control CM and SM RCS thrusters.
///
/// \ingroup AGCIO
/// \brief CSM AGC output channel 5.
///
typedef union {
	struct {
		unsigned SMC3:1;			///< Fire SM RCS C-3
		unsigned SMC4:1;			///< Fire SM RCS C-4
		unsigned SMA3:1;			///< Fire SM RCS A-3
		unsigned SMA4:1;			///< Fire SM RCS A-4
		unsigned SMD3:1;			///< Fire SM RCS D-3
		unsigned SMD4:1;			///< Fire SM RCS D-4
		unsigned SMB3:1;			///< Fire SM RCS B-3
		unsigned SMB4:1;			///< Fire SM RCS B-4
	} u;
	unsigned int word;				///< Word holds the flags from the bitfield in one 32-bit value.
} CSMOut5;

///
///
/// AGC output channel five, used to control CM and SM RCS thrusters.
///
/// \ingroup AGCIO
/// \brief CSM AGC output channel 6.
///
typedef union {
	struct {
		unsigned SMB1:1;			///< Fire SM RCS B-1
		unsigned SMB2:1;			///< Fire SM RCS B-2
		unsigned SMD1:1;			///< Fire SM RCS D-1
		unsigned SMD2:1;			///< Fire SM RCS D-2
		unsigned SMA1:1;			///< Fire SM RCS A-1
		unsigned SMA2:1;			///< Fire SM RCS A-2
		unsigned SMC1:1;			///< Fire SM RCS C-1
		unsigned SMC2:1;			///< Fire SM RCS C-2
	} u;
	unsigned int word;				///< Word holds the flags from the bitfield in one 32-bit value.
} CSMOut6;

class Saturn;
class IU;
class CSMToIUConnector;
class CSMToSIVBControlConnector;
class CDU;

//
// Class definition.
//

///
/// CSM guidance computer.
///
/// \ingroup AGC
/// \brief Derived class for the CSM AGC with extra CSM-specific functionality.
///
class CSMcomputer: public ApolloGuidance

{
public:

	///
	/// The AGC needs to talk to various other objects in the CSM. These are passed to the
	/// constructor so we can set references to them.
	///
	/// \brief CSM AGC constructor.
	/// \param s Sound library to use for playing sound.
	/// \param display Main control panel DSKY interface.
	/// \param display2 Lower equipment bay DSKY interface.
	/// \param im The CSM Inertial Measurement Unit.
	/// \param p The Panel SDK library.
	/// \param i The launch vehicle Instrument Unit connector for the launch vehicle autopilot.
	/// \param sivb The CSM to SIVb command connector (e.g. for fuel venting).
	///
	CSMcomputer(SoundLib &s, DSKY &display, DSKY &display2, IMU &im, CDU &sc, CDU &tc, PanelSDK &p);
	virtual ~CSMcomputer();

	bool ReadMemory(unsigned int loc, int &val);
	void WriteMemory(unsigned int loc, int val);

	void Timestep(double simt, double simdt);
	void Run() ;
	void agcTimestep(double simt, double simdt);

	//
	// Data access.
	//

	void SetInputChannelBit(int channel, int bit, bool val);
	void SetOutputChannel(int channel, ChannelValue val);

	void SetMissionInfo(std::string ProgramName, char *OtherVessel = 0);

	VESSEL *GetLM();
protected:

	void ProcessChannel5(ChannelValue val);
	void ProcessChannel6(ChannelValue val);
	void ProcessChannel10(ChannelValue val);
	void ProcessChannel11Bit(int bit, bool val);
	void ProcessChannel11(ChannelValue val);
	// DS20060226 TVC / Optics
	void ProcessChannel14(ChannelValue val);
	// DS20060308 FDAI NEEDLES
	void ProcessIMUCDUErrorCount(int channel, ChannelValue val);
	void ProcessIMUCDUReadCount(int channel, int val);

	FILE *Dfile;
	int count;

	unsigned int LastOut5;
	unsigned int LastOut6;
	unsigned int LastOut11;

	///
	/// \brief Second DSKY in the lower equipment bay.
	///
	DSKY &dsky2;

	Saturn *sat;
};

class Saturn;

// *** CM OPTICS ***
// I guess this can go here; it doesn't really warrant its own file, and it's part of GNC, so...

///
/// \ingroup AGC
/// \brief CM Optics.
///
class CMOptics {	
public: 
	CMOptics();														// Cons
	void Init(Saturn *vessel);										// Initialization
	void TimeStep(double simdt);                                    // Timestep
	void SystemTimestep(double simdt);
	bool PaintShaftDisplay(SURFHANDLE surf, SURFHANDLE digits, int xTexMul = 1);		// Update panel image
	bool PaintTrunnionDisplay(SURFHANDLE surf, SURFHANDLE digits, int xTexMul = 1);	// Update panel image
	void OpticsSwitchToggled();

	void SaveState(FILEHANDLE scn);
	void LoadState(FILEHANDLE scn);

	//
	// These should really be protected variables.
	//

	Saturn *sat;													// Our Ship

	int Powered;                                                    // 0 = NO, 1 = MNA, 2 = MNB, 3 = Both
	int OpticsManualMovement;										// Manual Movement Demand Flags
	double SextShaft;												// SXT Shaft
	double TeleShaft;												// SCT Shaft
	double SextTrunion;												// SXT Trunion
	double TeleTrunion;												// SCT Trunion
	double TeleShaftRate;
	double TeleTrunionRate;
	double dShaft;
	double dTrunion;
	bool SextDualView;												// Toggle logical for sextant dual-view
	bool SextDVLOSTog;												// Alternating flag that controls LineOfSight cycling in Dual-View mode
	double SextDVTimer;												// Governing timer to prevent view switching at greater than 15 frames per sim second
	bool OpticsCovered;												// Are optics covers in place?
protected:
	bool PaintDisplay(SURFHANDLE surf, SURFHANDLE digits, int value, int xTexMul = 1);
	void TelescopeServoDrive(double dt, double sxt_angle, double &sct_angle, double &sct_rate);
};


//
// Strings for state saving.
//

#define CMOPTICS_START_STRING "CMOPTICS_BEGIN"
#define CMOPTICS_END_STRING   "CMOPTICS_END"


#endif // _PA_CSMCOMPUTER_H
