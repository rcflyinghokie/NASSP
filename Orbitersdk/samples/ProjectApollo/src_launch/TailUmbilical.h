/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2019

Base class for S-IB Short Cable and S-IC Tail Service Mast Umbilical (Header)

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

class TailUmbilicalInterface;

class TailUmbilical
{
public:
	TailUmbilical(TailUmbilicalInterface *ml);
	virtual ~TailUmbilical();

	virtual void Disconnect() = 0;

	//Called by IU during a pad abort. Technically doesn't disconnect IU umbilical
	virtual void AbortDisconnect();

	//From SLV to ML
	virtual bool ESEGetSIThrustOKSimulate(int eng, int n);

	//From ML to SLV
	virtual bool SIStageLogicCutoff() = 0;
	virtual void SetEngineStart(int eng) = 0;
	virtual void SIGSECutoff(bool cut) = 0;
protected:
	bool UmbilicalConnected;
	TailUmbilicalInterface* TailUmb;
};