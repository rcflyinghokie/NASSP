/****************************************************************************
This file is part of Project Apollo - NASSP

RTCC Utilities (Header)

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

#include <vector>

namespace rtcc
{
	struct MEDProcessingDoubleOptions
	{
		int missing = 0;		// 0 = ignore, 1 = error
		double scale = 1.0;		// Used as offset with time option
		bool mincheck = false;
		double minval = 0.0;	// Check on input value
		bool maxcheck = false;
		double maxval = 0.0;	// Check on input value
		double defaultval = 0.0;
	};

	int MEDProcessingDouble(const std::vector<std::string> &data, unsigned i, MEDProcessingDoubleOptions opt, double &val)
	{
		//missing: 0 = ignore, 1 = error
		std::string item;
		bool isMissing;

		item = data[i];
		isMissing = (item == "");

		if (isMissing)
		{
			//Error
			if (opt.missing == 1)
			{
				return 1;
			}
			//Ignore
			else
			{
				return 0;
			}
		}

		double valtemp;

		if (sscanf(item.c_str(), "%lf", &valtemp) != 1)
		{
			return 2;
		}

		//Limit checking
		if (opt.mincheck)
		{
			if (valtemp < opt.minval)
			{
				return 2;
			}
		}
		if (opt.maxcheck)
		{
			if (valtemp > opt.maxval)
			{
				return 2;
			}
		}

		//Scale
		valtemp *= opt.scale;

		val = valtemp;
		return 0;
	}

	struct MEDProcessingIntegerOptions
	{
		int missing = 0; //0 = ignore, 1 = error
		bool mincheck = false;
		int minval = 0;
		bool maxcheck = false;
		int maxval = 0;
		int defaultval = 0;
	};

	int MEDProcessingInteger(const std::vector<std::string> &data, unsigned i, MEDProcessingDoubleOptions opt, int &val)
	{
		//missing: 0 = ignore, 1 = error
		std::string item;
		bool isMissing;

		item = data[i];
		isMissing = (item == "");

		if (isMissing)
		{
			//Error
			if (opt.missing == 1)
			{
				return 1;
			}
			//Ignore
			else
			{
				return 0;
			}
		}

		int valtemp;

		if (sscanf(item.c_str(), "%d", &valtemp) != 1)
		{
			return 2;
		}

		//Limit checking
		if (opt.mincheck)
		{
			if (valtemp < opt.minval)
			{
				return 2;
			}
		}
		if (opt.maxcheck)
		{
			if (valtemp > opt.maxval)
			{
				return 2;
			}
		}

		val = valtemp;
		return 0;
	}

	int MEDProcessingTime(const std::vector<std::string> &data, unsigned i, MEDProcessingDoubleOptions opt, double &secs)
	{
		//missing: 0 = ignore, 1 = error
		std::string item;
		bool isMissing;

		item = data[i];
		isMissing = (item == "");

		if (isMissing)
		{
			//Error
			if (opt.missing == 1)
			{
				return 1;
			}
			//Ignore
			else
			{
				return 0;
			}
		}

		double valtemp, ss;
		int hh, mm;
		bool pos;

		if (sscanf(item.c_str(), "%d:%d:%lf", &hh, &mm, &ss) != 3)
		{
			return 2;
		}

		pos = true;
		if (item[0] == '-')
		{
			pos = false;
			hh = abs(hh);
		}
		valtemp = 3600.0*(double)hh + 60.0*(double)mm + ss;
		if (pos == false)
		{
			valtemp = -valtemp;
		}

		//Limit checking
		if (opt.mincheck)
		{
			if (valtemp < opt.minval)
			{
				return 3;
			}
		}
		if (opt.maxcheck)
		{
			if (valtemp > opt.maxval)
			{
				return 4;
			}
		}

		secs = valtemp + opt.scale;
		return 0;
	}

}