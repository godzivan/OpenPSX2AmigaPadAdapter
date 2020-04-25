/*******************************************************************************
 * This file is part of OpenPSX2AmigaPadAdapter.                               *
 *                                                                             *
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is free software: you can redistribute it and/or    *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is distributed in the hope that it will be useful,  *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with OpenPSX2AmigaPadAdapter. If not, see http://www.gnu.org/licenses.*
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>

enum class AD5242_RDAC: uint8_t {
	RDAC1 = 0,
	RDAC2 = 1
};


class VariableResistor {
private:
	byte addr;
	byte outputs;
	byte point0;

	static const byte B_RDAC = 7;
	static const byte B_RESET = 6;
	static const byte B_SHUTDOWN = 5;
	static const byte B_OUTPUT1 = 4;
	static const byte B_OUTPUT2 = 3;

public:
	static const byte DEFAULT_ADDR = 0x2C;		// Add AD0/AD1

	boolean begin (const byte _addr = DEFAULT_ADDR) {
		boolean ret = false;

		Wire.begin (); // Join i2c bus

		Wire.beginTransmission (_addr);
		if (Wire.endTransmission () == 0) {
			// Found device
			addr = _addr;
			outputs = 0x00;
			point0 = 128;	// Internal power-on preset places wiper at midscale
			ret = true;
		}

		return ret;
	}

	void suspend () {
		Wire.end ();
	}

	void setValuePoint (const AD5242_RDAC rdac, const byte _point) {
		byte inst = outputs;
		if (rdac == AD5242_RDAC::RDAC2) {
			inst |= 1 << B_RDAC;	// A high selects RDAC2 for the dual-channel AD5242
		} else {
			// Save point se we can resend it in setOutputs()
			point0 = _point;
		}

		Wire.beginTransmission (addr);	// transmit to device
		Wire.write (inst);				// sends instruction byte
		Wire.write (_point);            // sends potentiometer value byte
		Wire.endTransmission ();		// stop transmitting
	}

	void setOutputs (const byte o1, const byte o2) {
		if (o1) {
			outputs |= 1 << B_OUTPUT1;
		} else {
			outputs &= ~(1 << B_OUTPUT1);
		}

		if (o2) {
			outputs |= 1 << B_OUTPUT2;
		} else {
			outputs &= ~(1 << B_OUTPUT2);
		}

		Wire.beginTransmission (addr);	// transmit to device
		Wire.write (outputs);			// sends instruction byte
		Wire.write (point0);            // sends potentiometer value byte
		Wire.endTransmission ();		// stop transmitting
	}

	void resetToMid (const AD5242_RDAC rdac) {
		byte inst = outputs | (1 << B_RESET);
		if (rdac == AD5242_RDAC::RDAC2) {
			inst |= 1 << B_RDAC;
		} else {
			// Save point se we don't mess up in setOutputs()
			point0 = 128;				// Midpoint, where Raw == Rwb
		}

		Wire.beginTransmission (addr);	// transmit to device
		Wire.write (inst);				// sends instruction byte
		Wire.write (point0);            // sends potentiometer value byte
		Wire.endTransmission ();		// stop transmitting
	}

	void read (byte& p1, byte& p2) {
		Wire.beginTransmission (addr);	// transmit to device
		Wire.write (outputs);			// Select RDAC1
		Wire.endTransmission (false);	// Switch to read mode
		Wire.requestFrom (addr, (byte) 1, (byte) 0);		// Read RDAC1 value
		while (Wire.available () < 1)
			;
		p1 = Wire.read ();
		Wire.beginTransmission (addr);
		Wire.write (outputs | (1 << B_RDAC));
		Wire.endTransmission (false);	// Switch to read mode
		Wire.requestFrom (addr, (byte) 1);// Read RDAC2 value
		while (Wire.available () < 1)
			;
		p2 = Wire.read ();
	}
};
