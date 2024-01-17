//
// Copyright (c) 2017 Regents of the SIGNET lab, University of Padova.
// Copyright (c) 2024 Benjamin Sherlock, Newcastle University, UK.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Padova (SIGNET lab) nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @file   uwnm3_phy.h
 * @author Benjamin Sherlock
 * @version 1.0.0
 *
 * \brief Definition of UwNm3Phy class.
 *
**/

#ifndef UWNM3_PHY_H
#define UWNM3_PHY_H

#include "uwphysical.h"
#include <math.h>
#include <iostream>
#include <map>
#include <vector>


class UwNm3Phy : public UnderwaterPhysical
{

public:
	/**
	 * Constructor of UwNm3Phy class.
	 */
	UwNm3Phy();

	/**
	 * Destructor of UwNm3Phy class.
	 */
	virtual ~UwNm3Phy();

	/**
   * TCL command interpreter. It implements the following OTcl methods:
   *
   * @param argc Number of arguments in <i>argv</i>.
   * @param argv Array of strings which are the command parameters (Note that
   * <i>argv[0]</i> is the name of the object).
   * @return TCL_OK or TCL_ERROR whether the command has been dispatched
   * successfully or not.
   *
   */
	virtual int command(int, const char *const *);

protected:
	/**
	 * Handles the end of a packet transmission
	 *
	 * @param Packet* p Pointer to the packet transmitted
	 *
	 */
	virtual void endTx(Packet *p);
	/**
	 * Handles the end of a packet reception
	 *
	 * @param Packet* p Pointer to the packet received
	 *
	 */
	virtual void endRx(Packet *p);
	/**
	 * Handles the start of a reception. This method is called from the recv
	 * method
	 *
	 * @param Packet* p Pointer to the packet that the PHY are receiving.
	 *
	 */
	virtual void startRx(Packet *p);

	
	/**
	 * Get the Packet Error Rate for a given SNR, Payload Length, and Multipath severity.
	 *
	 * @param double snrDb SNR value at the receiver
 	 * @param int payloadLength bytes length of the packet payload
 	 * @param int multipathLevel multipath severity (0,1,2)
	 *
	 */
	double getPER(double snrDb, int payloadLength = 32, int multipathLevel = 0);


private:

	/**
	 * Initialise the lookup table (LUT) for SNR vs PER.
	 */
	bool inititaliseLUT();

	/**
	* Return the distance between source and destination.
	*
	* @param p Packet by which the module gets information about source and
	*destination.
	**/
	virtual double getDistance(Packet *);

	


	std::vector<int> m_lutMultipathValues; // = std::vector<int>({0, 1, 2});
        std::vector<int> m_lutPayloadValues; // = std::vector<int>({4, 8, 16, 32, 64});
	std::vector<double> m_lutSnrValues; // LUT for the SNR values
	
	std::vector<std::vector<std::vector<double> > > m_lutPerValues; // LUT for all the PER values
	bool m_lutIsInitialised; // Flag for initialisation complete.

};

#endif /* UWNM3_PHY_H  */

