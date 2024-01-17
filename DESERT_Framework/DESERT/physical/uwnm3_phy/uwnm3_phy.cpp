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
 * @file   uwnm3_phy.cpp
 * @author Benjamin Sherlock
 * @version 1.0.0
 *
 * \brief Implementation of UwAhoiPhy class
 *
 */

#include "uwnm3_phy.h"
#include <vector>

/**
 * Adds the module for UwNm3PhyClass in ns2.
 */
static class UwNm3PhyClass : public TclClass
{
public:
	UwNm3PhyClass()
		: TclClass("Module/UW/NM3/PHY")
	{
	}
	TclObject *
	create(int, const char *const *)
	{
		return (new UwNm3Phy);
	}
} class_module_UwNm3Phy;

UwNm3Phy::UwNm3Phy()
	: UnderwaterPhysical()
{ // binding to TCL variables
    inititaliseLUT();
}

UwNm3Phy::~UwNm3Phy()
{
}

int
UwNm3Phy::command(int argc, const char *const *argv)
{
	return UnderwaterPhysical::command(argc, argv);
}





void 
UwNm3Phy::endTx(Packet *p)
{
	// Handles the end of a packet transmission
	hdr_cmn *ch = HDR_CMN(p);
	hdr_MPhy *ph = HDR_MPHY(p);
	hdr_mac *mach = HDR_MAC(p);
}

void 
UwNm3Phy::endRx(Packet *p)
{
	// Handles the end of a packet reception
	hdr_cmn *ch = HDR_CMN(p);
	hdr_MPhy *ph = HDR_MPHY(p);
	hdr_mac *mach = HDR_MAC(p);
	
	// Send message up a layer to get the mac address of this node.
	static int mac_addr = -1;
	ClMsgPhy2MacAddr msg;
	sendSyncClMsg(&msg);
	mac_addr = msg.getAddr();
}

void 
UwNm3Phy::startRx(Packet *p)
{
	// Handles the start of a reception. This method is called from the recv method
	hdr_cmn *ch = HDR_CMN(p);
	hdr_MPhy *ph = HDR_MPHY(p);
	hdr_mac *mach = HDR_MAC(p);
}


/*
void
UwNm3Phy::endRx(Packet *p)
{

	hdr_cmn *ch = HDR_CMN(p);
	hdr_MPhy *ph = HDR_MPHY(p);
	hdr_mac *mach = HDR_MAC(p);
	counter interferent_pkts;
	static int mac_addr = -1;
	ClMsgPhy2MacAddr msg;
	sendSyncClMsg(&msg);
	mac_addr = msg.getAddr();
	if (PktRx != 0) {
		if (PktRx == p) {
			double per_ni; // packet error rate due to noise and/or interference
			double per_n; // packet error rate due to noise only
			double x = RNG::defaultrng()->uniform_double();
			per_n = getPER(ph->Pr / ph->Pn, p);
			bool error_n = x <= per_n;
			error_n = 0;
			bool error_ni = 0;
			double interference = interference_ ? 
				interference_->getInterferencePower(p):0;
			if (!error_n) {
				per_ni = interference?getPER(ph->Pr/interference,p):getPER(0,p); 
				error_ni = x <= per_ni;
			}
			if (time_ready_to_end_rx_ > Scheduler::instance().clock()) {
				Rx_Time_ = Rx_Time_ + ph->duration - time_ready_to_end_rx_ +
						Scheduler::instance().clock();
			} else {
				Rx_Time_ += ph->duration;
			}
			time_ready_to_end_rx_ =
					Scheduler::instance().clock() + ph->duration;
			Energy_Rx_ += consumedEnergyRx(ph->duration);

			ch->error() = error_ni || error_n;
			if (debug_) {
				if (error_ni == 1) {
					std::cout
							<< NOW << "  UwAhoiPhy(" << mac_addr
							<< ")::endRx() packet " << ch->uid()
							<< " contains errors due to noise and interference."
							<< std::endl;
				} else if (error_n == 1) {
					std::cout << NOW << "  UwAhoiPhy(" << mac_addr
							  << ")::endRx() packet " << ch->uid()
							  << " contains errors due to noise." << std::endl;
				}
			}
			if (error_n) {
				incrErrorPktsNoise();
				if (mach->ftype() != MF_CONTROL) {
					incrTot_pkts_lost();
				} else if (mach->ftype() == MF_CONTROL) {
					incrTotCrtl_pkts_lost();
				}
			} else if (error_ni) {
				if (mach->ftype() != MF_CONTROL) {
					incrErrorPktsInterf();
					incrTot_pkts_lost();
					if (interferent_pkts.second >= 1) {
						incrCollisionDATA();
					} else {
						if (interferent_pkts.first > 0) {
							incrCollisionDATAvsCTRL();
						}
					}
				} else if (mach->ftype() == MF_CONTROL) {
					incrTotCrtl_pkts_lost();
					incrErrorCtrlPktsInterf();
					if (interferent_pkts.first > 0) {
						incrCollisionCTRL();
					}
				}
			}
			sendUp(p);
			PktRx = 0;
		} else {
			dropPacket(p);
		}
	} else {
		dropPacket(p);
	}
}
*/


bool 
UwNm3Phy::inititaliseLUT()
{
    /**
     * Receiver Performance to determine packet success probability
     * Lookup tables here based on varying data payload lengths and varying multipath severity.
     * Check curves in paper - Fig. 12.
     * Tables generated for the paper: 20190827-nm3paperdata.git 03-nm3packetsim\results
     * Anything below -9dB is lost, anything above 9dB is 100% successful.
     * Everything else is within the lookup table.
     * Multipath configurations (1,1,0) is a harsh multipath scenario.
     * (1,0,0) is less so. (0,0,0) is just AWGN.
     * Multipath level - 0=(0,0,0)/AWGN, 1=(1,0,0), 2=(1,1,0)
     */
      
    // Multipath Level => Payload Length => SNR => PER
    
    m_lutMultipathValues = std::vector<int>({0, 1, 2});
    m_lutPayloadValues = std::vector<int>({4, 8, 16, 32, 64});
    m_lutSnrValues = std::vector<double>({-9.0, -8.0, -7.0, -6.0, -5.0 , -4.0, -3.0, 
                                          -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 
                                          6.0, 7.0, 8.0, 9.0});

    
    m_lutPerValues = {
        { // Multipath Level 0
            { // Payload Value 4
                0.919, 0.736, 0.465, 0.236, 0.076,
                0.012, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L4 (0,0,0)
            },
            { // Payload Value 8, 
                0.946, 0.732, 0.484, 0.234, 0.074,
                0.013, 0.002, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L8 (0,0,0)
            },
            { // Payload Value 16
                0.984, 0.777, 0.482, 0.225, 0.070,
                0.017, 0.001, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L16 (0,0,0)
            },
            { // Payload Value 32
                1.000, 0.910, 0.469, 0.228, 0.074,
                0.011, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L32 (0,0,0)
            },
            { // Payload Value 64
                1.000, 0.997, 0.644, 0.221, 0.066,
                0.011, 0.001, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L64 (0,0,0)
            }
        }, // Multipath Level 0
        { // Multipath Level 1
            { // Payload Value 4
                1.000, 1.000, 0.998, 0.960, 0.776,
                0.434, 0.200, 0.062, 0.013, 0.001,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L4 (1,0,0)
            },
            { // Payload Value 8
                1.000, 1.000, 0.999, 0.981, 0.816,
                0.456, 0.194, 0.062, 0.011, 0.001,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L8 (1,0,0)
            },
            { // Payload Value 16
                1.000, 1.000, 1.000, 0.998, 0.944,
                0.605, 0.228, 0.061, 0.011, 0.001,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L16 (1,0,0)
            },
            { // Payload Value 32
                1.000, 1.000, 1.000, 1.000, 0.998,
                0.897, 0.388, 0.080, 0.013, 0.002,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L32 (1,0,0)
            },
            { // Payload Value 64
                1.000, 1.000, 1.000, 1.000, 1.000,
                0.998, 0.856, 0.270, 0.024, 0.001,
                0.000, 0.000, 0.000, 0.000, 0.000,
                0.000, 0.000, 0.000, 0.000 // # L64 (1,0,0)
            } 
        }, // Multipath Level 1
        { // Multipath Level 2
            { // Payload Value 4 - These are the values for L16 repeated
                1.000, 1.000, 1.000, 1.000, 1.000,
                0.998, 0.944, 0.717, 0.412, 0.161,
                0.047, 0.015, 0.006, 0.003, 0.001,
                0.001, 0.000, 0.000, 0.000 // # L16 (1,1,0)
            },
            { // Payload Value 8 - These are the values for L16 repeated
                1.000, 1.000, 1.000, 1.000, 1.000,
                0.998, 0.944, 0.717, 0.412, 0.161,
                0.047, 0.015, 0.006, 0.003, 0.001,
                0.001, 0.000, 0.000, 0.000 // # L16 (1,1,0)
            },
            { // Payload Value 16
                1.000, 1.000, 1.000, 1.000, 1.000,
                0.998, 0.944, 0.717, 0.412, 0.161,
                0.047, 0.015, 0.006, 0.003, 0.001,
                0.001, 0.000, 0.000, 0.000 // # L16 (1,1,0)
            },
            { // Payload Value 32
                1.000, 1.000, 1.000, 1.000, 1.000,
                1.000, 0.997, 0.952, 0.804, 0.553,
                0.245, 0.089, 0.020, 0.010, 0.003,
                0.001, 0.000, 0.000, 0.000 // # L32 (1,1,0)
            },
            { // Payload Value 64
                1.000, 1.000, 1.000, 1.000, 1.000,
                1.000, 1.000, 0.999, 0.984, 0.932,
                0.793, 0.483, 0.217, 0.068, 0.020,
                0.007, 0.002, 0.002, 0.000 // # L64 (1,1,0)
            }
        } // Multipath Level 2
    };
    
    m_lutIsInitialised = true;       
}



double
UwNm3Phy::getDistance(Packet *_p)
{
	hdr_MPhy *ph = HDR_MPHY(_p);
	double x_src = (ph->srcPosition)->getX();
	double y_src = (ph->srcPosition)->getY();
	double z_src = (ph->srcPosition)->getZ();
	double x_dst = (ph->dstPosition)->getX();
	double y_dst = (ph->dstPosition)->getY();
	double z_dst = (ph->dstPosition)->getZ();
	return sqrt(pow(x_src - x_dst, 2.0) + pow(y_src - y_dst, 2.0) +
			pow(z_src - z_dst, 2.0));
}


double 
UwNm3Phy::getPER(double snrDb, int payloadLength, int multipathLevel)
{
    int snrDbIndex = -1;
    int payloadLengthIndex = -1;
    int multipathLevelIndex = -1;

    // Check limits of the parameters
    if (snrDb < m_lutSnrValues[0])
    {
        return 1.0; // SNR is less than the minimum so 100% packet failure. 
    }    
    else if (snrDb > m_lutSnrValues[m_lutSnrValues.size()-1])
    {
        return 0.0; // SNR is greater than the maximum so 100% packet success.
    }
    else
    {
        // Search for the appropriate SNR value to use.
        for (size_t idx = 1; idx < m_lutSnrValues.size(); idx++)
        {
            if (snrDb < m_lutSnrValues[idx])
            {
                snrDbIndex = idx-1; // Take the SNR value below (worse case). 
                break;
            }
        }
    }
    
    // Clamp inputs to legitimate values
    if (payloadLength < m_lutPayloadValues[0])
    {
        payloadLength = m_lutPayloadValues[0];
        payloadLengthIndex = 0;
    }
    else if (payloadLength > m_lutPayloadValues[m_lutPayloadValues.size()-1])
    {
        payloadLength = m_lutPayloadValues[m_lutPayloadValues.size()-1];
        payloadLengthIndex = m_lutPayloadValues.size()-1;
    }
    else
    {
        // Search for the appropriate LUT payload length to use.
        for (size_t idx = 0; idx < m_lutPayloadValues.size(); idx++)
        {
            if (payloadLength < m_lutPayloadValues[idx])
            {
                payloadLengthIndex = idx;
                break;
            }
        }
    }
    
    if (multipathLevel < m_lutMultipathValues[0])
    {
        multipathLevel = m_lutMultipathValues[0];
        multipathLevelIndex = 0;
    }    
    else if (multipathLevel > m_lutMultipathValues[m_lutMultipathValues.size()-1])
    {
        multipathLevel = m_lutMultipathValues[m_lutMultipathValues.size()-1];
        multipathLevelIndex = m_lutMultipathValues.size()-1;
    }
    else
    {
        // Search for the appropriate multipath level to use.
        multipathLevelIndex = multipathLevel;
    }
    
    // Retrieve the PER value for the given indices.
    double perValue = m_lutPerValues[multipathLevelIndex][payloadLengthIndex][snrDbIndex];
    
    return perValue;
}


