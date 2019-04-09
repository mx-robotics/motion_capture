/* 
 * Copyright (c) 2019, Markus Bader
 * Copyright (c) 2011 University of Bonn, Computer Science Institute, 
 * Kathrin Gräve
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __MOCAP_OPTITRACK_H__
#define __MOCAP_OPTITRACK_H__

#include <string>
#include <vector>
#include <memory>
#include <motion_capture/data_model.h>

namespace motion_capture
{

class UdpMulticastSocket;
class DataModel;

/// \brief The data model for this node
class OptiTrack 
{
public:
    OptiTrack();


    bool init (int commandPort = 1510,  int dataPort = 9000, const std::string &multicastIpAddress = "224.0.0.1"); 
    bool receive (); 
private:
    std::shared_ptr<motion_capture::ServerDescription> serverDescription;
    std::shared_ptr<motion_capture::DataModel> dataModel;
    std::shared_ptr<motion_capture::UdpMulticastSocket> multicastClientSocketPtr;
};

}

#endif
