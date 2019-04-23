/*
 * Copyright (c) 2019, Markus Bader
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



#include <iostream>
#include <memory>
#include <motion_capture/data_model.h>
#include <motion_capture/natnet_messages.h>
#include <motion_capture/optitrack.h>
#include <motion_capture/udphdl.h>

#define PRINT_INFO(...) printf(__VA_ARGS__); printf("\n")

namespace motion_capture {

OptiTrack::OptiTrack() {
}

bool OptiTrack::init ( int commandPort,  int dataPort, const std::string &multicastIpAddress ) {

    DataModelPtr dataModel;
    serverDescription.reset(new motion_capture::ServerDescription(commandPort, dataPort, multicastIpAddress));
    dataModel.reset( new motion_capture::DataModel);
    
     
    // Create socket
    udpHdl.reset(new motion_capture::UDPHdl);
    udpHdl->initBidirektional(serverDescription->multicastIpAddress, serverDescription->dataPort, serverDescription->commandPort, 10);
    udpHdl->runThread();

    if ( !serverDescription->version.empty() ) {
        dataModel->setVersions ( &serverDescription->version[0], &serverDescription->version[0] );
    }

    // Need verion information from the server to properly decode any of their packets.
    // If we have not recieved that yet, send another request.
    
    motion_capture::natnet::ConnectionRequestMessage connectionRequestMsg;
    motion_capture::natnet::MessageBuffer connectionRequestMsgBuffer;
    connectionRequestMsg.serialize ( connectionRequestMsgBuffer, NULL );
    
    while ( !dataModel->hasServerInfo() ) {

        udpHdl->send(&connectionRequestMsgBuffer[0], connectionRequestMsgBuffer.size());

        while ( udpHdl->nrOfQueuedMsg() > 0 ) {
            // Grab latest message buffer
            motion_capture::natnet::MessageBuffer msgBuffer; 
            if(udpHdl->deque(msgBuffer) > 0) {
                PRINT_INFO ( "msgBuffer received: %zu byte", msgBuffer.size() );
                
                motion_capture::natnet::MessageDispatcher::dispatch ( msgBuffer, dataModel.get() );
                
                
                if(msgBuffer.size() == 4){                
                    PRINT_INFO ( "received:  %d, %d, %d, %d\n", (uint8_t) msgBuffer[0], (uint8_t) msgBuffer[1], (uint8_t) msgBuffer[2], (uint8_t) msgBuffer[3]);
                }
            }
            usleep ( 10 );
        }
    }
    serverInfo.reset(new motion_capture::ServerInfo(dataModel->getServerInfo()));
}


bool OptiTrack::receive (std::vector<DataModelPtr> &data){
    
        data.clear();
        while ( udpHdl->nrOfQueuedMsg() > 0 ) {
            PRINT_INFO ( "package received" );
            
            DataModelPtr dataModel(new motion_capture::DataModel(*serverInfo));
            motion_capture::natnet::MessageBuffer msgBuffer;
            udpHdl->deque(msgBuffer);
            
            motion_capture::natnet::MessageDispatcher::dispatch ( msgBuffer, dataModel.get() );
            data.push_back(dataModel);
        }
    
}
}
