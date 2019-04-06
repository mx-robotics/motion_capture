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
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>
#include <mocap_optitrack/socket.h>
#include <natnet/natnet_messages.h>


int main ( int argc, char **argv ) {

    std::cout << "Hello, world!" << std::endl;

    mocap_optitrack::ServerDescription serverDescription;

    mocap_optitrack::DataModel dataModel;
    std::unique_ptr<UdpMulticastSocket> multicastClientSocketPtr;


    // Create socket
    multicastClientSocketPtr.reset ( new UdpMulticastSocket ( serverDescription.dataPort, serverDescription.multicastIpAddress ) );

    if ( !serverDescription.version.empty() ) {
        dataModel.setVersions ( &serverDescription.version[0], &serverDescription.version[0] );
    }

    // Need verion information from the server to properly decode any of their packets.
    // If we have not recieved that yet, send another request.
    while ( !dataModel.hasServerInfo() ) {
        natnet::ConnectionRequestMessage connectionRequestMsg;
        natnet::MessageBuffer connectionRequestMsgBuffer;
        connectionRequestMsg.serialize ( connectionRequestMsgBuffer, NULL );

        int ret = multicastClientSocketPtr->send ( &connectionRequestMsgBuffer[0], connectionRequestMsgBuffer.size(), serverDescription.commandPort );


        int numBytesReceived = multicastClientSocketPtr->recv();
        if ( numBytesReceived > 0 ) {
            // Grab latest message buffer
            const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

            // Copy char* buffer into MessageBuffer and dispatch to be deserialized
            natnet::MessageBuffer msgBuffer ( pMsgBuffer, pMsgBuffer + numBytesReceived );
            natnet::MessageDispatcher::dispatch ( msgBuffer, &dataModel );

            usleep ( 10 );
        }
    }

    DSA_INFO ( "Initialization complete" );

    while ( true ) {
        // Get data from mocap server
        int numBytesReceived = multicastClientSocketPtr->recv();
        if ( numBytesReceived > 0 ) {
            DSA_INFO ( "package received" );
            // Grab latest message buffer
            const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

            // Copy char* buffer into MessageBuffer and dispatch to be deserialized
            natnet::MessageBuffer msgBuffer ( pMsgBuffer, pMsgBuffer + numBytesReceived );
            natnet::MessageDispatcher::dispatch ( msgBuffer, &dataModel );
        }
    }


    return 0;
}
