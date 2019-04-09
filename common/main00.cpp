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
#include <motion_capture/socket.h>
#include <motion_capture/natnet_messages.h>
#include <boost/program_options.hpp>

#define PRINT_INFO(...) printf(__VA_ARGS__); printf("\n")

struct Prarmeters {
    std::string host;
    unsigned int dataPort;
    unsigned int commandPort;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "host,h", po::value<std::string> ( &params.host )->default_value ( "224.0.0.1" ), "host mutlicast" )
    ( "dataPort,d", po::value<unsigned int> ( &params.dataPort )->default_value ( 9000 ), "optitrack data port" )
    ( "commandPort,c", po::value<unsigned int> ( &params.commandPort )->default_value ( 1510 ), "optitrack command port" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    return params;
}

int main ( int argc, char **argv ) {

    std::cout << "Hello, world!" << std::endl;
    Prarmeters params = readArgs ( argc, argv );

    motion_capture::ServerDescription serverDescription(params.commandPort, params.dataPort, params.host);

    motion_capture::DataModel dataModel;
    std::unique_ptr<motion_capture::UdpMulticastSocket> multicastClientSocketPtr;


    // Create socket
    multicastClientSocketPtr.reset ( new motion_capture::UdpMulticastSocket ( serverDescription.dataPort, serverDescription.multicastIpAddress ) );

    if ( !serverDescription.version.empty() ) {
        dataModel.setVersions ( &serverDescription.version[0], &serverDescription.version[0] );
    }

    // Need verion information from the server to properly decode any of their packets.
    // If we have not recieved that yet, send another request.
    while ( !dataModel.hasServerInfo() ) {
        motion_capture::natnet::ConnectionRequestMessage connectionRequestMsg;
        motion_capture::natnet::MessageBuffer sendBuffer;
        connectionRequestMsg.serialize ( sendBuffer, NULL );
        PRINT_INFO ( "sendBuffer:  %d, %d, %d, %d", (uint8_t) sendBuffer[0], (uint8_t) sendBuffer[1], (uint8_t) sendBuffer[2], (uint8_t) sendBuffer[3]);
    
        
        int ret = multicastClientSocketPtr->send ( &sendBuffer[0], sendBuffer.size(), serverDescription.commandPort );


        int numBytesReceived = multicastClientSocketPtr->recv();
        if ( numBytesReceived > 0 ) {
            // Grab latest message buffer
            PRINT_INFO ( "package received: %dbyte", numBytesReceived );
            const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

            if(numBytesReceived == 4){                
                PRINT_INFO ( "pMsgBuffer:  %d, %d, %d, %d\n", (uint8_t) pMsgBuffer[0], (uint8_t) pMsgBuffer[1], (uint8_t) pMsgBuffer[2], (uint8_t) pMsgBuffer[3]);
            }
            // Copy char* buffer into MessageBuffer and dispatch to be deserialized
            motion_capture::natnet::MessageBuffer msgBuffer ( pMsgBuffer, pMsgBuffer + numBytesReceived );
            motion_capture::natnet::MessageDispatcher::dispatch ( msgBuffer, &dataModel );

            usleep ( 10 );
        }
    }

    PRINT_INFO ( "Initialization complete" );

    while ( true ) {
        // Get data from mocap server
        int numBytesReceived = multicastClientSocketPtr->recv();
        if ( numBytesReceived > 0 ) {
            PRINT_INFO ( "package received: %dbyte", numBytesReceived );
            // Grab latest message buffer
            const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

            // Copy char* buffer into MessageBuffer and dispatch to be deserialized
            motion_capture::natnet::MessageBuffer msgBuffer ( pMsgBuffer, pMsgBuffer + numBytesReceived );
            motion_capture::natnet::MessageDispatcher::dispatch ( msgBuffer, &dataModel );
        }
    }


    return 0;
}
