#include <iostream>
#include <memory>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>
#include <mocap_optitrack/socket.h>
#include <natnet/natnet_messages.h>



int main(int argc, char **argv) {
    
    

    
    std::cout << "Hello, world!" << std::endl;
    
    mocap_optitrack::ServerDescription serverDescription;
    serverDescription.dataPort = 9000;
    serverDescription.commandPort = 1510;
    serverDescription.multicastIpAddress = "224.0.0.1";
    
    mocap_optitrack::DataModel dataModel;
    std::unique_ptr<UdpMulticastSocket> multicastClientSocketPtr;
    
    
      // Create socket
      multicastClientSocketPtr.reset( new UdpMulticastSocket(serverDescription.dataPort, serverDescription.multicastIpAddress) ); 

      if (!serverDescription.version.empty())
      {
        dataModel.setVersions(&serverDescription.version[0], &serverDescription.version[0]);
      }

      // Need verion information from the server to properly decode any of their packets.
      // If we have not recieved that yet, send another request.  
      while(!dataModel.hasServerInfo())
      {
        natnet::ConnectionRequestMessage connectionRequestMsg;
        natnet::MessageBuffer connectionRequestMsgBuffer;
        connectionRequestMsg.serialize(connectionRequestMsgBuffer, NULL);
        
        int ret = multicastClientSocketPtr->send(&connectionRequestMsgBuffer[0], connectionRequestMsgBuffer.size(), serverDescription.commandPort);

            
        int numBytesReceived = multicastClientSocketPtr->recv();
        if( numBytesReceived > 0 )
        {
            // Grab latest message buffer
            const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

            // Copy char* buffer into MessageBuffer and dispatch to be deserialized
            natnet::MessageBuffer msgBuffer(pMsgBuffer, pMsgBuffer + numBytesReceived);
            natnet::MessageDispatcher::dispatch(msgBuffer, &dataModel);
        
            usleep(10);
        }
      }

      DSA_INFO("Initialization complete");
    
    return 0;
}
