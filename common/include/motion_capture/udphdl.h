/***************************************************************************
 *   Software License Agreement (BSD License)                              *  
 *   Copyright (C) 2012 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef UDPHDL_H_
#define UDPHDL_H_

#include <iostream>
#include <deque>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/concept_check.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>



namespace motion_capture {
using boost::asio::ip::udp;
///Simple class to handle UDP messages
class UDPHdl {
    typedef std::shared_ptr<boost::thread> ThreadPtr;
    typedef std::shared_ptr<udp::socket> SocketPtr;
    typedef std::shared_ptr<udp::resolver::query> QueryPtr;
    typedef std::shared_ptr<udp::resolver> ResolverPtr;
public:
    
    enum State {
        State_NA = 0,
        State_RECEIVER = 1,
        State_TRANSMITTER = 2,
        State_BIDIREKTIONAL = 3,
    };
    /**
     * Constructor
     * @post initReceiver or initTransmitter
     **/
    UDPHdl();
    
    ///Destructor
    ~UDPHdl() ;
    
    /**
     * Initializes the class as transmitter (sender) and receiver to define both ports
     * @param host
     * @param local_port
     * @param remote_port
     * @param queySize
     */
    int initBidirektional (const std::string &host, unsigned short local_port, unsigned short remote_port, int queySize = 1 ) ;
    
    /**
     * Initializes the class as receiver
     * @param port
     * @param queySize
     */
    int initReceiver ( unsigned short port, int queySize = 1 ) ;

    /**
     * Initializes the class as transmitter (sender)
     * @param host
     * @param port
     */
    int initTransmitter (const std::string &host, unsigned short port) ;
    

    /**
     * Sends a object
     * @param data
     */
    void send ( const std::shared_ptr< std::vector<char> > &data ) ;
    
    /**
     * Sends a object
     * @param data
     * @param size
     */
    void send ( const char* data, size_t size ) ;
    
    /**
     * starts with the receiving
     */
    void run() ;
    /**
     * starts a thread to receive
     */
    void runThread() ;
    /**
     * stops the thread to receive
     */
    void stopThread() ;

    /**
     * returns zero if the com runs with the argument configuration
     * @param state
     * @param port
     * @param host
     * @return 0 on match, 1 if thread is not running, 2 on state missmatch
     * 3 on port missmatch, 4 on hostname missmatch, 5 state does not exist
     */
    int checkState ( State state, short port, std::string host = "" ) ;

    /**
     * returns ture if a thread is running
     */
    bool isRunning() ;

    /**
     * Number of currently queued msg
     **/
    unsigned int nrOfQueuedMsg() ;

    /**
     * deques a element threadsave
     * @param pkg holds afterwared the pointer to the object
     * @param front on false it will deque the oldest message
     */
    unsigned int deque (std::shared_ptr<std::vector<char> > &pkg, bool front = true ) ;
    

    /**
     * deques a element threadsave
     * @param msg copy of the dequed object
     * @param front on false it will deque the oldest message
     */
    unsigned int deque (std::vector<char> &pkg, bool front = true ) ;
    

    /**
     * Returns the configuration send or receiver
     * @see NA, RECEIVER, TRANSMITTER
     */
    State getState() ;
    /**
     * Sets a callbackfunction which is called on incommig messages
     * @param f
     * @code udp_.setCallback(boost::bind(&CDS::Crane::Receiver::receiveCallback, this, _1));
     */
    void setCallback ( boost::function<void ( std::shared_ptr< std::vector<char> > & ) > f ) ;

private:
    inline void bindFnc() {
        try {
            socketPtr_->async_receive_from ( boost::asio::buffer ( receiveBuffer_, MAX_PACKAGE_LENGTH ), endpoint_, boost::bind (
                                                 &UDPHdl::handle_receive_from, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

        } catch ( std::exception& e ) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    void handle_receive_from ( const boost::system::error_code& error, size_t bytes_recvd );

private:
    static const size_t MAX_PACKAGE_LENGTH = 0xFFFF;
    State state_;
    bool use_callback_;
    boost::asio::io_service ioService_;
    SocketPtr socketPtr_;
    QueryPtr queryPtr;
    ResolverPtr resolverPtr;
    udp::endpoint endpoint_;
    void *pReceiveFncPtr_;
    
    char receiveBuffer_[MAX_PACKAGE_LENGTH];
    ThreadPtr thread_receiverPtr_;
    std::deque<std::shared_ptr< std::vector<char> > > pkg_queue_;
    unsigned int max_queue;
    boost::interprocess::interprocess_mutex mutex_; /// used to prevent concurrent access to the data

    boost::function<void ( std::shared_ptr< std::vector<char> > & ) > callback_;

};
};

#endif /* UDPHDL_H_ */
