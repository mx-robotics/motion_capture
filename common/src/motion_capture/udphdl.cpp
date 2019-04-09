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


#include <motion_capture/udphdl.h>


#define UDPHDL_PRINT_ERROR(...) printf(__VA_ARGS__); printf("\n")
#define UDPHDL_PRINT_WARN(...) printf(__VA_ARGS__); printf("\n")
#define UDPHDL_PRINT_INFO(...) printf(__VA_ARGS__); printf("\n")


namespace motion_capture {
    
    UDPHdl::UDPHdl()
    : state_(State_NA)
    , use_callback_(false)
    {
    }
    
    
    ///Destructor
    UDPHdl::~UDPHdl() {
        if ( thread_receiverPtr_.get() != NULL ) thread_receiverPtr_->join();
    }
    
    int UDPHdl::initBidirektional (const std::string &host, unsigned short local_port, unsigned short remote_port, int queySize ){
        if ( socketPtr_.get() ) {
            UDPHDL_PRINT_WARN("UDPHdl::initBidirektional: class already initialized!\n");
            return 1;
        }
        UDPHDL_PRINT_INFO("UDPHdl::initBidirektional: local port:%hu  destination  %s:%hu \n", local_port, host.c_str(), remote_port);
        try {                
            endpoint_ = udp::endpoint(boost::asio::ip::address::from_string(host), remote_port);
            socketPtr_ = SocketPtr ( new udp::socket ( ioService_, udp::endpoint ( udp::v4(), local_port ) ) );
            max_queue = queySize;
            bindFnc();
            state_ = State_BIDIREKTIONAL;
        } catch ( std::exception& e ) {
            UDPHDL_PRINT_ERROR("UDPHdl::initTransmitter: Exception %s\n", e.what());
            return 1;
        }        
        return 0;
    }
    
    int UDPHdl::initReceiver ( unsigned short port, int queySize ) {
        if ( socketPtr_.get() ) {
            UDPHDL_PRINT_WARN("UDPHdl::initReceiver: class already initialized!\n");
            return 1;
        }
        try {
            socketPtr_ = SocketPtr ( new udp::socket ( ioService_, udp::endpoint ( udp::v4(), port ) ) );
            max_queue = queySize;
            bindFnc();
            state_ = State_RECEIVER;
        } catch ( std::exception& e ) {
            UDPHDL_PRINT_ERROR("UDPHdl::initReceiver: Exception %s\n", e.what());
            return 1;
        }
        return 0;
    }


    int UDPHdl::initTransmitter (const std::string &host, unsigned short port) {
        if ( socketPtr_.get() ) {
            UDPHDL_PRINT_WARN("UDPHdl::initTransmitter: class already initialized!\n");
            return 1;
        }
        UDPHDL_PRINT_ERROR("UDPHdl::initTransmitter: host  %s:%hu \n", host.c_str(), port);
        try {
            socketPtr_ = SocketPtr ( new udp::socket ( ioService_, udp::endpoint ( udp::v4(), 0 ) ) );
            resolverPtr = ResolverPtr ( new udp::resolver ( ioService_ ) );
            queryPtr = QueryPtr ( new udp::resolver::query ( udp::v4(), host, boost::lexical_cast<std::string> ( port ) ) );
            state_ = State_TRANSMITTER;
        } catch ( std::exception& e ) {
            UDPHDL_PRINT_ERROR("Exception: %s\n", e.what());
            return 1;
        }
        return 0;
    }


    void UDPHdl::send ( const std::shared_ptr< std::vector<char> > &data ) {
        send(&data->at(0), data->size());
    }
    
    void UDPHdl::send ( const char* data, size_t size ){ 
        switch ( state_ ) {
        case  State_TRANSMITTER:  
            {
                udp::resolver::iterator iterator_;
                iterator_ = resolverPtr->resolve ( *queryPtr );
                UDPHDL_PRINT_INFO("UDPHdl::send %zu bytes\n", size);
                socketPtr_->send_to ( boost::asio::buffer ( data, size ), *iterator_ );
            }
            break;
        case  State_BIDIREKTIONAL:
            {
                boost::system::error_code err;
                size_t sent = socketPtr_->send_to(boost::asio::buffer(data, size), endpoint_, 0, err);  
                UDPHDL_PRINT_INFO("UDPHdl::send %zu bytes\n", sent);
            }
            break;
        default:
            UDPHDL_PRINT_ERROR("UDPHdl::send No mode selected");
        }        
    }

    void UDPHdl::run() {
        ioService_.run();
    }

    void UDPHdl::runThread() {
        if ( thread_receiverPtr_.get() == NULL ) {
            thread_receiverPtr_ = ThreadPtr ( new boost::thread ( boost::bind ( &motion_capture::UDPHdl::run, this ) ) );
        } else {
            UDPHDL_PRINT_WARN("UDPHdl::runThread, thread exists already!\n");
        }
    }

    void UDPHdl::stopThread() {
        if ( thread_receiverPtr_.get() != NULL ) {
            ioService_.stop();
            thread_receiverPtr_->join();
            thread_receiverPtr_.reset();
        } else {
            UDPHDL_PRINT_WARN("UDPHdl::stopThread, no thread to stop!\n");
        }
    }

    int UDPHdl::checkState ( State state, short port, std::string host) {
        if ( !socketPtr_.get() ) return 1;
        if ( state_ != state ) return 2;
        short local_port;
        switch ( state_ ) {
        case  State_BIDIREKTIONAL:
        case  State_TRANSMITTER:
            if ( boost::lexical_cast<std::string> ( port ).compare ( queryPtr->service_name() ) != 0 ) return 3;
            if ( host.compare ( queryPtr->host_name() ) != 0 ) return 4;
            break;
        case  State_RECEIVER:
            socketPtr_->local_endpoint().port();
            if ( port != local_port ) return 3;
            break;
        case State_NA:
            break;
        default:
            return 5;
        }
        return 0;
    }

    bool UDPHdl::isRunning() {
        if ( socketPtr_.get() ) {
            return 1;
        } else {
            return 0;
        }
    }

    unsigned int UDPHdl::nrOfQueuedMsg() {
        return pkg_queue_.size();
    }

    unsigned int UDPHdl::deque (std::shared_ptr<std::vector<char> > &pkg, bool front) {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock ( mutex_ );
        unsigned int size = pkg_queue_.size();
        if ( size > 0 ) {
            if ( front ) {
                pkg = pkg_queue_.front();
                pkg_queue_.pop_front();
            } else {
                pkg = pkg_queue_.back();
                pkg_queue_.pop_back();
            }
        }
        return size;
    }
    unsigned int UDPHdl::deque (std::vector<char> &pkg, bool front) {
        std::shared_ptr<std::vector<char> > tmp;
        unsigned int size = deque (tmp, front);
        if(size > 0) {
            pkg = *tmp.get();
        }
        return size;
    }

    
    UDPHdl::State UDPHdl::getState() {
        return state_;
    }

    void UDPHdl::setCallback ( boost::function<void ( std::shared_ptr< std::vector<char> > & ) > f ) {
        use_callback_ = true;
        callback_ = f;
    }
    
    void UDPHdl::handle_receive_from ( const boost::system::error_code& error, size_t bytes_recvd ) {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock ( mutex_ );
        if ( ( !error && bytes_recvd > 0 ) ) {
            std::shared_ptr< std::vector<char> > package = std::make_shared<std::vector<char> > (receiveBuffer_, receiveBuffer_ + bytes_recvd);
            while ( pkg_queue_.size() > max_queue ) {
                pkg_queue_.pop_back();
            }
            pkg_queue_.push_front ( package );
            if ( use_callback_ ) callback_ ( package );
        } else {
            UDPHDL_PRINT_ERROR("Error: on object received %zu bytes received\n", bytes_recvd);
        }
        bindFnc();
    }

};

