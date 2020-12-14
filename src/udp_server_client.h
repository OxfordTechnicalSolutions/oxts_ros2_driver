#ifndef UDP_SERVER_CLIENT
#define UDP_SERVER_CLIENT

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <iostream>


namespace networking_udp
{
    using boost::asio::ip::udp;
    using boost::asio::ip::address;
    typedef unsigned char byte;

    /*
        ---CLIENT---

        Simple UDP client program which receives data on a given port
    */
    class client
    {
        boost::asio::io_service io_service;
        udp::endpoint remote_endpoint;

        udp::socket socket;
        udp::endpoint local_endpoint;

    public:

        /*
            default constructor to defer setting the local endpoint to a later time. Not safe to use this
            constructor and not specific local port with 
        */
        client() :
            socket(io_service)      
        {}

        /*
            Constructor requires a port. This will be the local port a socket is opened on

            param:
            * port - the port on the local machine the socket will be opened on when receiving data
        */
        client(short local_port) :
            socket(io_service),
            local_endpoint(boost::asio::ip::udp::v4(), local_port)
        {}

        /*
            sets the local port of the client which is what will be used when receiving data.

            params:
            * local_port - this specifies the port which incoming data will be received on when receive_from
            and receive are called.
        */
        void set_local_port(short local_port)
        {
            local_endpoint = udp::endpoint(boost::asio::ip::udp::v4(), local_port);
        }

        /*
            This function is used to received a UDP packet

            params:
            * recv_buffer - the data buffer which is written to
            * size_of_buffer - specifies the size of the buffer which will receive the data, if the datagram is larger
            the extra data won't be stored
            * sender_endpoint - the endpoint of the sender, useful if you have multiple sources of incoming data to same 
            port and want to differentiate data based on sender.

            return:
            number of bytes received, if it fails prints error code and returns 0.
        */
        std::size_t receive_from(byte* recv_buffer, std::size_t size_of_buffer, udp::endpoint required_sender_endpoint)
        {		
            boost::system::error_code error_code;
            std::size_t bytes_received = 0;
            udp::endpoint sender_endpoint;
            try
            {
                // opens a IPv4 socket then binds to it - uses local system ip and port specified in constructor
                socket.open(udp::v4(), error_code);
                socket.bind(local_endpoint, error_code);
                do
                {
                    bytes_received = socket.receive_from(boost::asio::buffer(recv_buffer, size_of_buffer), sender_endpoint, 0, error_code);
                } while (sender_endpoint != required_sender_endpoint);
                socket.close(error_code);
    #ifdef EXTRA_DETAIL
            std::cout << "Bytes received - " << bytes_received << std::endl;
    #endif 
            }
            catch(std::exception& e)
            {
                std::cout << error_code.message() << std::endl;
            }					
            return bytes_received;
        }

        /*
            This function is used to received a UDP packet

            params:
            * recv_buffer - the data buffer which is written to
            * size_of_buffer - specifies the size of the buffer which will receive the data, if the datagram is larger
            the extra data won't be stored
            * returns number of bytes in received, if it fails prints error code and returns 0.
            * If you need to filter payloads based on source endpoint, use recieve_from()

            return:
            number of bytes received, if it fails prints error code and returns 0.
        */
        std::size_t receive(byte* recv_buffer, std::size_t size_of_buffer)
        {
            boost::system::error_code ec;
            std::size_t bytes_received = 0;
            try
            {
                // opens a IPv4 socket then binds to it - uses local system ip and port specified in constructor
                socket.open(udp::v4(), ec);
                socket.bind(local_endpoint, ec);
                bytes_received = socket.receive(boost::asio::buffer(recv_buffer, size_of_buffer), 0, ec);
                socket.close(ec);
    #ifdef EXTRA_DETAIL
            std::cout << "Bytes received - " << bytes_received << std::endl;
    #endif 
            }
            catch (std::exception& e)
            {
                std::cout << ec.message() << std::endl;
            }
            return bytes_received;
        }

    };

    ///-------------------------------------------------------------------------------------------
    /*
        ---SERVER---

        This sends a datagram over ethernet to the endpoint specified
    */
    class server
    {
    private:
        boost::asio::io_service io_service;
        udp::socket socket;
        udp::endpoint remote_endpoint;
        
    public:
        // constructor
        server() :
            socket(io_service)
        {}

      /*
         sents the remote endpoint if the server class will send a large volume of data to a single endpoint

         params:
         * dest_ip - the ip of the destination endpoint
         * dest_port - port of the destination endpoint
      */
      void set_remote_endpoint(std::string dest_ip, short dest_port)
      {
         remote_endpoint = udp::endpoint(address::from_string(dest_ip), dest_port);
      }

        /*
         sends what is passed in the buffer to the endpoint specified in the set_remote_endpoint function

         params:
         * buffer - pointer to a buffer of data which is being sent
         * buffer_size - the amount of data in buffer to send

         return:
         * the number of bytes sent to the endpoint
      */
    std::size_t send(byte* buffer, std::size_t buffer_size)
    {
        boost::system::error_code error_code;
        std::size_t sent = 0;
        try
        {
            socket.open(udp::v4(), error_code);
            sent = socket.send_to(boost::asio::buffer(buffer, buffer_size),
            remote_endpoint, 0, error_code);
            socket.close();
#ifdef EXTRA_DETAIL
            std::cout << "Sent Payload --- " << sent << "\n";
#endif
        }
        catch (std::exception& e)
        {
            std::cout << error_code.message() << std::endl;
        }
        return sent;
    }

      /*
         sends what is passed in the buffer to the endpoint specified in remote_endpoint

         params:
         * buffer - pointer to a buffer of data which is being sent
         * buffer_size - the amount of data in buffer to send
         * remote_endpoint - the endpoint to send the data to

         return:
         * the number of bytes sent to the endpoint
      */
      std::size_t send_to(byte* buffer, std::size_t buffer_size, udp::endpoint remote_endpoint)
      {
         boost::system::error_code error_code;
         std::size_t sent = 0;
         try
         {
            socket.open(udp::v4(), error_code);
            sent = socket.send_to(boost::asio::buffer(buffer, buffer_size),
               remote_endpoint, 0, error_code);
            socket.close();
#ifdef EXTRA_DETAIL
            std::cout << "Sent Payload --- " << sent << "\n";
#endif
         }
         catch (std::exception& e)
         {
            std::cout << error_code.message() << std::endl;
         }
         return sent;
      }

      /*
         sends what is passed in the buffer to the endpoint specified in remote_endpoint allowing the 
         user to specify a specific port to send the data to

         params:
         * buffer - pointer to a buffer of data which is being sent
         * buffer_size - the amount of data in buffer to send
         * port - the port of the endpoint already specified

         return:
         * the number of bytes sent to the endpoint
      */
      std::size_t send_to_port(byte* buffer, std::size_t buffer_size, short port)
      {
         boost::system::error_code error_code;
         std::size_t sent = 0;
         udp::endpoint remote_endpoint_port_change(remote_endpoint.address(), port);
         try
         {
            socket.open(udp::v4(), error_code);
            sent = socket.send_to(boost::asio::buffer(buffer, buffer_size),
               remote_endpoint_port_change, 0, error_code);
            socket.close();
#ifdef EXTRA_DETAIL
            std::cout << "Sent Payload --- " << sent << "\n";
#endif
         }
         catch (std::exception & e)
         {
            std::cout << error_code.message() << std::endl;
         }
         return sent;
      }
	};
}


#endif // UDP_SERVER_CLIENT