// VelodyneCapture
//
// VelodyneCapture is the capture class to retrieve the laser data from Velodyne sensors using Boost.Asio and PCAP.
// VelodyneCapture will be able to retrieve lasers infomation about azimuth, vertical and distance that capture at one rotation.
// This class supports direct capture form Velodyne sensors, or capture from PCAP files.
// ( This class only supports VLP-16 and HDL-32E sensor, and not supports Dual Return mode. )
//
// If direct capture from sensors, VelodyneCapture are requires Boost.Asio and its dependent libraries ( Boost.System, Boost.Date_Time, Boost.Regex ).
// Please define HAVE_BOOST in preprocessor.
//
// If capture from PCAP files, VelodyneCapture are requires PCAP.
// Please define HAVE_PCAP in preprocessor.
//
// This source code is licensed under the MIT license. Please see the License in License.txt.
// Copyright (c) 2017 Tsukasa SUGIURA
// t.sugiura0204@gmail.com

#ifndef VELODYNE_CAPTURE
#define VELODYNE_CAPTURE

#include <string>
#include <sstream>
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>
#include <queue>
#include <vector>
#include <cassert>
#include <cstdint>
#include <chrono>
#include <iomanip>

#define HAVE_PCAP 1

#ifdef HAVE_PCAP
#include <pcap.h>
#endif

namespace velodyne
{
    struct Laser
    {
        double azimuth;
        double vertical;
        unsigned short distance;
        unsigned char intensity;
        unsigned char id;
        long long time;

        const bool operator < ( const struct Laser& laser ){
            if( azimuth == laser.azimuth ){
                return id < laser.id;
            }
            else{
                return azimuth < laser.azimuth;
            }
        }
    };

    class VelodyneCapture
    {
        protected:

            #ifdef HAVE_PCAP
            pcap_t* pcap = nullptr;
            std::string filename = "";
            #endif

            std::atomic_bool run = { false };

            std::vector<Laser> oneRingLasers;
            double last_azimuth;

            int MAX_NUM_LASERS;
            std::vector<double> lut;

            static const int LASER_PER_FIRING = 32;
            static const int FIRING_PER_PKT = 12;

            #pragma pack(push, 1)
            typedef struct LaserReturn
            {
                uint16_t distance;
                uint8_t intensity;
            } LaserReturn;
            #pragma pack(pop)

            struct FiringData
            {
                uint16_t blockIdentifier;
                uint16_t rotationalPosition;
                LaserReturn laserReturns[LASER_PER_FIRING];
            };

            struct DataPacket
            {
                FiringData firingData[FIRING_PER_PKT];
                uint32_t gpsTimestamp;
                uint8_t mode;
                uint8_t sensorType;
            };

        public:
            // Constructor
            VelodyneCapture()
            {
            }

            #ifdef HAVE_PCAP
            // Constructor ( capture from PCAP )
            VelodyneCapture( const std::string& filename )
            {
                open( filename );
            };
            #endif

            // Destructor
            ~VelodyneCapture()
            {
                close();
            }

            #ifdef HAVE_PCAP
            // Open Capture from PCAP
            const bool open( const std::string& filename )
            {
                // Open PCAP File
                char error[PCAP_ERRBUF_SIZE];
                pcap_t* pcap = pcap_open_offline( filename.c_str(), error );
                if( !pcap ){
                    throw std::runtime_error( error );
                    return false;
                }

                // Convert PCAP_NETMASK_UNKNOWN to 0xffffffff
                struct bpf_program filter;
                std::ostringstream oss;
                if( pcap_compile( pcap, &filter, oss.str().c_str(), 0, 0xffffffff ) == -1 ){
                    throw std::runtime_error( pcap_geterr( pcap ) );
                    return false;
                }

                if( pcap_setfilter( pcap, &filter ) == -1 ){
                    throw std::runtime_error( pcap_geterr( pcap ) );
                    return false;
                }

                this->pcap = pcap;
                this->filename = filename;
                this->last_azimuth = 0.0;

                return true;
            };
            #endif

            // Check Open
            const bool isOpen()
            {
                return (pcap != nullptr);
            }


            // Close Capture
            void close()
            {
                // Close PCAP
                if( pcap ){
                    pcap_close( pcap );
                    pcap = nullptr;
                    filename = "";
                }
            }

            // Operator Retrieve Capture Data with Sort
            void operator >> ( std::vector<Laser>& lasers )
            {
                // Retrieve Capture Data
                capturePCAP(lasers);
            }

        private:
            // Capture Thread from PCAP
            void capturePCAP(std::vector<Laser> &lasers)
            {
                run = true;

                lasers.clear();

                while( run ){
                    // Retrieve Header and Data from PCAP
                    struct pcap_pkthdr* header;
                    const unsigned char* data;
                    const int ret = pcap_next_ex( pcap, &header, &data );
                    if( ret <= 0 ){
                        break;
                    }

                    // Check Packet Data Size
                    // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
                    if( ( header->len - 42 ) != 1206 ){
                        continue;
                    }

                    // Retrieve Unix Time ( microseconds )
                    std::stringstream ss;
                    ss << header->ts.tv_sec << std::setw( 6 ) << std::left << std::setfill( '0' ) << header->ts.tv_usec;
                    const long long unixtime = std::stoll( ss.str() );

                    // Convert to DataPacket Structure ( Cut Header 42 bytes )
                    // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
                    const DataPacket* packet = reinterpret_cast<const DataPacket*>( data + 42 );
                    assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

                    // Caluculate Interpolated Azimuth
                    double interpolated = 0.0;
                    if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                        interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
                    }
                    else{
                        interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
                    }

                    // Processing Packet
                    for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                        // Retrieve Firing Data
                        const FiringData firing_data = packet->firingData[firing_index];
                        for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                            // Retrieve Rotation Azimuth
                            double azimuth = static_cast<double>( firing_data.rotationalPosition );

                            // Interpolate Rotation Azimuth
                            if( laser_index >= MAX_NUM_LASERS )
                            {
                                azimuth += interpolated;
                            }

                            // Reset Rotation Azimuth
                            if( azimuth >= 36000 )
                            {
                                azimuth -= 36000;
                            }

                            // Complete Retrieve Capture One Rotation Data
                            if( last_azimuth > azimuth ){
                                // Push One Rotation Data to Queue
                                lasers = oneRingLasers;
                                oneRingLasers.clear();
                                run = false;
                            }

                            Laser laser;
                            laser.azimuth = azimuth / 100.0;
                            laser.vertical = lut[laser_index % MAX_NUM_LASERS];
                            laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance;
                            laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                            laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );
                            laser.time = unixtime;

                            oneRingLasers.push_back( laser );

                            // Update Last Rotation Azimuth
                            last_azimuth = azimuth;
                        }
                    }
                }
            }
            #endif
    };

    class VLP16Capture : public VelodyneCapture
    {
        private:
            static const int MAX_NUM_LASERS = 16;
            const std::vector<double> lut = { -15.0, 1.0, -13.0, -3, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0 };

        public:
            VLP16Capture() : VelodyneCapture()
            {
                initialize();
            };

            #ifdef HAVE_BOOST
            VLP16Capture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : VelodyneCapture( address, port )
            {
                initialize();
            };
            #endif

            #ifdef HAVE_PCAP
            VLP16Capture( const std::string& filename ) : VelodyneCapture( filename )
            {
                initialize();
            };
            #endif

            ~VLP16Capture()
            {
            };

        private:
            void initialize()
            {
                VelodyneCapture::MAX_NUM_LASERS = MAX_NUM_LASERS;
                VelodyneCapture::lut = lut;
            };
    };

    class HDL32EPcapCapture : public VelodyneCapture
    {
        private:
            static const int MAX_NUM_LASERS = 32;
            const std::vector<double> lut = { -30.67, -9.3299999, -29.33, -8.0, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };

        public:
            HDL32EPcapCapture() : VelodyneCapture()
            {
                initialize();
            }

            #ifdef HAVE_PCAP
            HDL32EPcapCapture( const std::string& filename ) : VelodyneCapture( filename )
            {
                initialize();
            }
            #endif

            ~HDL32EPcapCapture()
            {
            }

        private:
            void initialize()
            {
                VelodyneCapture::MAX_NUM_LASERS = MAX_NUM_LASERS;
                VelodyneCapture::lut = lut;
            }
    };
}

