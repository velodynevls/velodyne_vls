/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing Group
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

namespace velodyne_driver
{
  static double prev_frac_packet = 0;
  inline   std::string toBinary(int n)
  {
        std::string r;
        while(n!=0) {r=(n%2==0 ?"0":"1")+r; n/=2;}
        while (r.length() != 8){
          r = '0' + r;
        }
        return r;
  }

  inline   double convertBinaryToDecimal(std::string binaryString)
  {
      double value = 0;
      int indexCounter = 0;
      for(int i=binaryString.length()-1;i>=0;i--){

          if(binaryString[i]=='1'){
              value += pow(2, indexCounter);
          }
          indexCounter++;
      }
      return value;
  }

  inline   double computeTimeStamp(velodyne_msgs::VelodyneScanPtr scan, int index){

      std::string digit4 = toBinary(scan->packets[index].data[1203]);
      std::string digit3 = toBinary(scan->packets[index].data[1202]);
      std::string digit2 = toBinary(scan->packets[index].data[1201]);
      std::string digit1 = toBinary(scan->packets[index].data[1200]);
      std::string digit = digit4 + digit3 + digit2 + digit1; // string concatenation
      double value = convertBinaryToDecimal(digit);
      // compute the seconds from the beginning of that hour to when the data being captured
      double time_stamp = (double)value / 1000000;
      return time_stamp;
  }

/** Utility function for Velodyne Driver
 *  gets the number of laser beams fired concurrently 
 *  for different sensor models 
*/

inline int get_concurrent_beams(uint8_t sensor_model)
{
/*
Strongest 0x37 (55)   HDL-32E 0x21 (33)
Last Return 0x38 (56) VLP-16 0x22 (34)
Dual Return 0x39 (57) Puck LITE 0x22 (34)
         -- --        Puck Hi-Res 0x24 (36)
         -- --        VLP-32C 0x28 (40)
         -- --        Velarray 0x31 (49)
         -- --        VLS-128 0xA1 (161)
*/

  switch(sensor_model)
  {
    case 33:
        return(2); // hdl32e
    case 34:
        return(1); // vlp16 puck lite
    case 36:
        return(1); // puck hires  (same as vlp16 ?? need to check)
    case 40:
        return(2); // vlp32c
    case 49:
        return(2); // velarray
    case 161:
        return(8); // vls128
    case 99:
        return(8); // vls128
    default:
        ROS_WARN_STREAM("[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2 simultaneous firings");
        return(2); // hdl-64e

  }
}

/** Utility function for Velodyne Driver
 *  gets the number of packet multiplier for dual return mode vs 
 *  single return mode 
*/

inline int get_rmode_multiplier(uint8_t sensor_model, uint8_t packet_rmode)
{
 /*
    HDL64E 2
    VLP32C 2
    HDL32E 2
    VLS128 3
    VLSP16 2
*/
  if(packet_rmode  == 57)
  {
    switch(sensor_model)
    {
      case 33:
          return(2); // hdl32e
      case 34:
          return(2); // vlp16 puck lite
      case 36:
          return(2); // puck hires 
      case 40:
          return(2); // vlp32c
      case 49:
          return(2); // velarray
      case 161:
          return(3); // vls128
      case 99:
          return(3); // vls128
      default:
          ROS_WARN_STREAM("[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2x number of packekts for Dual return");
          return(2); // hdl-64e
    }
   }
   else
   {
     return(1);
   }
}

/** Utility function for the Velodyne driver 
 *
 *  provides a estimated value for number of packets in 
 *  1 full scan at current operating rpm estimate of the sensor 
 *  This value is used by the poll() routine to assemble 1 scan from 
 *  required number of packets 
 *  @returns number of packets in full scan 
 */

inline int get_auto_npackets(uint8_t sensor_model, uint8_t packet_rmode, double auto_rpm, double firing_cycle, int active_slots) 
{
  double rps = auto_rpm / 60.0; 
  double time_for_360_degree_scan = 1.0/rps;
  double total_number_of_firing_cycles_per_full_scan = time_for_360_degree_scan / firing_cycle;
  double total_number_of_firings_per_full_scan =  total_number_of_firing_cycles_per_full_scan 
                                                * get_concurrent_beams(sensor_model); 
  double total_number_of_points_captured_for_single_return = active_slots * total_number_of_firings_per_full_scan;
  double total_number_of_packets_per_full_scan = total_number_of_points_captured_for_single_return / 384;
  double total_number_of_packets_per_second = total_number_of_packets_per_full_scan / time_for_360_degree_scan;
  double auto_npackets = get_rmode_multiplier(sensor_model,packet_rmode) * floor((total_number_of_packets_per_full_scan+prev_frac_packet));
  prev_frac_packet = get_rmode_multiplier(sensor_model,packet_rmode) * (total_number_of_packets_per_full_scan + prev_frac_packet) - auto_npackets ;
  return(auto_npackets);
}

/** Utility function for the Velodyne driver 
 *
 *  provides a estimated value for number of packets in 
 *  1 second at current operating rpm estimate of the sensor 
 *  This value is used by the pcap reader (InputPCAP class ) 
 *  to pace the speed of packet reading.
 *  @returns number of packets per second 
 */

inline double get_auto_packetrate(uint8_t sensor_model, uint8_t packet_rmode, double auto_rpm, double firing_cycle, int active_slots) 
{
  double rps = auto_rpm / 60.0; 
  double time_for_360_degree_scan = 1.0/rps;
  double total_number_of_firing_cycles_per_full_scan = time_for_360_degree_scan / firing_cycle;
  double total_number_of_firings_per_full_scan =  total_number_of_firing_cycles_per_full_scan 
                                                * get_concurrent_beams(sensor_model); 
  double total_number_of_points_captured_for_single_return = active_slots * total_number_of_firings_per_full_scan;
  double total_number_of_packets_per_full_scan = total_number_of_points_captured_for_single_return / 384;
  double total_number_of_packets_per_second = total_number_of_packets_per_full_scan / time_for_360_degree_scan;
  return((get_rmode_multiplier(sensor_model,packet_rmode)*total_number_of_packets_per_second));
}
/** Constructor for the Velodyne driver 
 *
 *  provides a binding to ROS node for processing and 
 *  configuration 
 *  @returns handle to driver object
 */

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") ||
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
      slot_time = 1.2e-6; // basic slot time
      num_slots = 116;                     // number of active + maintenence slots
      active_slots = 32;                  // number of active slots
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
      slot_time = 1.2e-6; // basic slot time
      num_slots = 116;                     // number of slots
      active_slots = 32;                  // number of active slots
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
      slot_time = 1.152e-6; // basic slot time
      num_slots = 40;                     // number of slots
      active_slots = 32;                  // number of active slots
    }
 else if (config_.model == "VLP32C")
    {
      packet_rate = 3014; // 12 groups of 32 firings where a pair of 2 firings corresponds to 55.296us -> 1/(12*55.296us)
      model_full_name = "VLP-32C";
      slot_time = 2.304e-6; // basic slot time
      num_slots = 24;                     // number of slots
      active_slots = 16;                  // number of active slots
    }
 else if (config_.model == "VLS128")
    {
      packet_rate = 12507; // 3 groups of 128 firings where a set of 8 firings corresponds to 55.296us -> 1/(12*55.296us) 
      model_full_name = "VLS-128";
      slot_time = 2.665e-6;              // basic slot time
      num_slots = 20;                    // number of slots
      active_slots = 16;                 // number of active slots
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 1507;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
 //     slot_time = 2.304e-6; // basic slot time
      slot_time = 2.304e-6; // basic slot time
      num_slots = 24;                     // number of slots
      active_slots = 16;                  // number of active slots
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
      slot_time = 1.2e-6;                  // basic slot time
      num_slots = 116;                     // number of slots
      active_slots = 32;                   // number of active slots
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  private_nh.getParam("rpm", config_.rpm);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate
  auto_rpm = config_.rpm;

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  int npackets = (int) ceil(packet_rate / frequency);
  private_nh.param("npackets", config_.npackets, npackets);
  private_nh.getParam("npackets", config_.npackets);
  private_nh.setParam("npackets", npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");
  private_nh.param("auto_rpm_alpha",auto_alpha,0.999);
  private_nh.getParam("auto_rpm_alpha", auto_alpha);
  ROS_INFO_STREAM("Automatic RPM smoothing coeff " << auto_alpha  << " (1 means no tracking, zero means no smoothing) ");

  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
  firing_cycle = slot_time * num_slots; // firing cycle time
}

/** poll the device
 *
 * poll is used by nodelet to bind to the ROS thread.
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < config_.npackets; ++i)
  {
      while (true)
      {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
          if (rc == 1) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
          if (rc == 0) continue; // timeout?
      }
      // Automatic RPM detection logic pushed here.
      // got a packet here 
      // Build the detection state machine to update config_.npackets automatically 
      // after observing the first few hundred  packets  
      curr_packet_toh  = scan->packets[i].data[1200];
      curr_packet_toh |= scan->packets[i].data[1201] << 8;
      curr_packet_toh |= scan->packets[i].data[1202] << 16;
      curr_packet_toh |= scan->packets[i].data[1203] << 24;
      curr_packet_azm  = scan->packets[i].data[2]; // lower word of azimuth block 0
      curr_packet_azm |= scan->packets[i].data[3] << 8; // higher word of azimuth block 0
      curr_packet_rmode = scan->packets[i].data[1204];
      curr_packet_sensor_model = scan->packets[i].data[1205];
      if(i > 0 ) 
      {
          int  delta_azm = ((curr_packet_azm + 36000) - prev_packet_azm) % 36000; 
          long  delta_toh = ((curr_packet_toh + 3600000000) - prev_packet_toh) % 3600000000; 
          double inst_azm_rate = double(delta_azm)*1e4 / double(delta_toh); // 1 step diff
          auto_rpm  = auto_alpha*auto_rpm + (1.0 - auto_alpha) * (inst_azm_rate/ 6) ; // 6 is basically ratio of 360 deg for 1 revolution 
                                                                                      //  and 60 seconds in minute
                                                                                      // auto_rpm works as a 1 tap IIR with auto_alpha as
                                                                                      // a coeff of memory.
                                                                                      // we can trade tracking bandwidth with variance of 
                                                                                      // auto_rpm by auto_alpha.. value of 0 will kill tracking
                                                                                      // and have zero variance in auto_rpm
          
          // std::cerr << "delta_azm = " << delta_azm ;
          // std::cerr << ", delta_toh = " << delta_toh ;
          // std::cerr << ", rate = " << inst_azm_rate ;
          // std::cerr << ", auto_rpm = " << auto_rpm ;
          // std::cerr <<  std::endl;
         
      }
      prev_packet_toh = curr_packet_toh;
      prev_packet_azm = curr_packet_azm;
  }
  // calculate npackets for next scan
  auto_npackets =  get_auto_npackets(curr_packet_sensor_model, curr_packet_rmode, auto_rpm,firing_cycle,active_slots); 
  
  // average the time stamp from first package and last package
  double firstTimeStamp = computeTimeStamp(scan, 0);
  double lastTimeStamp = computeTimeStamp(scan, config_.npackets - 1);
  double meanTimeStamp = (firstTimeStamp + lastTimeStamp)/2;
  // std::cerr << " Velodyne Driver Timestamp first packet= " << firstTimeStamp << std::endl;
  // std::cerr << " Velodyne Driver Timestamp last packet= " << lastTimeStamp << std::endl;
  time_t seconds;
  seconds = time (NULL);
  int gpsSeconds = ((int)(seconds/3600)) * 3600 + floor(meanTimeStamp);
  int nanSecs =  (meanTimeStamp - floor(meanTimeStamp)) * pow(10,9);
  scan->header.stamp = ros::Time(gpsSeconds, nanSecs);
  // std::cerr<< scan->header.stamp << std::endl;
  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);
  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();
  // update npackets for next run
  // std::cerr << ", auto_rpm = " << auto_rpm ;
  // std::cerr << ", auto_npackets = " << auto_npackets;
  // std::cerr << ", prev_frac_packet = " << prev_frac_packet << std::endl ;
         
  config_.npackets = auto_npackets; 
  if (dump_file != "")                  // have PCAP file?
  {
    auto_packet_rate = get_auto_packetrate(curr_packet_sensor_model, curr_packet_rmode, auto_rpm,firing_cycle,active_slots);
    input_->setPacketRate(auto_packet_rate);
  }
  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
  auto_alpha = config.auto_rpm_alpha;
}

} // namespace velodyne_driver
