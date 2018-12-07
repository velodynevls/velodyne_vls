/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);

private:

  ///Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    double alpha;
    double time_offset;              ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  /* auto rpm detection members */
  uint32_t prev_packet_toh;
  uint16_t prev_packet_azm;
  uint32_t curr_packet_toh;
  uint16_t curr_packet_azm;
  double   auto_rpm;
  double   auto_alpha;
  uint32_t auto_npackets;
  double   auto_packet_rate;
  uint8_t  curr_packet_rmode; //    [strongest return or farthest mode => Singular Retruns per firing]  
                              // or [Both  => Dual Retruns per fire]
  uint8_t  curr_packet_sensor_model; // extract the sensor id from packet
  double   slot_time ;
  uint8_t  num_slots; 
  uint8_t  active_slots;
  double   firing_cycle; 
  std::string dump_file; // string to hold pcap file name
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
