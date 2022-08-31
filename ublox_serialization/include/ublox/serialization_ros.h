//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_SERIALIZATION_ROS_H
#define UBLOX_SERIALIZATION_ROS_H

#include "serialization.h"
#include "checksum.h"

#include <ros/serialization.h>
#include <ublox_msgs/NavRELPOSNED9.h>

namespace ublox {

template <typename T>
void Serializer<T>::read(const uint8_t *data, uint32_t count, 
                         typename boost::call_traits<T>::reference message) {
  ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
  ros::serialization::Serializer<T>::read(stream, message);
}

template <typename T>
uint32_t Serializer<T>::serializedLength(
    typename boost::call_traits<T>::param_type message) {
  return ros::serialization::Serializer<T>::serializedLength(message);
}

template <typename T>
void Serializer<T>::write(uint8_t *data, uint32_t size, 
                          typename boost::call_traits<T>::param_type message) {
  ros::serialization::OStream stream(data, size);
  ros::serialization::Serializer<T>::write(stream, message);
}

///
/// @brief Serializes the NavRELPOSNED9 message, which has a differing message definitions on
/// each end (additional Header on ROS-side which is not present on Ublox Receiver)
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavRELPOSNED9_<ContainerAllocator> > {
  typedef ublox_msgs::NavRELPOSNED9_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  /// @brief 
  /// As the GPS Module will produce a message w/o stamp, 
  /// this method differs from the actual message structure
  /// @param data 
  /// @param count 
  /// @param m 
  static void read(const uint8_t *data, uint32_t count, 
                   typename CallTraits::reference m) {
    ROS_INFO_ONCE("Using NavRELPOSNED9 serializer defined in ublox_serialization to read message");
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.refStationId);
    stream.next(m.iTOW);
    stream.next(m.relPosN);
    stream.next(m.relPosE);
    stream.next(m.relPosD);
    stream.next(m.relPosLength);
    stream.next(m.relPosHeading);
    stream.next(m.reserved2);
    stream.next(m.relPosHPN);
    stream.next(m.relPosHPE);
    stream.next(m.relPosHPD);
    stream.next(m.relPosHPLength);
    stream.next(m.accN);
    stream.next(m.accE);
    stream.next(m.accD);
    stream.next(m.accLength);
    stream.next(m.accHeading);
    stream.next(m.reserved3);
    stream.next(m.flags);
    // TODO: Add header
    m.header.stamp = ros::Time::now();
  }

  /// Used by the writer - Shouldn't need to worry as this message is only read
  static uint32_t serializedLength (typename CallTraits::param_type m) {
    ROS_INFO_ONCE("Using NavRELPOSNED9 serializer defined in ublox_serialization to get message length");
    ros::serialization::LStream stream;
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.refStationId);
    stream.next(m.iTOW);
    stream.next(m.relPosN);
    stream.next(m.relPosE);
    stream.next(m.relPosD);
    stream.next(m.relPosLength);
    stream.next(m.relPosHeading);
    stream.next(m.reserved2);
    stream.next(m.relPosHPN);
    stream.next(m.relPosHPE);
    stream.next(m.relPosHPD);
    stream.next(m.relPosHPLength);
    stream.next(m.accN);
    stream.next(m.accE);
    stream.next(m.accD);
    stream.next(m.accLength);
    stream.next(m.accHeading);
    stream.next(m.reserved3);
    stream.next(m.flags);
    return stream.getLength();
  }

  static void write(uint8_t *data, uint32_t size, 
                    typename CallTraits::param_type m) {
    ROS_INFO_ONCE("Using NavRELPOSNED9 serializer defined in ublox_serialization to write message");
    ros::serialization::OStream stream(data, size);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.refStationId);
    stream.next(m.iTOW);
    stream.next(m.relPosN);
    stream.next(m.relPosE);
    stream.next(m.relPosD);
    stream.next(m.relPosLength);
    stream.next(m.relPosHeading);
    stream.next(m.reserved2);
    stream.next(m.relPosHPN);
    stream.next(m.relPosHPE);
    stream.next(m.relPosHPD);
    stream.next(m.relPosHPLength);
    stream.next(m.accN);
    stream.next(m.accE);
    stream.next(m.accD);
    stream.next(m.accLength);
    stream.next(m.accHeading);
    stream.next(m.reserved3);
    stream.next(m.flags);
  }
};

} // namespace ublox

#endif // UBLOX_SERIALIZATION_ROS_H
