/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include "dynamixel_sdk/packet_handler.h"
#include "dynamixel_sdk/protocol1_packet_handler.h"
#include "dynamixel_sdk/protocol2_packet_handler.h"

using namespace dynamixel;

PacketHandler *PacketHandler::getPacketHandler(float protocol_version)
{
  if (protocol_version == 1.0)
  {
    return (PacketHandler *)(Protocol1PacketHandler::getInstance());
  }
  else if (protocol_version == 2.0)
  {
    return (PacketHandler *)(Protocol2PacketHandler::getInstance());
  }

  return (PacketHandler *)(Protocol2PacketHandler::getInstance());
}
