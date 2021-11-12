// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples bulk_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /bulk_set_item dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'position', item2: 'LED', value1: 1000, value2: 1}"
 * $ rostopic pub -1 /run dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'LED', item2: 'position', value1: 1, value2: 1000}"
 * $ rosservice call /bulk_get_item "{id1: 1, id2: 2, item1: 'position', item2: 'LED'}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_LED      65
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_POSITION    116

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting

#define DXL2_ID               2              
#define DXL3_ID               3
#define DXL5_ID               5              
#define DXL6_ID               6
#define DXL8_ID               8              
#define DXL9_ID               9 
#define DXL11_ID              11             
#define DXL12_ID              12             
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

bool bulkGetItemCallback(
  dynamixel_sdk_examples::BulkGetItem::Request & req,
  dynamixel_sdk_examples::BulkGetItem::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position1 = 0;
  int32_t position2 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  if (req.item1 == "position") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
  } else if (req.item1 == "LED") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id1, ADDR_PRESENT_LED, 1);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", req.id1);
    return 0;
  }

  if (req.item2 == "position") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
  } else if (req.item2 == "LED") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id2, ADDR_PRESENT_LED, 1);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID %d", req.id2);
    return 0;
  }

  uint32_t value1 = 0;
  uint32_t value2 = 0;
  dxl_comm_result = groupBulkRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    if (req.item1 == "position") {
      value1 = groupBulkRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    } else if (req.item2 == "LED") {
      value1 = groupBulkRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    }

    if (req.item1 == "position") {
      value2 = groupBulkRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    } else if (req.item2 == "LED") {
      value2 = groupBulkRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    }

    ROS_INFO("getItem : [ID:%d] [%s: %d]", req.id1, req.item1.c_str(), value1);
    ROS_INFO("getItem : [ID:%d] [%s: %d]", req.id2, req.item2.c_str(), value2);
    res.value1 = value1;
    res.value2 = value2;
    groupBulkRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    groupBulkRead.clearParam();
    return false;
  }
}

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
  
    // Storing start time
    clock_t start_time = clock();
  
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}     

void running(const dynamixel_sdk_examples::BulkSetItem::ConstPtr & msg)
{
  for (int i = 1; i<20; i++)
  {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t addr_goal_item[0];
  uint8_t len_goal_item[0];

  /*
  uint8_t param_goal_position5[4];
  uint8_t param_goal_position6[4];
  uint8_t param_goal_position8[4];
  uint8_t param_goal_position9[4];
  uint8_t param_goal_position11[4];
  uint8_t param_goal_position12[4];


  //int value2 = 3500;
  //int value3 =
  int value5 =500;
  int value6 =1000;
  int value8 =3500;
  int value9 =3000;
  int value11 =1400;
  int value12 =2000;

  int id2 = 2;
  int id3 = 3;
  int id5 = 5;
  int id6 = 6;
  int id8 = 8;
  int id9 = 9;
  int id11 = 11;
  int id12 = 12;
  */
  for (int x = 0; x<9; x++)
  {
  uint8_t param_goal_position[4];
  int value_list[8] = {2500,2000,500,1000,3500,3000,1400,2000};
  int id_list[8] = {2,3,5,6,8,9,11,12};
  int from[8] = {-1,-1,1,1,-1,-1,-1,-1};
  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)((value_list[x])+(from[x]*(1000*(i%2)))); // Convert int32 -> uint32
  //ROS_INFO("Running : %d ",((value_list[x])+(from[x]*(1000*(i%2)))));
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position));
  addr_goal_item[0] = ADDR_GOAL_POSITION;
  len_goal_item[0] = 4;
  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id_list[x], addr_goal_item[0], len_goal_item[0], param_goal_position);
  }
  /*
  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    uint32_t position5 = (unsigned int)((value5)+(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position5[0] = DXL_LOBYTE(DXL_LOWORD(position5));
    param_goal_position5[1] = DXL_HIBYTE(DXL_LOWORD(position5));
    param_goal_position5[2] = DXL_LOBYTE(DXL_HIWORD(position5));
    param_goal_position5[3] = DXL_HIBYTE(DXL_HIWORD(position5));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;
  
    uint32_t position6 = (unsigned int)((value6)+(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position6[0] = DXL_LOBYTE(DXL_LOWORD(position6));
    param_goal_position6[1] = DXL_HIBYTE(DXL_LOWORD(position6));
    param_goal_position6[2] = DXL_LOBYTE(DXL_HIWORD(position6));
    param_goal_position6[3] = DXL_HIBYTE(DXL_HIWORD(position6));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;

    uint32_t position8 = (unsigned int)((value8)-(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position8[0] = DXL_LOBYTE(DXL_LOWORD(position8));
    param_goal_position8[1] = DXL_HIBYTE(DXL_LOWORD(position8));
    param_goal_position8[2] = DXL_LOBYTE(DXL_HIWORD(position8));
    param_goal_position8[3] = DXL_HIBYTE(DXL_HIWORD(position8));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;


    uint32_t position9 = (unsigned int)((value9)-(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position9[0] = DXL_LOBYTE(DXL_LOWORD(position9));
    param_goal_position9[1] = DXL_HIBYTE(DXL_LOWORD(position9));
    param_goal_position9[2] = DXL_LOBYTE(DXL_HIWORD(position9));
    param_goal_position9[3] = DXL_HIBYTE(DXL_HIWORD(position9));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;


    uint32_t position11 = (unsigned int)((value11)-(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position11[0] = DXL_LOBYTE(DXL_LOWORD(position11));
    param_goal_position11[1] = DXL_HIBYTE(DXL_LOWORD(position11));
    param_goal_position11[2] = DXL_LOBYTE(DXL_HIWORD(position11));
    param_goal_position11[3] = DXL_HIBYTE(DXL_HIWORD(position11));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;


    uint32_t position12 = (unsigned int)((value12)-(1000*(i%2))); // Convert int32 -> uint32
    param_goal_position12[0] = DXL_LOBYTE(DXL_LOWORD(position12));
    param_goal_position12[1] = DXL_HIBYTE(DXL_LOWORD(position12));
    param_goal_position12[2] = DXL_LOBYTE(DXL_HIWORD(position12));
    param_goal_position12[3] = DXL_HIBYTE(DXL_HIWORD(position12));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id5, addr_goal_item[0], len_goal_item[0], param_goal_position5);

  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id6, addr_goal_item[0], len_goal_item[0], param_goal_position6);

  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id8, addr_goal_item[0], len_goal_item[0], param_goal_position8);

  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id9, addr_goal_item[0], len_goal_item[0], param_goal_position9);

  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id11, addr_goal_item[0], len_goal_item[0], param_goal_position11);

  dxl_addparam_result = groupBulkWrite.addParam((uint8_t) id12, addr_goal_item[0], len_goal_item[0], param_goal_position12);
  */

  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Running : %d ",i);
  } else {
    ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupBulkWrite.clearParam();
  delay(100);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL3_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL5_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL6_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL8_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL8_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL9_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL9_ID);
    return -1;
  }
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL11_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL11_ID);
    return -1;
  }
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL12_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL12_ID);
    return -1;
  }

  ros::init(argc, argv, "run_fix_position");
  ros::NodeHandle nh;
  //ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/bulk_get_item", bulkGetItemCallback);
  ros::Subscriber bulk_set_item_sub = nh.subscribe("/run", 10, running);
  ros::spin();

  portHandler->closePort();
  return 0;
}
