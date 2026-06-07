// /*
//  * Copyright (C) 2016 - 2019 Xilinx, Inc.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are
//  met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  notice,
//  *    this list of conditions and the following disclaimer in the
//  documentation
//  *    and/or other materials provided with the distribution.
//  * 3. The name of the author may not be used to endorse or promote products
//  *    derived from this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
//  IMPLIED
//  * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
//  * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
//  TO,
//  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS;
//  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//  * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  */

#include "communication.hpp"
#include "interface_communication.hpp"
#include "task.h"
#include "xil_printf.h"

// dto
#include "dto/act_data.h"
#include "dto/act_info.h"
#include "dto/comm_big_data_test.h"
#include "dto/comm_reliable_test.h"
#include "dto/common_parameter.h"
#include "dto/feature_inspect_command.h"
#include "dto/feature_inspect_finish_command.h"
#include "dto/feature_inspect_finish_res.h"
#include "dto/feature_inspect_ready_command.h"
#include "dto/feature_inspect_ready_res.h"
#include "dto/full_system_info.h"
#include "dto/guidance_command.h"
#include "dto/ins_data.h"
#include "dto/ins_info.h"
#include "dto/ins_main_data.h"
#include "dto/missile_target_state.h"
#include "dto/navigation_test_command.h"
#include "dto/one_test_result.h"
#include "dto/fin_control_main_command.h"
#include "dto/fin_control_req.h"
#include "dto/fin_control_res.h"
#include "dto/position_req.h"
#include "dto/position_res.h"
#include "dto/reboot_command.h"
#include "dto/reboot_res.h"
#include "dto/skr_dist.h"
#include "dto/skr_info.h"
#include "dto/skr_main_data.h"
#include "dto/target_distance_req.h"
#include "dto/target_distance_res.h"

// enum
#include "enum/device_id.h"
#include "enum/icd_id.h"
#include "enum/subsystem.h"

// fsm
#include "fsm/act_status.h"
#include "fsm/gcu_status.h"
#include "fsm/ins_status.h"


Network::ICommunication *i_communication = nullptr;

void status_task_act_test(void *pvParameters) {

  FinControlMainCommand fin_control_main_command = {
      .message_id = IcdId::FIN_CONTROL_MAIN_COMMAND,
      .a_motor_theta = 0,
      .b_motor_theta = 0
  };
  int32_t step = 1 * 1000;        // 1도 (단위: 0.001°)
  const int32_t MIN_ANGLE = -90 * 1000;
  const int32_t MAX_ANGLE = 90 * 1000;
  while (1) {

      // 전송 로그
    xil_printf("[ACT] Send a_motor=%d.%03d deg, b_motor=%d.%03d deg\r\n",fin_control_main_command.a_motor_theta / 1000,(fin_control_main_command.a_motor_theta < 0 ?-fin_control_main_command.a_motor_theta : fin_control_main_command.a_motor_theta) % 1000,fin_control_main_command.b_motor_theta / 1000,(fin_control_main_command.b_motor_theta < 0 ?-fin_control_main_command.b_motor_theta : fin_control_main_command.b_motor_theta) % 1000);

    // 전송
    i_communication->send_dto(TypeFlag::NONE,DeviceId::ACT, (uint8_t *)&fin_control_main_command, sizeof(FinControlMainCommand));
    // 다음 각도 계산
    fin_control_main_command.a_motor_theta += step;
    fin_control_main_command.b_motor_theta += step;
    // 경계 도달 시 방향 반전
    if (fin_control_main_command.a_motor_theta >= MAX_ANGLE) {
      fin_control_main_command.a_motor_theta = MAX_ANGLE;
      fin_control_main_command.b_motor_theta = MAX_ANGLE;
      step = -1 * 1000;
    } else if (fin_control_main_command.a_motor_theta <= MIN_ANGLE) {
      fin_control_main_command.a_motor_theta = MIN_ANGLE;
      fin_control_main_command.b_motor_theta = MIN_ANGLE;
      step = 1 * 1000;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void status_task_test1(void *pvParameters) {
  CommReliableTest comm_reliable_test = {.message_id = IcdId::COMM_RELIABLE_TEST, .counter = 0};
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Send Task Running...\r\n");
    i_communication->send_dto(TypeFlag::SECURE,DeviceId::TEST2, (uint8_t *)&comm_reliable_test, sizeof(CommReliableTest));
    comm_reliable_test.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
  }
}

void status_task_test2(void *pvParameters) {
  CommReliableTest comm_reliable_test = {.message_id = IcdId::COMM_RELIABLE_TEST, .counter = 0};
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Send Task Running...\r\n");
    i_communication->send_dto(TypeFlag::RELIABLE,DeviceId::TEST1, (uint8_t *)&comm_reliable_test, sizeof(CommReliableTest));
    comm_reliable_test.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
  }
}

void send_task_for_ui(void *pvParameters) {
  CommReliableTest comm_reliable_test = {.message_id = IcdId::COMM_RELIABLE_TEST, .counter = 0};
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Send Task Running...\r\n");
    i_communication->send_dto(TypeFlag::NONE,DeviceId::UI, (uint8_t *)&comm_reliable_test, sizeof(CommReliableTest));
    comm_reliable_test.counter++;
    i_communication->send_dto(TypeFlag::NONE,DeviceId::TEST2, (uint8_t *)&comm_reliable_test, sizeof(CommReliableTest));
    comm_reliable_test.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
  }
}

void test_receive_callback(IcdId id, uint8_t *data, size_t len) {
  if (id == IcdId::COMM_BIG_DATA_TEST) {
    CommBigDataTest *dto = reinterpret_cast<CommBigDataTest *>(data);
    xil_printf("Received BIG_DATA: ICD=%04X, len=%d, counter=%u, big_data_size=%u\r\n",static_cast<uint16_t>(id), len, dto->counter, dto->big_data_size);
  } 
  else if(id==IcdId::COMM_RELIABLE_TEST){
    CommReliableTest *cmd = reinterpret_cast<CommReliableTest*>(data);
    xil_printf("Received data with ICD ID:%04X, length:%d, COMM_RELIABLE_TEST received: counter=%d\r\n", static_cast<uint16_t>(id), len , cmd->counter);
  }
  else {
    xil_printf("[ERROR] Unknown ICD ID received: %04X\r\n", static_cast<uint16_t>(id));
  }
}

void big_data_send_task1(void *pvParameters) {
  //xil_printf("[DEBUG] sizeof(CommBigDataTest)=%d\r\n", sizeof(CommBigDataTest));
  static CommBigDataTest dto;
  dto.message_id = IcdId::COMM_BIG_DATA_TEST;
  dto.counter = 0;

  const char *base =
    "The quick brown fox jumps over the lazy dog. "
    "In the beginning, there was nothing, and then the universe exploded into existence "
    "with a spectacular burst of energy and light that spread across the infinite cosmos. "
    "Scientists have long studied the origins of life on Earth, tracing back billions of years "
    "to the first single-celled organisms that emerged from the primordial soup of ancient oceans. "
    "Throughout history, humanity has sought to understand the fundamental nature of reality, "
    "pushing the boundaries of knowledge through mathematics, philosophy, and empirical observation. "
    "The development of modern technology has accelerated at an unprecedented pace, transforming "
    "every aspect of human civilization from communication and transportation to medicine and warfare. ";

  uint32_t base_len = strlen(base);
  uint32_t target_size = 180 * 320 * 3;  // 178200 bytes (180x320*3 RGB 이미지)
  uint32_t filled = 0;

  while (filled < target_size) {
    uint32_t copy_len = (target_size - filled > base_len) ? base_len : (target_size - filled);
    memcpy(dto.big_data + filled, base, copy_len);
    filled += copy_len;
  }
  dto.big_data_size = target_size;
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Sending BIG_DATA: counter=%u, size=%u\r\n", dto.counter, dto.big_data_size);
    i_communication->send_dto(TypeFlag::RELIABLE,DeviceId::TEST2, reinterpret_cast<uint8_t *>(&dto), sizeof(CommBigDataTest));
    dto.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
  }
}

void big_data_send_task2(void *pvParameters) {
  //xil_printf("[DEBUG] sizeof(CommBigDataTest)=%d\r\n", sizeof(CommBigDataTest));
  static CommBigDataTest dto;
  dto.message_id = IcdId::COMM_BIG_DATA_TEST;
  dto.counter = 0;

  const char *base =
    "The quick brown fox jumps over the lazy dog. "
    "In the beginning, there was nothing, and then the universe exploded into existence "
    "with a spectacular burst of energy and light that spread across the infinite cosmos. "
    "Scientists have long studied the origins of life on Earth, tracing back billions of years "
    "to the first single-celled organisms that emerged from the primordial soup of ancient oceans. "
    "Throughout history, humanity has sought to understand the fundamental nature of reality, "
    "pushing the boundaries of knowledge through mathematics, philosophy, and empirical observation. "
    "The development of modern technology has accelerated at an unprecedented pace, transforming "
    "every aspect of human civilization from communication and transportation to medicine and warfare. ";

  uint32_t base_len = strlen(base);
  uint32_t target_size = 180 * 320 * 3;  // 178200 bytes (180x320*3 RGB 이미지)
  uint32_t filled = 0;

  while (filled < target_size) {
    uint32_t copy_len = (target_size - filled > base_len) ? base_len : (target_size - filled);
    memcpy(dto.big_data + filled, base, copy_len);
    filled += copy_len;
  }
  dto.big_data_size = target_size;
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Sending BIG_DATA: counter=%u, size=%u\r\n", dto.counter, dto.big_data_size);
    i_communication->send_dto(TypeFlag::NONE,DeviceId::TEST1, reinterpret_cast<uint8_t *>(&dto), sizeof(CommBigDataTest));
    dto.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
  }
}

void big_data_send_for_ui(void *pvParameters) {
  //xil_printf("[DEBUG] sizeof(CommBigDataTest)=%d\r\n", sizeof(CommBigDataTest));
  static CommBigDataTest dto;
  dto.message_id = IcdId::COMM_BIG_DATA_TEST;
  dto.counter = 0;

  const char *base =
    "The quick brown fox jumps over the lazy dog. "
    "In the beginning, there was nothing, and then the universe exploded into existence "
    "with a spectacular burst of energy and light that spread across the infinite cosmos. "
    "Scientists have long studied the origins of life on Earth, tracing back billions of years "
    "to the first single-celled organisms that emerged from the primordial soup of ancient oceans. "
    "Throughout history, humanity has sought to understand the fundamental nature of reality, "
    "pushing the boundaries of knowledge through mathematics, philosophy, and empirical observation. "
    "The development of modern technology has accelerated at an unprecedented pace, transforming "
    "every aspect of human civilization from communication and transportation to medicine and warfare. ";

  uint32_t base_len = strlen(base);
  uint32_t target_size = 180 * 320 * 3;  // 178200 bytes (180x320*3 RGB 이미지)
  uint32_t filled = 0;

  while (filled < target_size) {
    uint32_t copy_len = (target_size - filled > base_len) ? base_len : (target_size - filled);
    memcpy(dto.big_data + filled, base, copy_len);
    filled += copy_len;
  }
  dto.big_data_size = target_size;
  TickType_t last = xTaskGetTickCount();
  while (1) {
    //xil_printf("Sending BIG_DATA: counter=%u, size=%u\r\n", dto.counter, dto.big_data_size);
    i_communication->send_dto(TypeFlag::NONE, DeviceId::UI, reinterpret_cast<uint8_t *>(&dto), sizeof(CommBigDataTest));
    dto.counter++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}


int main(void) {
  static Network::Communication communication;
  i_communication = &communication;
  i_communication->init(DeviceId::TEST1);
  //i_communication->init(DeviceId::TEST2);
  i_communication->register_callback((receive_callback_t)test_receive_callback);
  //xTaskCreate(send_task_for_ui, (const char *)"send_task_for_ui", 1024,NULL, tskIDLE_PRIORITY+2, NULL);
  //xTaskCreate(status_task_test1, (const char *)"status_task_test1", 1024,NULL, tskIDLE_PRIORITY, NULL);
  //xTaskCreate(status_task_test2, (const char *)"status_task_test2", 1024,NULL, tskIDLE_PRIORITY, NULL);
  //xTaskCreate(status_task_act_test,(const char *)"status_task_act_test", 1024,NULL, tskIDLE_PRIORITY, NULL);
  //xTaskCreate(big_data_send_for_ui, (const char *)"big_data_send_for_ui", 2048, NULL, tskIDLE_PRIORITY+1, NULL);
  //xTaskCreate(big_data_send_task1, (const char *)"big_data_send_task1", 2048, NULL, tskIDLE_PRIORITY, NULL);
  //xTaskCreate(big_data_send_task2, (const char *)"big_data_send_task2", 2048, NULL, tskIDLE_PRIORITY, NULL);
  vTaskStartScheduler();
}
