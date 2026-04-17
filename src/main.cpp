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
#include "dto/common_parameter.h"
#include "dto/feature_inspect_ready_command.h"
#include "dto/feature_inspect_finish_command.h"
#include "enum/device_id.h"
#include "enum/icd_id.h"
#include "enum/subsystem.h"
#include "interface_communication.hpp"
#include "task.h"
#include "xil_printf.h"

void receive_callback(IcdId id, uint8_t *data, size_t len) {
  xil_printf("Received data with ICD ID: %d, length: %d\r\n",
             static_cast<int>(id), len);
}

Network::ICommunication *i_communication = nullptr;

void status_task(void *pvParameters) {
	//FeatureInspectReadyCommand feature_inspect_ready_command = {.message_id = IcdId::FEATURE_INSPECT_READY_COMMAND};
	FeatureInspectFinishCommand feature_inspect_finish_command = {.message_id = IcdId::FEATURE_INSPECT_FINISH_COMMAND};
  while (1) {
    xil_printf("Status Task Running...\r\n");
    //i_communication->send_dto_reliable(DeviceId::TEST2, (uint8_t *)&feature_inspect_ready_command, sizeof(FeatureInspectReadyCommand));
    i_communication->send_dto_reliable(DeviceId::TEST1, (uint8_t *)&feature_inspect_finish_command, sizeof(FeatureInspectFinishCommand));
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1초마다 상태 출력
  }
}

int main(void) {
  Network::Communication communication;
  i_communication = &communication;
  //i_communication->object_init(DeviceId::TEST1);
  i_communication->object_init(DeviceId::TEST2);
  i_communication->register_callback((receive_callback_t)receive_callback);
  xTaskCreate(status_task, (const char *)"status_task", 1024,NULL, tskIDLE_PRIORITY, NULL);
  vTaskStartScheduler();
}
