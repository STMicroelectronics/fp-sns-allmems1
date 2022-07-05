/**
  ******************************************************************************
  * @file    bluenrg_l2cap_aci.c
  * @author  AMS - HEA&RF BU
  * @brief   File with HCI commands for BlueNRG FW6.0 and above
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2013 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "bluenrg_types.h"
#include "bluenrg_def.h"
#include "hci_const.h"
#include "bluenrg_aci_const.h"
#include "bluenrg_l2cap_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_gap.h"

#define MIN(a,b)            ((a) < (b) )? (a) : (b)
#define MAX(a,b)            ((a) > (b) )? (a) : (b)

tBleStatus aci_l2cap_connection_parameter_update_request(uint16_t conn_handle, uint16_t interval_min,
							 uint16_t interval_max, uint16_t slave_latency,
							 uint16_t timeout_multiplier)
{
  struct hci_request rq;
  uint8_t status;
  l2cap_conn_param_update_req_cp cp;

  cp.conn_handle = htobs(conn_handle);
  cp.interval_min = htobs(interval_min);
  cp.interval_max = htobs(interval_max);
  cp.slave_latency = htobs(slave_latency);
  cp.timeout_multiplier = htobs(timeout_multiplier);

  BLUENRG_memset(&rq, 0, sizeof(rq));
  rq.ogf = OGF_VENDOR_CMD;
  rq.ocf = OCF_L2CAP_CONN_PARAM_UPDATE_REQ;
  rq.cparam = &cp;
  rq.clen = L2CAP_CONN_PARAM_UPDATE_REQ_CP_SIZE;
  rq.event = EVT_CMD_STATUS;
  rq.rparam = &status;
  rq.rlen = 1;

  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;
  
  return status;  
}

tBleStatus aci_l2cap_connection_parameter_update_response_IDB05A1(uint16_t conn_handle, uint16_t interval_min,
							 uint16_t interval_max, uint16_t slave_latency,
							 uint16_t timeout_multiplier, uint16_t min_ce_length, uint16_t max_ce_length,
                             uint8_t id, uint8_t accept)
{
  struct hci_request rq;
  uint8_t status;
  l2cap_conn_param_update_resp_cp_IDB05A1 cp;

  cp.conn_handle = htobs(conn_handle);
  cp.interval_min = htobs(interval_min);
  cp.interval_max = htobs(interval_max);
  cp.slave_latency = htobs(slave_latency);
  cp.timeout_multiplier = htobs(timeout_multiplier);
  cp.min_ce_length = htobs(min_ce_length);
  cp.max_ce_length = htobs(max_ce_length);
  cp.id = id;
  cp.accept = accept;

  BLUENRG_memset(&rq, 0, sizeof(rq));
  rq.ogf = OGF_VENDOR_CMD;
  rq.ocf = OCF_L2CAP_CONN_PARAM_UPDATE_RESP;
  rq.cparam = &cp;
  rq.clen = sizeof(cp);
  rq.rparam = &status;
  rq.rlen = 1;

  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;

  return status;
}

tBleStatus aci_l2cap_connection_parameter_update_response_IDB04A1(uint16_t conn_handle, uint16_t interval_min,
							 uint16_t interval_max, uint16_t slave_latency,
							 uint16_t timeout_multiplier, uint8_t id, uint8_t accept)
{
  struct hci_request rq;
  uint8_t status;
  l2cap_conn_param_update_resp_cp_IDB04A1 cp;

  cp.conn_handle = htobs(conn_handle);
  cp.interval_min = htobs(interval_min);
  cp.interval_max = htobs(interval_max);
  cp.slave_latency = htobs(slave_latency);
  cp.timeout_multiplier = htobs(timeout_multiplier);
  cp.id = id;
  cp.accept = accept;

  BLUENRG_memset(&rq, 0, sizeof(rq));
  rq.ogf = OGF_VENDOR_CMD;
  rq.ocf = OCF_L2CAP_CONN_PARAM_UPDATE_RESP;
  rq.cparam = &cp;
  rq.clen = sizeof(cp);
  rq.rparam = &status;
  rq.rlen = 1;

  if (hci_send_req(&rq, FALSE) < 0)
    return BLE_STATUS_TIMEOUT;

  return status;
}
