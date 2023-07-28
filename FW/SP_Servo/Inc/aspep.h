/**
  ******************************************************************************
  * @file  aspep.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief  This file provides aspep API that implement the aspep protocol
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef aspep_h
#define aspep_h

#include "parameters_conversion.h"
#include "mcptl.h"

#define ASPEP_CTRL    ((uint8_t)0) /* Beacon, Nack, or Ping*/

#define ASPEP_OK                  ((uint8_t)0)
#define ASPEP_SYNC_NOT_EXPECTED   1
#define ASPEP_NOT_CONNECTED       2
#define ASPEP_BUFFER_ERROR        3

#define ASPEP_BAD_PACKET_TYPE 1
#define ASPEP_BAD_PACKET_SIZE 2
#define ASPEP_BAD_CRC_HEADER  4
#define ASPEP_BAD_CRC_DATA    5

#define ASPEP_PING_RESET 0
#define ASPEP_PING_CFG   1

#define ASPEP_HEADER_SIZE 4
#define ASPEP_CTRL_SIZE 4
#define ASPEP_DATACRC_SIZE 2U

#define ID_MASK     ((uint32_t)0xF)
#define DATA_PACKET ((uint32_t)0x9)
#define PING        ((uint32_t)0x6)
#define BEACON      ((uint32_t)0x5)
#define NACK        ((uint32_t)0xF)
#define ACK         ((uint32_t)0xA)
typedef uint32_t ASPEP_packetType;

typedef bool (*ASPEP_send_cb_t)    (void *pHW_Handle, void *txbuffer, uint16_t length);
typedef void (*ASPEP_receive_cb_t) (void *pHW_Handle, void *rxbuffer, uint16_t length);
typedef void (*ASPEP_hwinit_cb_t)  (void *pHW_Handle);
typedef void (*ASPEP_hwsync_cb_t)  (void *pHW_Handle);

/* ASPEP Sate Machine Type*/
typedef enum
{
  ASPEP_IDLE,
  ASPEP_CONFIGURED,
  ASPEP_CONNECTED,
} ASPEP_sm_type;

/* ASPEP Transport Layer Sate Machine Type*/
typedef enum
{
  WAITING_PACKET,
  WAITING_PAYLOAD,
} ASPEP_TL_sm_type;

typedef struct
{
  uint8_t buffer[ASPEP_HEADER_SIZE];
  buff_access_t state;
} ASPEP_ctrlBuff_t;

typedef struct
{
  uint8_t DATA_CRC;
  uint8_t RX_maxSize;
  uint8_t TXS_maxSize;
  uint8_t TXA_maxSize;
  uint8_t version;
} ASPEP_Capabilities_def;

typedef struct
{
  MCTL_Handle_t _Super;
  void *HWIp;
  uint8_t *rxBuffer;   /* Contains the ASPEP Data payload */
  uint8_t rxHeader[4]; /* Contains the ASPEP 32 bits header*/
  ASPEP_ctrlBuff_t ctrlBuffer;
  MCTL_Buff_t syncBuffer;
  MCTL_Buff_t asyncBufferA;
  MCTL_Buff_t asyncBufferB;
  MCTL_Buff_t *lastRequestedAsyncBuff;
  MCTL_Buff_t *asyncNextBuffer;
  void *lockBuffer;
  ASPEP_hwinit_cb_t fASPEP_HWInit;
  ASPEP_hwsync_cb_t fASPEP_HWSync;
  ASPEP_receive_cb_t fASPEP_receive;
  ASPEP_send_cb_t fASPEP_send;
  uint16_t rxLength;
  uint16_t maxRXPayload;
  uint8_t syncPacketCount;
  bool NewPacketAvailable;
  uint8_t badPacketFlag; /* Contains the error code in case of ASPEP decoding issue */
  uint8_t liid;
  ASPEP_sm_type ASPEP_State;
  ASPEP_TL_sm_type ASPEP_TL_State;
  ASPEP_packetType rxPacketType;
  ASPEP_Capabilities_def Capabilities;
} ASPEP_Handle_t;

void ASPEP_start(ASPEP_Handle_t *pHandle);
/* MCTL (Motor Control Transport Layer) API */
bool ASPEP_getBuffer(MCTL_Handle_t *pHandle, void **buffer, uint8_t syncAsync);
uint8_t ASPEP_sendPacket(MCTL_Handle_t *pHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync);
uint8_t *ASPEP_RXframeProcess(MCTL_Handle_t *pHandle, uint16_t *packetLength);
/*   */
void ASPEP_HWDataReceivedIT(ASPEP_Handle_t *pHandle);
void ASPEP_HWDataTransmittedIT(ASPEP_Handle_t *pHandle);
/* Debugger stuff*/
void ASPEP_HWDMAReset(ASPEP_Handle_t *pHandle);
#endif

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
