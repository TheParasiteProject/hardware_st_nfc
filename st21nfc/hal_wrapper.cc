/******************************************************************************
 *
 *  Copyright (C) 2017 ST Microelectronics S.A.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *
 ******************************************************************************/
#define LOG_TAG "NfcNciHalWrapper"
#include <cutils/properties.h>
#include <errno.h>
#include <hardware/nfc.h>
#include <log/log.h>
#include <stpropnci.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "android_logmsg.h"
#include "hal_event_logger.h"
#include "hal_fd.h"
#include "halcore.h"
#include "i2clayer.h"
#include "st21nfc_dev.h"
#define OPEN_TIMEOUT_MAX_COUNT 5

extern void HalCoreCallback(void* context, uint32_t event, const void* d,
                            size_t length);
extern bool I2cOpenLayer(void* dev, HAL_CALLBACK callb, HALHANDLE* pHandle);
extern void I2cCloseLayer();
extern void I2cRecovery();

static void halWrapperDataCallback(uint16_t data_len, uint8_t* p_data);
static void halWrapperCallback(uint8_t event, uint8_t event_status);
static std::string hal_wrapper_state_to_str(uint16_t event);
static void hal_wrapper_store_timeout_log();

nfc_stack_callback_t* mHalWrapperCallback = NULL;
nfc_stack_data_callback_t* mHalWrapperDataCallback = NULL;
hal_wrapper_state_e mHalWrapperState = HAL_WRAPPER_STATE_CLOSED;
HALHANDLE mHalHandle = NULL;

/* Did we detect ST custom NFC stack ? */
bool is_st_stack = false;

uint8_t mClfMode;
uint8_t mFwUpdateTaskMask;
int mRetryFwDwl;
uint8_t mFwUpdateResMask = 0;
uint8_t* ConfigBuffer = NULL;
uint8_t mError_count = 0;
bool mIsActiveRW = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_activerw = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ready_cond = PTHREAD_COND_INITIALIZER;

static const uint8_t ApduGetAtr[] = {0x2F, 0x04, 0x05, 0x80,
                                     0x8A, 0x00, 0x00, 0x04};

static const uint8_t nciHeaderPropSetConfig[9] = {0x2F, 0x02, 0x98, 0x04, 0x00,
                                                  0x14, 0x01, 0x00, 0x92};
static uint8_t nciPropEnableFwDbgTraces[256];
static uint8_t nciPropGetFwDbgTracesConfig[] = {0x2F, 0x02, 0x05, 0x03,
                                                0x00, 0x14, 0x01, 0x00};
static uint8_t nciAndroidPassiveObserver[256];
static bool isDebuggable;

bool mReadFwConfigDone = false;

bool mHciCreditLent = false;
bool mfactoryReset = false;
bool ready_flag = 0;
bool mTimerStarted = false;
bool mFieldInfoTimerStarted = false;
bool forceRecover = false;
unsigned long hal_field_timer = 0;
unsigned long hal_activerw_timer = 0;
struct halconfig mHalconfig;
uint8_t propmsg[258];
uint16_t propmsgLen;

static bool sEnableFwLog = false;
uint8_t mObserverMode = 0;
bool mObserverRsp = false;
bool mPerTechCmdRsp = false;
bool storedLog = false;
bool mObserveModeSuspended = false;
bool mObserveModeSuspendPendingNotifyPollingLoop = false;
static uint16_t OpenTimeoutCount = 0;

bool mDisplayFwLog = false;

static inline void callHalWrapperDataCallback(uint16_t data_len,
                                              uint8_t* p_data) {
  if (stpropnci_process(MSG_DIR_FROM_NFCC, p_data, data_len)) {
    // STLOG_HAL_V("%s - message intercepted by stpropnci_process, drop",
    // __func__);
    return;
  }

  // Other cases, we send the frame to stack
  mHalWrapperDataCallback(data_len, p_data);  // send to the stack
}

void wait_ready() {
  pthread_mutex_lock(&mutex);
  while (!ready_flag) {
    pthread_cond_wait(&ready_cond, &mutex);
  }
  pthread_mutex_unlock(&mutex);
}

void set_ready(bool ready) {
  pthread_mutex_lock(&mutex);
  ready_flag = ready;
  pthread_cond_signal(&ready_cond);
  pthread_mutex_unlock(&mutex);
}

static void stpropnci_cb(bool dir_to_nfcc, uint8_t* payload,
                         uint16_t payloadlen) {
  STLOG_HAL_D("NFC-NCI HAL: %s  dir:%d hdr:%02hhx%02hhx%02hhx", __func__,
              dir_to_nfcc, payload[0], payload[1], payload[2]);
  if (dir_to_nfcc == MSG_DIR_TO_NFCC) {
    // send downward
    if (!HalSendDownstream(mHalHandle, payload, payloadlen)) {
      STLOG_HAL_D("NFC-NCI HAL: %s failed to send downstream", __func__);
    }
  } else {
    // Log message Hal to Stack
    DispHal("RX DATA H2S", (payload), payloadlen);
    mHalWrapperDataCallback(payloadlen, (payload));  // send to the stack
  }
}

bool hal_wrapper_open(st21nfc_dev_t* dev, nfc_stack_callback_t* p_cback,
                      nfc_stack_data_callback_t* p_data_cback,
                      HALHANDLE* pHandle) {
  bool result;

  STLOG_HAL_D("%s", __func__);

  set_ready(0);
  // Initializing structure holding some HAL configs
  memset(&mHalconfig, 0, sizeof(halconfig));

  mFwUpdateResMask = hal_fd_init();

  // init the stnciprop library
  if (!stpropnci_init((int)hal_trace_level, stpropnci_cb)) {
    return -1;
  }

  // update config
  GetNumValue(NAME_STNFC_ACTIVERW_TIMER, &hal_activerw_timer,
              sizeof(hal_activerw_timer));
  GetNumValue(NAME_STNFC_REMOTE_FIELD_TIMER, &hal_field_timer,
              sizeof(hal_field_timer));
  mHalconfig.hal_trace_level =  (int)hal_trace_level;
  mHalconfig.hal_activerw_timer =  (int)hal_activerw_timer;
  mHalconfig.hal_field_timer =  (int)hal_field_timer;
  stpropnci_config_settings (mHalconfig);

  mRetryFwDwl = 5;
  mFwUpdateTaskMask = 0;

  mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
  mHciCreditLent = false;
  mReadFwConfigDone = false;
  mError_count = 0;
  is_st_stack = false;
  mObserverMode = 0;
  mObserverRsp = false;
  mObserveModeSuspended = false;
  mObserveModeSuspendPendingNotifyPollingLoop = false;
  mDisplayFwLog = false;

  mHalWrapperCallback = p_cback;
  mHalWrapperDataCallback = p_data_cback;

  dev->p_data_cback = halWrapperDataCallback;
  dev->p_cback = halWrapperCallback;

  result = I2cOpenLayer(dev, HalCoreCallback, pHandle);

  if (!result || !(*pHandle)) {
    return -1;  // We are doomed, stop it here, NOW !
  }

  isDebuggable = property_get_int32("ro.debuggable", 0);
  mHalHandle = *pHandle;

  HalEventLogger::getInstance().initialize();
  HalEventLogger::getInstance().log() << __func__ << std::endl;
  HalEventLogger::getInstance().store_timer_activity("open", 10000);
  HalSendDownstreamTimer(mHalHandle, 10000);
  wait_ready();

  return 1;
}

int hal_wrapper_close(int call_cb, int nfc_mode) {
  STLOG_HAL_V("%s - Sending PROP_NFC_MODE_SET_CMD(%d)", __func__, nfc_mode);
  uint8_t propNfcModeSetCmdQb[] = {0x2f, 0x02, 0x02, 0x02, (uint8_t)nfc_mode};

  mHalWrapperState = HAL_WRAPPER_STATE_CLOSING;
  is_st_stack = false;
  HalEventLogger::getInstance().log() << __func__ << std::endl;
  // Send PROP_NFC_MODE_SET_CMD
  HalEventLogger::getInstance().store_timer_activity("close", 100);
  if (!HalSendDownstreamTimer(mHalHandle, propNfcModeSetCmdQb,
                              sizeof(propNfcModeSetCmdQb), 100)) {
    STLOG_HAL_E("NFC-NCI HAL: %s  HalSendDownstreamTimer failed", __func__);
    return -1;
  }
  // Let the CLF receive and process this
  usleep(50000);

  stpropnci_deinit();

  I2cCloseLayer();
  if (call_cb) mHalWrapperCallback(HAL_NFC_CLOSE_CPLT_EVT, HAL_NFC_STATUS_OK);

  return 1;
}

void hal_wrapper_send_core_config_prop() {
  long retlen = 0;
  int isfound = 0;

  // allocate buffer for setting parameters
  ConfigBuffer = (uint8_t*)malloc(256 * sizeof(uint8_t));
  if (ConfigBuffer != NULL) {
    isfound = GetByteArrayValue(NAME_CORE_CONF_PROP, (char*)ConfigBuffer, 256,
                                &retlen);

    if (isfound > 0) {
      STLOG_HAL_V("%s - Enter", __func__);
      set_ready(0);

      HalEventLogger::getInstance().store_timer_activity("send core config",
                                                         1000);
      if (!HalSendDownstreamTimer(mHalHandle, ConfigBuffer, retlen, 1000)) {
        STLOG_HAL_E("NFC-NCI HAL: %s  SendDownstream failed", __func__);
      }
      wait_ready();
    }
    free(ConfigBuffer);
    ConfigBuffer = NULL;
  }
}

void hal_wrapper_send_vs_config() {
  STLOG_HAL_V("%s - Enter", __func__);
  set_ready(0);
  mHalWrapperState = HAL_WRAPPER_STATE_PROP_CONFIG;
  mReadFwConfigDone = true;
  HalEventLogger::getInstance().store_timer_activity("send vs config", 1000);
  if (!HalSendDownstreamTimer(mHalHandle, nciPropGetFwDbgTracesConfig,
                              sizeof(nciPropGetFwDbgTracesConfig), 1000)) {
    STLOG_HAL_E("%s - SendDownstream failed", __func__);
  }
  wait_ready();
}

void hal_wrapper_send_config() {
  hal_wrapper_send_vs_config();
  mHalWrapperState = HAL_WRAPPER_STATE_PROP_CONFIG;
  hal_wrapper_send_core_config_prop();
}

void hal_wrapper_factoryReset() {
  mfactoryReset = true;
  STLOG_HAL_V("%s - mfactoryReset = %d", __func__, mfactoryReset);
}

void hal_wrapper_set_observer_mode(uint8_t enable, bool per_tech_cmd) {
  mObserverMode = enable;
  mObserverRsp = true;
  mPerTechCmdRsp = per_tech_cmd;
  mObserveModeSuspended = false;
  mObserveModeSuspendPendingNotifyPollingLoop = false;
}
void hal_wrapper_get_observer_mode() { mObserverRsp = true; }

void hal_wrapper_update_complete() {
  STLOG_HAL_V("%s ", __func__);
  mHalWrapperCallback(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_OK);
  mHalWrapperState = HAL_WRAPPER_STATE_OPEN_CPLT;
}
void halWrapperDataCallback(uint16_t data_len, uint8_t* p_data) {
  uint8_t propNfcModeSetCmdOn[] = {0x2f, 0x02, 0x02, 0x02, 0x01};
  uint8_t coreInitCmd[] = {0x20, 0x01, 0x02, 0x00, 0x00};
  uint8_t coreResetCmd[] = {0x20, 0x00, 0x01, 0x01};
  unsigned long num = 0;
  unsigned long swp_log = 0;
  unsigned long rf_log = 0;
  int mObserverLength = 0;
  int nciPropEnableFwDbgTraces_size = sizeof(nciPropEnableFwDbgTraces);

  switch (mHalWrapperState) {
    case HAL_WRAPPER_STATE_CLOSED:  // 0
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_CLOSED", __func__);
      break;
    case HAL_WRAPPER_STATE_OPEN:  // 1
      // CORE_RESET_NTF
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_OPEN", __func__);
      stpropnci_inform(false, p_data, data_len);

      if ((p_data[0] == 0x60) && (p_data[1] == 0x00)) {
        mIsActiveRW = false;
        mFwUpdateTaskMask = ft_cmd_HwReset(p_data, &mClfMode);

        if (mfactoryReset == true) {
          STLOG_HAL_V(
              "%s - first boot after factory reset detected - start FW update",
              __func__);
          if ((mFwUpdateResMask & FW_PATCH_AVAILABLE) &&
              (mFwUpdateResMask & FW_CUSTOM_PARAM_AVAILABLE)) {
            mFwUpdateTaskMask = FW_UPDATE_NEEDED | CONF_UPDATE_NEEDED;
            mfactoryReset = false;
          }
        }
        STLOG_HAL_V(
            "%s - mFwUpdateTaskMask = %d,  mClfMode = %d,  mRetryFwDwl = %d",
            __func__, mFwUpdateTaskMask, mClfMode, mRetryFwDwl);
        // CLF in MODE LOADER & Update needed.
        if (mClfMode == FT_CLF_MODE_LOADER) {
          HalSendDownstreamStopTimer(mHalHandle);
          STLOG_HAL_V("%s --- CLF mode is LOADER ---", __func__);

          if (mRetryFwDwl == 0) {
            STLOG_HAL_W(
                "%s - Reached maximum nb of retries, FW update failed, exiting",
                __func__);
            mHalWrapperCallback(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_FAILED);
            I2cCloseLayer();
          } else {
            mHalWrapperState = HAL_WRAPPER_STATE_UPDATE;
            if (((p_data[3] == 0x01) && (p_data[8] == HW_ST54L)) ||
                ((p_data[2] == 0x41) && (p_data[3] == 0xA2))) {  // ST54L
              FwUpdateHandler(mHalHandle, data_len, p_data);
            } else {
              STLOG_HAL_V("%s - Send APDU_GET_ATR_CMD", __func__);
              HalEventLogger::getInstance().log()
                  << __func__ << " Send APDU_GET_ATR_CMD" << std::endl;
              HalEventLogger::getInstance().store_timer_activity(
                  "Send APDU_GET_ATR_CMD", FW_TIMER_DURATION);
              if (!HalSendDownstreamTimer(mHalHandle, ApduGetAtr,
                                          sizeof(ApduGetAtr),
                                          FW_TIMER_DURATION)) {
                STLOG_HAL_E("%s - SendDownstream failed", __func__);
              }
            }
            mRetryFwDwl--;
          }
        } else if (mFwUpdateTaskMask == 0 || mRetryFwDwl == 0) {
          STLOG_HAL_V("%s - Proceeding with normal startup", __func__);
          if (p_data[3] == 0x01) {
            // Normal mode, start HAL
            mHalWrapperCallback(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_OK);
            mHalWrapperState = HAL_WRAPPER_STATE_OPEN_CPLT;
          } else {
            // No more retries or CLF not in correct mode
            mHalWrapperCallback(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_FAILED);
          }
          // CLF in MODE ROUTER & Update needed.
        } else if (mClfMode == FT_CLF_MODE_ROUTER) {
          if ((mFwUpdateTaskMask & FW_UPDATE_NEEDED) &&
              (mFwUpdateResMask & FW_PATCH_AVAILABLE)) {
            STLOG_HAL_V(
                "%s - CLF in ROUTER mode, FW update needed, try upgrade FW -",
                __func__);
            mRetryFwDwl--;

            if (!HalSendDownstream(mHalHandle, coreResetCmd,
                                   sizeof(coreResetCmd))) {
              STLOG_HAL_E("%s - SendDownstream failed", __func__);
            }
            mHalWrapperState = HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL;
          } else if ((mFwUpdateTaskMask & CONF_UPDATE_NEEDED) &&
                     (mFwUpdateResMask & FW_CUSTOM_PARAM_AVAILABLE)) {
            if (!HalSendDownstream(mHalHandle, coreResetCmd,
                                   sizeof(coreResetCmd))) {
              STLOG_HAL_E("%s - SendDownstream failed", __func__);
            }
            mHalWrapperState = HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM;
          } else if ((mFwUpdateTaskMask & UWB_CONF_UPDATE_NEEDED) &&
                     (mFwUpdateResMask & FW_UWB_PARAM_AVAILABLE)) {
            if (!HalSendDownstream(mHalHandle, coreResetCmd,
                                   sizeof(coreResetCmd))) {
              STLOG_HAL_E("%s - SendDownstream failed", __func__);
            }
            mHalWrapperState = HAL_WRAPPER_STATE_APPLY_UWB_PARAM;
          }
        }
      } else {
        callHalWrapperDataCallback(data_len, p_data);
      }
      set_ready(1);
      break;
    case HAL_WRAPPER_STATE_OPEN_CPLT:  // 2
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_OPEN_CPLT",
                  __func__);
      stpropnci_inform(false, p_data, data_len);
      // CORE_INIT_RSP
      if ((p_data[0] == 0x40) && (p_data[1] == 0x01)) {
      } else if ((p_data[0] == 0x60) && (p_data[1] == 0x06)) {
        STLOG_HAL_V("%s - Sending PROP_NFC_MODE_SET_CMD", __func__);

        // Send PROP_NFC_MODE_SET_CMD(ON)
        mHalWrapperState = HAL_WRAPPER_STATE_NFC_ENABLE_ON;
        HalEventLogger::getInstance().store_timer_activity(
            "Sending PROP_NFC_MODE_SET_CMD", 500);
        if (!HalSendDownstreamTimer(mHalHandle, propNfcModeSetCmdOn,
                                    sizeof(propNfcModeSetCmdOn), 500)) {
          STLOG_HAL_E("NFC-NCI HAL: %s  HalSendDownstreamTimer failed",
                      __func__);
        }
      } else {
        callHalWrapperDataCallback(data_len, p_data);
      }
      break;

    case HAL_WRAPPER_STATE_NFC_ENABLE_ON:  // 3
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_NFC_ENABLE_ON",
                  __func__);
      stpropnci_inform(false, p_data, data_len);
      // PROP_NFC_MODE_SET_RSP
      if ((p_data[0] == 0x4f) && (p_data[1] == 0x02)) {
        // DO nothing: wait for core_reset_ntf or timer timeout
      }
      // CORE_RESET_NTF
      else if ((p_data[0] == 0x60) && (p_data[1] == 0x00)) {
        // Stop timer
        HalSendDownstreamStopTimer(mHalHandle);
        if (forceRecover == true) {
          forceRecover = false;
          callHalWrapperDataCallback(data_len, p_data);
          break;
        }

        // Send CORE_INIT_CMD
        STLOG_HAL_V("%s - Sending CORE_INIT_CMD", __func__);
        if (!HalSendDownstream(mHalHandle, coreInitCmd, sizeof(coreInitCmd))) {
          STLOG_HAL_E("NFC-NCI HAL: %s  SendDownstream failed", __func__);
        }
      }
      // CORE_INIT_RSP
      else if ((p_data[0] == 0x40) && (p_data[1] == 0x01)) {
        STLOG_HAL_D("%s - NFC mode enabled", __func__);
        // Do we need to lend a credit ?
        if (p_data[13] == 0x00) {
          STLOG_HAL_D("%s - 1 credit lent", __func__);
          p_data[13] = 0x01;
          mHciCreditLent = true;
        }

        mHalWrapperState = HAL_WRAPPER_STATE_READY;
        callHalWrapperDataCallback(data_len, p_data);
      }
      break;

    case HAL_WRAPPER_STATE_PROP_CONFIG:  // 4
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_PROP_CONFIG",
                  __func__);
      stpropnci_inform(false, p_data, data_len);
      // CORE_SET_CONFIG_RSP
      if ((p_data[0] == 0x40) && (p_data[1] == 0x02)) {
        HalSendDownstreamStopTimer(mHalHandle);
        GetNumValue(NAME_STNFC_REMOTE_FIELD_TIMER, &hal_field_timer,
                    sizeof(hal_field_timer));
        STLOG_HAL_D("%s - hal_field_timer = %lu", __func__, hal_field_timer);
        set_ready(1);
        // Exit state, all processing done
        mHalWrapperCallback(HAL_NFC_POST_INIT_CPLT_EVT, HAL_NFC_STATUS_OK);
        mHalWrapperState = HAL_WRAPPER_STATE_READY;
      } else if (mHciCreditLent && (p_data[0] == 0x60) && (p_data[1] == 0x06)) {
        // CORE_CONN_CREDITS_NTF
        if (p_data[4] == 0x01) {  // HCI connection
          mHciCreditLent = false;
          STLOG_HAL_D("%s - credit returned", __func__);
          if (p_data[5] == 0x01) {
            // no need to send this.
            break;
          } else {
            if (p_data[5] != 0x00 && p_data[5] != 0xFF) {
              // send with 1 less
              p_data[5]--;
            }
          }
        }
        callHalWrapperDataCallback(data_len, p_data);
      } else if (p_data[0] == 0x4f) {
        // PROP_RSP
        if (mReadFwConfigDone == true) {
          mReadFwConfigDone = false;
          HalSendDownstreamStopTimer(mHalHandle);
          // NFC_STATUS_OK
          if (p_data[3] == 0x00) {
            bool confNeeded = false;
            bool firmware_debug_enabled = property_get_int32(
                "persist.vendor.nfc.firmware_debug_enabled", 0);

            // Check if FW DBG shall be set
            if (GetNumValue(NAME_STNFC_FW_DEBUG_ENABLED, &num, sizeof(num)) ||
                isDebuggable || sEnableFwLog) {
              if (firmware_debug_enabled || sEnableFwLog) {
                num = 1;
                swp_log = 30;
                mDisplayFwLog = true;
              } else if (isDebuggable) {
                swp_log = 30;
                mDisplayFwLog = true;
              } else {
                swp_log = 8;
                mDisplayFwLog = false;
              }
              rf_log = 15;

              if (num == 1) {
                GetNumValue(NAME_STNFC_FW_SWP_LOG_SIZE, &swp_log,
                            sizeof(swp_log));
                GetNumValue(NAME_STNFC_FW_RF_LOG_SIZE, &rf_log, sizeof(rf_log));
              }
              // limit swp and rf payload length between 4 and 30.
              if (swp_log > 30)
                swp_log = 30;
              else if (swp_log < 4)
                swp_log = 4;

              if (rf_log > 30)
                rf_log = 30;
              else if (rf_log < 4)
                rf_log = 4;

              if ((rf_log || swp_log) &&
                  ((p_data[15] != rf_log) || (p_data[17] != swp_log))) {
                STLOG_HAL_D("%s - FW DBG payload traces changes needed",
                            __func__);
                confNeeded = true;
              }

              // If conf file indicate set needed and not yet enabled
              if ((num == 1) && (p_data[7] == 0x00)) {
                STLOG_HAL_D("%s - FW DBG traces enabling needed", __func__);
                nciPropEnableFwDbgTraces[9] = 0x01;
                confNeeded = true;
              } else if ((num == 0) && (p_data[7] == 0x01)) {
                STLOG_HAL_D("%s - FW DBG traces disabling needed", __func__);
                nciPropEnableFwDbgTraces[9] = 0x00;
                confNeeded = true;
              } else {
                STLOG_HAL_D(
                    "%s - No FW DBG traces enable/disable change needed",
                    __func__);
              }

              if (data_len < 9 || p_data[6] == 0 ||
                  p_data[6] < (data_len - 7) ||
                  p_data[6] > (sizeof(nciPropEnableFwDbgTraces) - 9)) {
                if (confNeeded) {
                  android_errorWriteLog(0x534e4554, "169328517");
                  confNeeded = false;
                }
              }

              if (confNeeded) {
                memcpy(nciPropEnableFwDbgTraces, nciHeaderPropSetConfig, 9);
                memcpy(&nciPropEnableFwDbgTraces[10], &p_data[8],
                       p_data[6] - 1);
                if (rf_log || swp_log) {
                  nciPropEnableFwDbgTraces[9] = (uint8_t)num;
                  nciPropEnableFwDbgTraces[17] = (uint8_t)rf_log;
                  nciPropEnableFwDbgTraces[19] = (uint8_t)swp_log;
                }
                if ((9 + p_data[6]) < sizeof(nciPropEnableFwDbgTraces)) {
                  nciPropEnableFwDbgTraces_size = 9 + p_data[6];
                }

                confNeeded = false;

                if (!HalSendDownstream(mHalHandle, nciPropEnableFwDbgTraces,
                                       nciPropEnableFwDbgTraces_size)) {
                  STLOG_HAL_E("%s - SendDownstream failed", __func__);
                }
                mHalWrapperState = HAL_WRAPPER_STATE_APPLY_PROP_CONFIG;
                break;
              } else {
                set_ready(1);
              }
            }
          } else {
            set_ready(1);
          }
        } else {
          set_ready(1);
        }
      }
      break;

    case HAL_WRAPPER_STATE_READY:  // 5
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_READY", __func__);

      if (!((p_data[0] == 0x60) && (p_data[3] == 0xa0))) {
        if (mHciCreditLent && (p_data[0] == 0x60) && (p_data[1] == 0x06)) {
          if (p_data[4] == 0x01) {  // HCI connection
            mHciCreditLent = false;
            STLOG_HAL_D("%s - credit returned", __func__);
            if (p_data[5] == 0x01) {
              // no need to send this.
              break;
            } else {
              if (p_data[5] != 0x00 && p_data[5] != 0xFF) {
                // send with 1 less
                p_data[5]--;
              }
            }
          }
        } else if ((p_data[0] == 0x61) && (p_data[1] == 0x07)) {
          // RF_FIELD_INFO_NTF
          if (p_data[3] == 0x01) {  // field on
            // start timer
            if (hal_field_timer) {
              mFieldInfoTimerStarted = true;
              HalEventLogger::getInstance().store_timer_activity("field on",
                                                                 20000);
              HalSendDownstreamTimer(mHalHandle, 20000);
            }
          } else if (p_data[3] == 0x00) {
            if (mFieldInfoTimerStarted) {
              HalSendDownstreamStopTimer(mHalHandle);
              mFieldInfoTimerStarted = false;
            }
          }
        }else if (p_data[0] == 0x60 && p_data[1] == 0x00) {
          stpropnci_inform(false, p_data, data_len);
          STLOG_HAL_E("%s - Reset trigger from 0x%x to 0x0", __func__,
                      p_data[3]);
          p_data[3] = 0x0;  // Only reset trigger that should be received in
                            // HAL_WRAPPER_STATE_READY is unreocoverable error.
          mHalWrapperState = HAL_WRAPPER_STATE_RECOVERY;
        }
        callHalWrapperDataCallback(data_len, p_data);
      } else {
        STLOG_HAL_V("%s - Core reset notification - Nfc mode ", __func__);
      }
      break;

    case HAL_WRAPPER_STATE_CLOSING:  // 6
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_CLOSING",
                  __func__);
      stpropnci_inform(false, p_data, data_len);
      if ((p_data[0] == 0x4f) && (p_data[1] == 0x02)) {
        hal_fd_close();
        // intercept this expected message, don t forward.
        mHalWrapperState = HAL_WRAPPER_STATE_CLOSED;
      } else {
        callHalWrapperDataCallback(data_len, p_data);
      }
      break;

    case HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL:  // 6
      STLOG_HAL_V(
          "%s - mHalWrapperState = HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL",
          __func__);
      ExitHibernateHandler(mHalHandle, data_len, p_data);
      break;

    case HAL_WRAPPER_STATE_UPDATE:  // 7
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_UPDATE", __func__);
      stpropnci_inform(false, p_data, data_len);
      FwUpdateHandler(mHalHandle, data_len, p_data);
      break;
    case HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM:  // 8
      STLOG_HAL_V(
          "%s - mHalWrapperState = HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM",
          __func__);
      stpropnci_inform(false, p_data, data_len);
      ApplyCustomParamHandler(mHalHandle, data_len, p_data);
      break;
    case HAL_WRAPPER_STATE_APPLY_UWB_PARAM:  // 9
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_APPLY_UWB_PARAM",
                  __func__);
      ApplyUwbParamHandler(mHalHandle, data_len, p_data);
      break;
    case HAL_WRAPPER_STATE_APPLY_PROP_CONFIG:
      STLOG_HAL_V("%s - mHalWrapperState = HAL_WRAPPER_STATE_APPLY_PROP_CONFIG",
                  __func__);
      if (p_data[0] == 0x4f) {
        I2cResetPulse();
      } else if ((p_data[0] == 0x60) && (p_data[1] == 0x00)) {
        // Send CORE_INIT_CMD
        STLOG_HAL_D("%s - Sending CORE_INIT_CMD", __func__);
        if (!HalSendDownstream(mHalHandle, coreInitCmd, sizeof(coreInitCmd))) {
          STLOG_HAL_E("NFC-NCI HAL: %s  SendDownstream failed", __func__);
        }
      }
      // CORE_INIT_RSP
      else if ((p_data[0] == 0x40) && (p_data[1] == 0x01)) {
        set_ready(1);
      }
      break;
    case HAL_WRAPPER_STATE_RECOVERY:
      STLOG_HAL_W("%s - mHalWrapperState = HAL_WRAPPER_STATE_RECOVERY",
                  __func__);
      break;
  }
}

static void halWrapperCallback(uint8_t event,
                               __attribute__((unused)) uint8_t event_status) {
  uint8_t coreInitCmd[] = {0x20, 0x01, 0x02, 0x00, 0x00};
  uint8_t rfDeactivateCmd[] = {0x21, 0x06, 0x01, 0x00};
  uint8_t p_data[6];
  uint16_t data_len;

  switch (mHalWrapperState) {
    case HAL_WRAPPER_STATE_CLOSING:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_D("NFC-NCI HAL: %s  Timeout. Close anyway", __func__);
        HalSendDownstreamStopTimer(mHalHandle);
        hal_fd_close();
        mHalWrapperState = HAL_WRAPPER_STATE_CLOSED;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_OPEN:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        set_ready(1);
        OpenTimeoutCount++;
        STLOG_HAL_E(
            "NFC-NCI HAL: %s  Timeout accessing the CLF. OpenTimeoutCount:%d",
            __func__, OpenTimeoutCount);
        HalSendDownstreamStopTimer(mHalHandle);
        hal_wrapper_store_timeout_log();
        if (OpenTimeoutCount > OPEN_TIMEOUT_MAX_COUNT) {
          mHalWrapperState = HAL_WRAPPER_STATE_CLOSED;
          OpenTimeoutCount = 0;
          return;
        }
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xAF;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        mHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_CLOSED:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_D("NFC-NCI HAL: %s  Timeout. Close anyway", __func__);
        HalSendDownstreamStopTimer(mHalHandle);
        return;
      }
      break;

    case HAL_WRAPPER_STATE_UPDATE:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("%s - Timer for FW update procedure timeout, retry",
                    __func__);
        hal_wrapper_store_timeout_log();
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xAE;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        mHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_NFC_ENABLE_ON:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        // timeout
        // Send CORE_INIT_CMD
        STLOG_HAL_V("%s - Sending CORE_INIT_CMD", __func__);
        if (!HalSendDownstream(mHalHandle, coreInitCmd, sizeof(coreInitCmd))) {
          STLOG_HAL_E("NFC-NCI HAL: %s  SendDownstream failed", __func__);
        }
        return;
      }
      break;
    case HAL_WRAPPER_STATE_PROP_CONFIG:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("%s - Timer when sending conf parameters, retry", __func__);
        hal_wrapper_store_timeout_log();
        abort();  // TODO: fix it when we have a better recovery method.
        HalSendDownstreamStopTimer(mHalHandle);
        resetHandlerState();
        I2cResetPulse();
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
      }
      break;

    case HAL_WRAPPER_STATE_READY:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        if (mTimerStarted || mFieldInfoTimerStarted) {
          STLOG_HAL_E("NFC-NCI HAL: %s  Timeout.. Recover!", __func__);
          STLOG_HAL_E("%s mIsActiveRW = %d", __func__, mIsActiveRW);
          HalSendDownstreamStopTimer(mHalHandle);
          mTimerStarted = false;
          mFieldInfoTimerStarted = false;
          // forceRecover = true;
          resetHandlerState();
          if (!HalSendDownstream(mHalHandle, rfDeactivateCmd,
                                 sizeof(rfDeactivateCmd))) {
            STLOG_HAL_E("%s - SendDownstream failed", __func__);
          }
          usleep(10000);
          // Core Generic Error

          p_data[0] = 0x60;
          p_data[1] = 0x00;
          p_data[2] = 0x03;
          p_data[3] = 0xAA;
          p_data[4] = 0x00;
          p_data[5] = 0x00;
          data_len = 0x6;
          callHalWrapperDataCallback(data_len, p_data);
          mHalWrapperState = HAL_WRAPPER_STATE_RECOVERY;
        }
        return;
      }
      break;

    case HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("NFC-NCI HAL: %s  Timeout at state: %s", __func__,
                    hal_wrapper_state_to_str(mHalWrapperState).c_str());
        hal_wrapper_store_timeout_log();
        HalSendDownstreamStopTimer(mHalHandle);
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xAB;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        callHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_OPEN_CPLT:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("NFC-NCI HAL: %s  Timeout at state: %s", __func__,
                    hal_wrapper_state_to_str(mHalWrapperState).c_str());
        hal_wrapper_store_timeout_log();
        HalSendDownstreamStopTimer(mHalHandle);
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xAC;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        callHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("NFC-NCI HAL: %s  Timeout at state: %s", __func__,
                    hal_wrapper_state_to_str(mHalWrapperState).c_str());
        hal_wrapper_store_timeout_log();
        HalSendDownstreamStopTimer(mHalHandle);
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xAD;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        callHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    case HAL_WRAPPER_STATE_RECOVERY:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("NFC-NCI HAL: %s  Timeout at state: %s", __func__,
                    hal_wrapper_state_to_str(mHalWrapperState).c_str());
        hal_wrapper_store_timeout_log();
        HalSendDownstreamStopTimer(mHalHandle);
        p_data[0] = 0x60;
        p_data[1] = 0x00;
        p_data[2] = 0x03;
        p_data[3] = 0xBA;
        p_data[4] = 0x00;
        p_data[5] = 0x00;
        data_len = 0x6;
        mHalWrapperDataCallback(data_len, p_data);
        mHalWrapperState = HAL_WRAPPER_STATE_OPEN;
        return;
      }
      break;

    default:
      if (event == HAL_WRAPPER_TIMEOUT_EVT) {
        STLOG_HAL_E("NFC-NCI HAL: %s  Timeout at state: %s", __func__,
                    hal_wrapper_state_to_str(mHalWrapperState).c_str());
        if (!storedLog) {
          hal_wrapper_store_timeout_log();
          storedLog = true;
        }
      }
      break;
  }

  mHalWrapperCallback(event, event_status);
}

/*******************************************************************************
 **
 ** Function         nfc_set_state
 **
 ** Description      Set the state of NFC stack
 **
 ** Returns          void
 **
 *******************************************************************************/
void hal_wrapper_set_state(hal_wrapper_state_e new_wrapper_state) {
  ALOGD("nfc_set_state %d->%d", mHalWrapperState, new_wrapper_state);

  mHalWrapperState = new_wrapper_state;
}

/*******************************************************************************
 **
 ** Function         hal_wrapper_setFwLogging
 **
 ** Description      Enable the FW log by hte GUI if needed.
 **
 ** Returns          void
 **
 *******************************************************************************/
void hal_wrapper_setFwLogging(bool enable) {
  ALOGD("%s : enable = %d", __func__, enable);

  sEnableFwLog = enable;
}

/*******************************************************************************
 **
 ** Function         hal_wrapper_dumplog
 **
 ** Description      Dump HAL event logs.
 **
 ** Returns          void
 **
 *******************************************************************************/
void hal_wrapper_dumplog(int fd) {
  ALOGD("%s : fd= %d", __func__, fd);

  HalEventLogger::getInstance().dump_log(fd);
}

/*******************************************************************************
**
** Function         hal_wrapper_state_to_str
**
** Description      convert wrapper state to string
**
** Returns          string
**
*******************************************************************************/
static std::string hal_wrapper_state_to_str(uint16_t event) {
  switch (event) {
    case HAL_WRAPPER_STATE_CLOSED:
      return "HAL_WRAPPER_STATE_CLOSED";
    case HAL_WRAPPER_STATE_OPEN:
      return "HAL_WRAPPER_STATE_OPEN";
    case HAL_WRAPPER_STATE_OPEN_CPLT:
      return "HAL_WRAPPER_STATE_OPEN_CPLT";
    case HAL_WRAPPER_STATE_NFC_ENABLE_ON:
      return "HAL_WRAPPER_STATE_NFC_ENABLE_ON";
    case HAL_WRAPPER_STATE_PROP_CONFIG:
      return "HAL_WRAPPER_STATE_PROP_CONFIG";
    case HAL_WRAPPER_STATE_READY:
      return "HAL_WRAPPER_STATE_READY";
    case HAL_WRAPPER_STATE_CLOSING:
      return "HAL_WRAPPER_STATE_CLOSING";
    case HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL:
      return "HAL_WRAPPER_STATE_EXIT_HIBERNATE_INTERNAL";
    case HAL_WRAPPER_STATE_UPDATE:
      return "HAL_WRAPPER_STATE_UPDATE";
    case HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM:
      return "HAL_WRAPPER_STATE_APPLY_CUSTOM_PARAM";
    case HAL_WRAPPER_STATE_APPLY_UWB_PARAM:
      return "HAL_WRAPPER_STATE_APPLY_UWB_PARAM";
    case HAL_WRAPPER_STATE_APPLY_PROP_CONFIG:
      return "HAL_WRAPPER_STATE_APPLY_PROP_CONFIG";
    case HAL_WRAPPER_STATE_RECOVERY:
      return "HAL_WRAPPER_STATE_RECOVERY";
    default:
      return "Unknown";
  }
}

/*******************************************************************************
**
** Function         hal_wrapper_store_timeout_log
**
** Description      Store timeout event logs.
**
** Returns          void
*******************************************************************************/
static void hal_wrapper_store_timeout_log() {
  HalEventLogger::getInstance().log()
      << " Timeout at state: " << hal_wrapper_state_to_str(mHalWrapperState)
      << " mIsActiveRW=" << mIsActiveRW << " mTimerStarted=" << mTimerStarted
      << " activity=" << TimerAct.activity << " duration=" << TimerAct.duration
      << std::endl;
  HalEventLogger::getInstance().store_log();
}
