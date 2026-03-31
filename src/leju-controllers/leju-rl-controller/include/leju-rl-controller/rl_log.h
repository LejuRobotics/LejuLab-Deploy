#pragma once

#include "lejusdk-utils/leju_log.h"

#define RL_TAG "RL"
#define RL_LOGI(...)    LOG_I(RL_TAG, __VA_ARGS__)
#define RL_LOGD(...)    LOG_D(RL_TAG, __VA_ARGS__)
#define RL_LOGE(...)    LOG_E(RL_TAG, __VA_ARGS__)
#define RL_LOGT(...)    LOG_T(RL_TAG, __VA_ARGS__)
#define RL_LOGW(...)    LOG_W(RL_TAG, __VA_ARGS__)

// 带颜色的日志宏
#define RL_LOG_INFO(...)     LOG_I(RL_TAG, __VA_ARGS__)
#define RL_LOG_SUCCESS(...)  LOG_SUCCESS(RL_TAG, __VA_ARGS__)
#define RL_LOG_WARNING(...)  LOG_WARNING(RL_TAG, __VA_ARGS__)
#define RL_LOG_FAILURE(...)  LOG_FAILURE(RL_TAG, __VA_ARGS__)
