#include "hard_sync.h"

#include "hard_sync_port.h"
#include "schm_hard_sync.h"

static const HardSyncConfig *g_config = 0;
static bool g_initialized = false;

static HardSyncResult ValidateOutput(const HardSyncOutputConfig *output) {
  if (output == 0) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (!output->enabled) {
    return HARD_SYNC_OK;
  }
  if (output->period_us == 0u || output->pulse_width_us == 0u) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (output->pulse_width_us >= output->period_us) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (output->phase_offset_us >= output->period_us) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (output->signal_kind > HARD_SYNC_SIGNAL_LIDAR_FRAME_TRIGGER) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (output->polarity > HARD_SYNC_ACTIVE_LOW) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  return HARD_SYNC_OK;
}

HardSyncResult HardSync_ValidateConfig(const HardSyncConfig *config) {
  if (config == 0 || config->outputs == 0 || config->output_count == 0u ||
      config->output_count > HARD_SYNC_MAX_OUTPUTS ||
      config->timebase_hz == 0u) {
    return HARD_SYNC_INVALID_CONFIG;
  }

  for (size_t i = 0u; i < config->output_count; ++i) {
    HardSyncResult result = ValidateOutput(&config->outputs[i]);
    if (result != HARD_SYNC_OK) {
      return result;
    }
  }

  return HARD_SYNC_OK;
}

HardSyncResult HardSync_Init(const HardSyncConfig *config) {
  HardSyncResult result = HardSync_ValidateConfig(config);
  if (result != HARD_SYNC_OK) {
    return result;
  }

  result = HardSyncPort_InitTimebase(config->timebase_hz);
  if (result != HARD_SYNC_OK) {
    return HARD_SYNC_PORT_ERROR;
  }

  for (size_t i = 0u; i < config->output_count; ++i) {
    if (!config->outputs[i].enabled) {
      continue;
    }
    result = HardSyncPort_ConfigurePeriodicOutput(&config->outputs[i]);
    if (result != HARD_SYNC_OK) {
      return HARD_SYNC_PORT_ERROR;
    }
  }

  SchM_Enter_HardSync();
  g_config = config;
  g_initialized = true;
  SchM_Exit_HardSync();

  return HARD_SYNC_OK;
}

HardSyncResult HardSync_Start(void) {
  if (!g_initialized || g_config == 0) {
    return HARD_SYNC_NOT_INITIALIZED;
  }

  for (size_t i = 0u; i < g_config->output_count; ++i) {
    const HardSyncOutputConfig *output = &g_config->outputs[i];
    if (!output->enabled) {
      continue;
    }
    HardSyncResult result = HardSyncPort_StartOutput(output->output_id);
    if (result != HARD_SYNC_OK) {
      return HARD_SYNC_PORT_ERROR;
    }
  }

  return HARD_SYNC_OK;
}

HardSyncResult HardSync_Stop(void) {
  if (!g_initialized || g_config == 0) {
    return HARD_SYNC_NOT_INITIALIZED;
  }

  for (size_t i = 0u; i < g_config->output_count; ++i) {
    const HardSyncOutputConfig *output = &g_config->outputs[i];
    if (!output->enabled) {
      continue;
    }
    HardSyncResult result = HardSyncPort_StopOutput(output->output_id);
    if (result != HARD_SYNC_OK) {
      return HARD_SYNC_PORT_ERROR;
    }
  }

  return HARD_SYNC_OK;
}

const HardSyncConfig *HardSync_GetConfig(void) {
  return g_config;
}
