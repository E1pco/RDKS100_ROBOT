#include "hard_sync.h"
#include "rdks100_hard_sync_config.h"

int main(void) {
  HardSyncResult result = HardSync_Init(&g_rdks100_hard_sync_config);
  if (result != HARD_SYNC_OK) {
    return (int)result;
  }

  result = HardSync_Start();
  if (result != HARD_SYNC_OK) {
    return (int)result;
  }

  return 0;
}
