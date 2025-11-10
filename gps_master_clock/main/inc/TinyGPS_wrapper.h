#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

bool TinyGPS_wrapper_encode(char c);
  void crack_datetime(int *year, uint8_t *month, uint8_t *day, 
    uint8_t *hour, uint8_t *minute, uint8_t *second, uint8_t *hundredths, unsigned long *fix_age);
#ifdef __cplusplus
}
#endif