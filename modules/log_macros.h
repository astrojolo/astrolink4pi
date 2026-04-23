#include <indilogger.h>

#define DEBUGFDEVICE_LOG_ONCE(flag, dev, level, fmt, ...) \
    do { \
        if (!(flag)) { \
            DEBUGFDEVICE(dev, level, fmt, ##__VA_ARGS__); \
            (flag) = true; \
        } \
    } while (0)