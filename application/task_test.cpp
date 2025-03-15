#include "task_test.h"
#include "cmsis_os.h"

[[noreturn]] void task_test_entry(void const * argument) {
    while (true) {
        osDelay(1000);
    }
}
