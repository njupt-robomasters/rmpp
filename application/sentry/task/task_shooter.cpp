#include "app.hpp"

extern "C" [[noreturn]] void task_shooter_entry(const void* argument) {
    while (true) {
        if (dj6.is_connected) {
            shooter.SetEnable(true);
            switch (dj6.left_switch) {
            case DJ6::UP:
                shooter.SetPrepareShoot(false);
                shooter.SetShoot(false);
                break;
            case DJ6::MID:
                shooter.SetPrepareShoot(true);
                shooter.SetShoot(false);
                break;
            case DJ6::DOWN:
                shooter.SetPrepareShoot(true);
                shooter.SetShoot(true);
                break;
            }
        } else {
            shooter.SetEnable(false);
        }

        shooter.Update();
        BSP::OS::Delay(1);
    }
}
