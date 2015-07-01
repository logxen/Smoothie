/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INKJET_MODULE_H
#define INKJET_MODULE_H

#include "libs/Module.h"
#include <stdint.h>

#include "Pin.h"
#include "mbed.h"

// This module implements control for TIJ inkjet heads such as the HP C6602 and the HP 51604
class Inkjet: public Module {
    public:
        Inkjet();
        virtual ~Inkjet() {};
        void on_module_loaded();
        
        
    private:
        void on_gcode_received(void *argument);

        int     num_cartridges;
        int     active_cartridge;
        Timer   **cartridge_timers;
        Pin     **cartridge_fire_pins;
        Pin     nozzle_pin_a;
        Pin     nozzle_pin_b;
        Pin     nozzle_pin_c;
        Pin     nozzle_pin_d;

        int     nozzle_on_time;
        int     nozzle_off_time;
        int     cartridge_off_time;
};

#endif // INKJET_MODULE_H

