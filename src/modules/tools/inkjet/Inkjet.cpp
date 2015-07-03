/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Inkjet.h"
#include "Config.h"
#include "libs/nuts_bolts.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"

//#include "libs/Pin.h"
//#include "InterruptIn.h"
//#include "PwmOut.h"
//#include "port_api.h"
//#include "us_ticker_api.h"

#define MAX_CARTRIDGES  6
#define NUM_NOZZLES     12

#define inkjet_checksum                 CHECKSUM("inkjet")
#define enable_checksum                 CHECKSUM("enable")
#define num_cartridges_checksum         CHECKSUM("num_cartridges")
#define cartridge_zero_pin_checksum     CHECKSUM("cartridge_zero_pin")
#define cartridge_one_pin_checksum      CHECKSUM("cartridge_one_pin")
#define cartridge_two_pin_checksum      CHECKSUM("cartridge_two_pin")
#define cartridge_three_pin_checksum    CHECKSUM("cartridge_three_pin")
#define cartridge_four_pin_checksum     CHECKSUM("cartridge_four_pin")
#define cartridge_five_pin_checksum     CHECKSUM("cartridge_five_pin")
#define nozzle_pin_a_checksum           CHECKSUM("nozzle_pin_a")
#define nozzle_pin_b_checksum           CHECKSUM("nozzle_pin_b")
#define nozzle_pin_c_checksum           CHECKSUM("nozzle_pin_c")
#define nozzle_pin_d_checksum           CHECKSUM("nozzle_pin_d")
#define nozzle_on_time_checksum         CHECKSUM("nozzle_on_time")
#define nozzle_off_time_checksum        CHECKSUM("nozzle_off_time")
#define cartridge_off_time_checksum     CHECKSUM("cartridge_off_time")

Inkjet::Inkjet()
{
}

void Inkjet::on_module_loaded()
{
    this->num_cartridges = THEKERNEL->config->value(inkjet_checksum, num_cartridges_checksum)->by_default(1)->as_number();
    if (this->num_cartridges > MAX_CARTRIDGES) this->num_cartridges = MAX_CARTRIDGES;

    if ( (!THEKERNEL->config->value(inkjet_checksum, enable_checksum)->by_default(false)->as_bool()) || (this->num_cartridges < 1) )
    {
      delete this; // Inkjet module is disabled
      return;
    }

    this->cartridge_fire_pins = new Pin*[num_cartridges];
    this->cartridge_timers = new Timer*[num_cartridges];
    for(int i=0;i<num_cartridges;i++)
    {
        this->cartridge_fire_pins[i] = new Pin();
        this->cartridge_timers[i] = new Timer();
        this->cartridge_timers[i]->start();
    }

    this->cartridge_fire_pins[0]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_zero_pin_checksum)->by_default("nc")->as_string())->as_output();
    if (num_cartridges > 1) this->cartridge_fire_pins[1]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_one_pin_checksum)->by_default("nc")->as_string())->as_output();
    if (num_cartridges > 2) this->cartridge_fire_pins[2]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_two_pin_checksum)->by_default("nc")->as_string())->as_output();
    if (num_cartridges > 3) this->cartridge_fire_pins[3]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_three_pin_checksum)->by_default("nc")->as_string())->as_output();
    if (num_cartridges > 4) this->cartridge_fire_pins[4]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_four_pin_checksum)->by_default("nc")->as_string())->as_output();
    if (num_cartridges > 5) this->cartridge_fire_pins[5]->from_string(THEKERNEL->config->value(inkjet_checksum, cartridge_five_pin_checksum)->by_default("nc")->as_string())->as_output();

    this->nozzle_pin_a.from_string(THEKERNEL->config->value(inkjet_checksum, nozzle_pin_a_checksum)->by_default("nc")->as_string())->as_output();
    this->nozzle_pin_b.from_string(THEKERNEL->config->value(inkjet_checksum, nozzle_pin_b_checksum)->by_default("nc")->as_string())->as_output();
    this->nozzle_pin_c.from_string(THEKERNEL->config->value(inkjet_checksum, nozzle_pin_c_checksum)->by_default("nc")->as_string())->as_output();
    this->nozzle_pin_d.from_string(THEKERNEL->config->value(inkjet_checksum, nozzle_pin_d_checksum)->by_default("nc")->as_string())->as_output();

    this->nozzle_on_time = THEKERNEL->config->value(inkjet_checksum, nozzle_on_time_checksum)->by_default(6)->as_number();
    this->nozzle_off_time = THEKERNEL->config->value(inkjet_checksum, nozzle_off_time_checksum)->by_default(800)->as_number();
    this->cartridge_off_time = THEKERNEL->config->value(inkjet_checksum, cartridge_off_time_checksum)->by_default(1)->as_number();
    
    this->active_cartridge = 0;

    register_for_event(ON_GCODE_RECEIVED);
}

void Inkjet::on_gcode_received(void* argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    
    if (gcode->has_m)
    {
        if (gcode->m == 700) // fire inkjet cartridge
        {
            // M700: wait for queue to empty and then fire inkjet
            gcode->mark_as_taken();
            THEKERNEL->conveyor->wait_for_empty_queue();

            if(gcode->has_letter('P'))
            {
                // set new active cartridge
                int new_cartridge = gcode->get_value('P');
                if (new_cartridge >= 0 && new_cartridge < this->num_cartridges)
                {
                    this->active_cartridge = new_cartridge;
                }
            }

            if(gcode->has_letter('S'))
            {
                // fire active cartridge
                uint32_t data = gcode->get_value('S');

                while (this->cartridge_timers[this->active_cartridge]->read_us() < this->nozzle_off_time)
                    ;
                this->cartridge_timers[this->active_cartridge]->reset();

                for (int i=0;i<NUM_NOZZLES;i++)
                {
                    if (data & 1<<i)
                    {
                        // select nozzle
                        this->nozzle_pin_a.set(i & 1<<0);
                        this->nozzle_pin_b.set(i & 1<<1);
                        this->nozzle_pin_c.set(i & 1<<2);
                        this->nozzle_pin_d.set(i & 1<<3);

                        // fire!
                        this->cartridge_fire_pins[this->active_cartridge]->set(1);
                        wait_us(this->nozzle_on_time);
                        this->cartridge_fire_pins[this->active_cartridge]->set(0);
                        wait_us(this->cartridge_off_time);
                    }
                }

            }
        }
    }
}

