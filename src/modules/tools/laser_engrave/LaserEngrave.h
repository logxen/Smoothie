/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LASER_ENGRAVE_MODULE_H
#define LASER_ENGRAVE_MODULE_H

#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"

#define laser_engrave_module_enable_checksum 20
#define laser_engrave_command_checksum       41059
#define laser_width_checksum                 34714
#define laser_engrave_feedrate_checksum      1286
#define laser_engrave_brightness_checksum    63488
#define laser_engrave_contrast_checksum      50740
#define steps_per_millimeter_checksum        58088

#define OFF 0
#define SOLO 1
#define FOLLOW 2

class LaserEngrave : public Module{
    public:
        LaserEngrave(PwmOut& pin);
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_console_line_received( void* argument );
        void laser_engrave_command( string parameters, StreamOutput* stream );
        void on_block_end(void* argument);
        void on_block_begin(void* argument);
        void on_play(void* argument);
        void on_pause(void* argument);
        uint32_t stepping_tick(uint32_t dummy);
//        uint32_t reset_step_pin(uint32_t dummy);

        unsigned short get_pixel(unsigned short x, unsigned short y);
        void send_gcode(Gcode* gcode);
        void on_speed_change(void* argument);
        void set_proportional_power(double rate);

        PwmOut laser_pin;    // PWM output to regulate the laser power
        bool   laser_on;     // Laser status

        double          laser_width;
        double          default_engrave_feedrate;
        double          default_engrave_brightness;
        double          default_engrave_contrast;

        unsigned short  current_scan_line;
        unsigned short  current_pixel_row;
        unsigned short  current_pixel_col;
        RingBuffer<unsigned short, 32> pixel_queue;
        unsigned short  target_scan_line;
        double          start_position;               // Start point ( in steps ) for the current move
        double          target_position;              // End point ( in steps ) for the current move
        double          current_position;             // Current point ( in steps ) for the current move, incremented every time a step is outputed
        double          current_power;                // Current value for pwm control
        Block*          current_block;                // Current block we are stepping, same as Stepper's one

        double          engrave_x;
        double          engrave_y;
        double          engrave_feedrate;
        double          engrave_brightness;
        double          engrave_contrast;

        int             step_counter;
        int             counter_increment;
        int             steps_per_millimeter;

        char mode;

        bool paused;
};

#endif
