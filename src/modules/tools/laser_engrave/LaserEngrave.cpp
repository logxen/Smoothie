/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <sstream>
#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Stepper.h"
#include "LaserEngrave.h"
#include "modules/robot/Player.h"
#include "libs/SerialMessage.h"
#include "libs/nuts_bolts.h"

LaserEngrave::LaserEngrave(PwmOut& pin) : laser_pin(pin) {
}

void LaserEngrave::on_module_loaded() {
    if( !this->kernel->config->value( laser_engrave_module_enable_checksum )->by_default(false)->as_bool() ){ return; }

    this->on_config_reload(this);

    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
//    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);

    // Start values
    this->current_scan_line = 0;
    this->target_scan_line = 0;
    this->start_position = 0;
    this->target_position = 0;
    this->current_position = 0;
    this->current_power = 0;
    this->current_block = NULL;
    this->mode = OFF;

    // Update speed every *acceleration_ticks_per_second*
    // TODO: Make this an independent setting
    //this->kernel->slow_ticker->attach( this->kernel->stepper->acceleration_ticks_per_second , this, &LaserEngrave::acceleration_tick );

    // Initiate main_interrupt timer and step reset timer
    this->kernel->step_ticker->attach( this, &LaserEngrave::stepping_tick );
//    this->kernel->step_ticker->reset_attach( this, &LaserEngrave::reset_step_pin );
}

// Get config
void LaserEngrave::on_config_reload(void* argument){
    this->laser_width = this->kernel->config->value(laser_width_checksum)->by_default(0.25)->as_number();
    this->default_engrave_feedrate = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(1200)->as_number();
    this->default_engrave_brightness = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(0)->as_number();
    this->default_engrave_contrast = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(1)->as_number();
//    this->microseconds_per_step_pulse = this->kernel->config->value(microseconds_per_step_pulse_ckeckusm)->by_default(5)->as_number();
    this->steps_per_millimeter        = this->kernel->config->value(steps_per_millimeter_checksum       )->by_default(1)->as_number();
//    this->feed_rate                   = this->kernel->config->value(default_feed_rate_checksum          )->by_default(1)->as_number();
//    this->acceleration                = this->kernel->config->value(acceleration_checksum               )->by_default(1)->as_number();
}

// When a new line is received, check if it is a command, and if it is, act upon it
void LaserEngrave::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    switch( check_sum ){
        case laser_engrave_command_checksum: this->laser_engrave_command(  get_arguments(possible_command), new_message.stream ); break;
    }
}

void LaserEngrave::laser_engrave_command( string parameters, StreamOutput* stream ){
    // Get filename
    string filename          = shift_parameter( parameters );

    // Read fileheader
    // ** fake fileheader system **
    unsigned short image_width = 10;
    unsigned short image_height = 10;
    unsigned short image_bpp = 8;

    // Get other parameters
    Gcode gcode = Gcode();
    gcode.command = parameters;
    gcode.stream = stream;

    if(gcode.has_letter('I')) {
        engrave_x = gcode.get_value('I');
    } else {
        engrave_x = image_width;
    }
    if(gcode.has_letter('J')) {
        engrave_y = gcode.get_value('J');
    } else {
        engrave_y = image_height;
    }
    if(gcode.has_letter('F')) {
        engrave_feedrate = gcode.get_value('F');
    } else {
        engrave_feedrate = default_engrave_feedrate; // if feedrate not specified, use value from config
    }
    if(gcode.has_letter('B')) { // Image black level
        engrave_brightness = gcode.get_value('B');
    } else {
        engrave_brightness = default_engrave_brightness;
    }
    if(gcode.has_letter('C')) { // Image gain
        engrave_contrast = gcode.get_value('C');
    } else {
        engrave_contrast = default_engrave_contrast;
    }

    target_scan_line = floor(engrave_y / laser_width);
    double ppsl = target_scan_line / image_height; // num of y pixels per scan line
    this->steps_per_pixel = (image_width / engrave_x) * steps_per_millimeter;
    stringstream ss;
    ss.str(" F"); ss << engrave_feedrate;
    string feedrate = ss.str();
    ss.str("G0 Y"); ss << engrave_y << feedrate;
    string g_scan_forward = ss.str();
    ss.str("G0 Y"); ss << engrave_y * -1 << feedrate;
    string g_scan_back = ss.str();
    ss.str("G0 X"); ss << engrave_x << feedrate;
    string g_scan_x_forward = ss.str();
    ss.str("G0 X"); ss << engrave_x * -1 << feedrate;
    string g_scan_x_back = ss.str();
    ss.str("G0 X"); ss << copysign(engrave_x,laser_width);
    string g_advance_line;

    while(this->kernel->player->queue.size() > 0) { wait_us(500); } // wait for the queue to empty

    stream->printf("Engraving %s at %f mm/min", filename.c_str(), engrave_feedrate);
    // begin by setting the machine into relative mode
    //TODO: need to cache current mode
    send_gcode(new Gcode("G91") );
    // trace a box around the area to be engraved with the laser off (G0) for professionalism
    send_gcode(new Gcode(g_scan_forward) );
    send_gcode(new Gcode(g_scan_x_forward) );
    send_gcode(new Gcode(g_scan_back) );
    send_gcode(new Gcode(g_scan_x_back) );

    while(this->kernel->player->queue.size() > 0) { wait_us(500); } // wait for the queue to empty

    // begin engraving
    current_pixel_row = 0;
    current_pixel_col = 0;
    for (int sl=0;sl<target_scan_line;sl++) {
        do {
            int n = this->pixel_queue.capacity() - this->pixel_queue.size();
            if(n > 0 && current_pixel_row < image_height) {
                for(int i=0;i<n;i++) {
                    if(current_pixel_row%2 == 0){
                        if(current_pixel_col >= image_width) {
                            current_pixel_col = 0;
                            current_pixel_row++;
                            if(current_pixel_row >= image_height)
                                break;
                        }
                        pixel_queue.push_back(get_pixel(current_pixel_col, current_pixel_row));
                        current_pixel_col++;
                    } else {
                        if(current_pixel_col < 0) {
                            current_pixel_col = image_width-1;
                            current_pixel_row++;
                            if(current_pixel_row >= image_height)
                                break;
                        }
                        pixel_queue.push_back(get_pixel(current_pixel_col, current_pixel_row));
                        current_pixel_col--;
                    }
                }
            }
        }
        while(this->kernel->player->queue.size() >= this->kernel->player->queue.capacity()-3);
        //current_scan_line = sl;
        //current_pixel_row = floor(sl * ppsl);
        send_gcode(new Gcode( (sl % 2) == 0 ? g_scan_forward : g_scan_back) );
        send_gcode(new Gcode(g_advance_line) );
    }

    // return the toolhead to original location
    if(target_scan_line % 2 != 0) { send_gcode(new Gcode(g_scan_back) ); }
    send_gcode(new Gcode(g_scan_x_back) );

    // return the machine to previous settings
    //TODO: actually check what old mode was instead of assuming absolute
    send_gcode(new Gcode("G90") );
    stream->printf("Engrave completed");
/*
    // Open file
    FILE *lp = fopen(filename.c_str(), "r");
    string buffer;
    int c;

    // Print each line of the file
    while ((c = fgetc (lp)) != EOF){
        if (c == '\n'){
            stream->printf("%s\n", buffer.c_str());
            struct SerialMessage message;
            message.message = buffer;
            message.stream = stream;
            this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            buffer.clear();
        }else{
            buffer += c;
        }
    };

    fclose(lp);
*/
}

// Set laser power at the beginning of a block
void LaserEngrave::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);
    if(this->mode == FOLLOW) {
        this->current_block = block;
        this->set_proportional_power(this->current_power);
    }
}

// Turn laser off laser at the end of a move
void  LaserEngrave::on_block_end(void* argument){
    this->laser_pin = 0;
    this->current_block = NULL;
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void LaserEngrave::on_pause(void* argument){
    this->paused = true;
    this->laser_pin = 0;
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void LaserEngrave::on_play(void* argument){
    this->paused = false;
    this->set_proportional_power(this->current_power);
}

/*
// Turn laser on/off depending on received GCodes
void LaserEngrave::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    this->laser_on = false;
    if( gcode->has_letter('G' )){
        int code = gcode->get_value('G');
        if( code == 0 ){                    // G0
            this->laser_pin = 0;
            this->laser_on =  false;
        }else if( code >= 1 && code <= 3 ){ // G1, G2, G3
            this->laser_on =  true;
        }
    }
}
*/

inline uint32_t LaserEngrave::stepping_tick(uint32_t dummy){
    if( this->paused ){ return 0; }

    this->step_counter++;
    if( this->step_counter > 1<<16 ){
        this->step_counter -= 1<<16;
    if(this->step_counter - this->current_position > this->steps_per_pixel) {
        this->current_position += this->steps_per_pixel;
        double pixel;
        this->pixel_queue.pop_front(pixel);
        this->current_power = this->engrave_brightness + pixel * this->engrave_contrast;
        this->set_proportional_power(this->current_power);
    }
/*
        // If we still have steps to do 
        // TODO: Step using the same timer as the robot, and count steps instead of absolute float position 
        if( ( this->current_position < this->target_position && this->direction == 1 ) || ( this->current_position > this->target_position && this->direction == -1 ) ){    
            this->current_position += (double(double(1)/double(this->steps_per_millimeter)))*double(this->direction);
            this->dir_pin = ((this->direction > 0) ? 1 : 0);
            this->step_pin = 1;
        }else{
            // Move finished
            if( this->mode == SOLO && this->current_block != NULL ){
                // In follow mode, the robot takes and releases the block, in solo mode we do
                this->current_block->release();        
            } 
        }
*/
    }
}
/*
uint32_t LaserEngrave::reset_step_pin(uint32_t dummy){
    this->step_pin = 0;
}
*/

double LaserEngrave::get_pixel(int x, int y) {
    //TODO: Implement some more impressive bitmap than 'the black bears in the black forest during a snowstorm at night under a new moon'
    return 0;
}

void LaserEngrave::send_gcode(Gcode* gcode) {
    this->kernel->call_event(ON_GCODE_RECEIVED, gcode );
}

// We follow the stepper module here, so speed must be proportional
void LaserEngrave::on_speed_change(void* argument){
    this->set_proportional_power(this->current_power);
}

void LaserEngrave::set_proportional_power(double rate){
    if( this->laser_on && this->kernel->stepper->current_block ){
        this->laser_pin = double(this->kernel->stepper->trapezoid_adjusted_rate)/double(this->kernel->stepper->current_block->nominal_rate * rate);
    }
}
