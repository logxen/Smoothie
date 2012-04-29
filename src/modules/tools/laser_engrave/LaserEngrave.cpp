/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

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
    this->mode = OFF;
    this->scanning = false;
    this->paused = false;

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
    this->default_engrave_brightness = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(0.0)->as_number();
    this->default_engrave_contrast = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(1.0)->as_number();
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
    this->filename          = shift_parameter( parameters );
    this->stream = stream;

    // Read fileheader
    // Get other parameters
    Gcode gcode = Gcode();
    gcode.command = parameters;
    gcode.stream = stream;

    // ** fake fileheader system **
    this->image_width = 10;
    if(gcode.has_letter('X'))
        this->image_width = gcode.get_value('X');
    this->image_height = 10;
    if(gcode.has_letter('Y'))
        this->image_height = gcode.get_value('Y');
    this->image_bpp = 8; // not yet used

    if(gcode.has_letter('I')) {
        engrave_x = gcode.get_value('I');
    } else {
        engrave_x = this->image_width;
    }
    if(gcode.has_letter('J')) {
        engrave_y = gcode.get_value('J');
    } else {
        engrave_y = this->image_height;
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

    target_scan_line = floor(engrave_y / laser_width); // nub of scan liness
    this->steps_per_pixel = (engrave_x / this->image_width) * steps_per_millimeter;
    char buffer[16];
    sprintf(buffer, " F%f", engrave_feedrate);
    string feedrate(buffer);
    sprintf(buffer, "G0 X%f", engrave_x);
    string g_scan_forward(buffer);
    sprintf(buffer, "G0 X%f", engrave_x * -1);
    string g_scan_back(buffer);
    sprintf(buffer, "G0 Y%f", engrave_y * -1);
    string g_scan_y_forward = (buffer);
    sprintf(buffer, "G0 Y%f", engrave_y);
    string g_scan_y_back = (buffer);
    sprintf(buffer, "G0 Y%f", copysign(laser_width, engrave_y * -1));
    string g_advance_line = (buffer);

    while(this->kernel->player->queue.size() > 0) { wait_us(500); } // wait for the queue to empty

    stream->printf("Engraving %s at %f mm/min\r\n", filename.c_str(), engrave_feedrate);
    // begin by setting the machine into relative mode
    //TODO: need to cache current mode
    send_gcode("G91\r\n", stream);
    // trace a box around the area to be engraved with the laser off (G0) for professionalism
    send_gcode(g_scan_forward + "\r\n", stream);
    send_gcode(g_scan_y_forward + "\r\n", stream);
    send_gcode(g_scan_back + "\r\n", stream);
    send_gcode(g_scan_y_back + "\r\n", stream);

    while(this->kernel->player->queue.size() > 0) { wait_us(500); } // wait for the queue to empty

    // begin engraving
    current_pixel_row = 0;
    current_pixel_col = 0;
    this->mode = FOLLOW;
    this->scanning = false;
    for (int sl=0;sl<target_scan_line;sl++) {
        // fill the pixel buffer at least once per pass
        do { fill_pixel_buffer(); wait_us(500); } 
        // if there is room in the queue break from the buffer fill loop to add some gcodes to the queue
        while(this->kernel->player->queue.size() >= this->kernel->player->queue.capacity()-3);
        // send the gcodes for a scanline
        send_gcode(((sl % 2) == 0 ? g_scan_forward : g_scan_back) + feedrate + "\r\n", stream);
        send_gcode(g_advance_line + "\r\n", stream);
    }

    // keep the buffer full until the queue is empty
    while(this->kernel->player->queue.size() > 0) { fill_pixel_buffer(); wait_us(500); }
    this->mode = OFF;

    // return the toolhead to original location
    if(target_scan_line % 2 != 0) { send_gcode(g_scan_back + "\r\n", stream); }
    send_gcode(g_scan_y_back + "\r\n", stream);

    // return the machine to previous settings
    //TODO: actually check what old mode was instead of assuming absolute
    send_gcode("G90\r\n", stream);
    stream->printf("Engrave completed\r\n");
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
        scanning = !scanning;
        if(scanning) {
            this->stream->printf("DEBUG: Beginning scan block\r\n");
            this->current_position = this->kernel->stepper->stepped[ALPHA_STEPPER];
            this->laser_on = true;
            //this->set_proportional_power(this->current_power);
        }
    }
}

// Turn laser off laser at the end of a move
void  LaserEngrave::on_block_end(void* argument){
    this->laser_on = false;
    this->laser_pin = 0;
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
    if( this->paused || this->mode == OFF || !this->laser_on ){ return 0; }

    if(this->kernel->stepper->stepped[ALPHA_STEPPER] - this->current_position > this->steps_per_pixel) {
        this->current_position += this->steps_per_pixel;
        double pixel;
        this->pixel_queue.pop_front(pixel);
        this->current_power = 1 - pixel;
        this->set_proportional_power(this->current_power);
    }
    return 0;
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
/*
uint32_t LaserEngrave::reset_step_pin(uint32_t dummy){
    this->step_pin = 0;
}
*/

void LaserEngrave::fill_pixel_buffer() {
            int n = this->pixel_queue.capacity() - this->pixel_queue.size();
            if(n > 0 && current_pixel_row < this->image_height) {
                for(int i=0;i<n;i++) {
                    if(current_pixel_row%2 == 0){
                        if(current_pixel_col >= this->image_width) {
                            current_pixel_col = this->image_width-1;
                            current_pixel_row++;
                            if(current_pixel_row >= this->image_height)
                                break;
                        }
                        pixel_queue.push_back(get_pixel(current_pixel_col, current_pixel_row));
                        current_pixel_col++;
                    } else {
                        if(current_pixel_col < 0) {
                            current_pixel_col = 0;
                            current_pixel_row++;
                            if(current_pixel_row >= this->image_height)
                                break;
                        }
                        pixel_queue.push_back(get_pixel(current_pixel_col, current_pixel_row));
                        current_pixel_col--;
                    }
                }
                this->stream->printf("DEBUG: added %d pixels to the queue\r\n", n);
            }
}

#define BLACK_CHECKSUM          62462
#define WHITE_CHECKSUM          33059
#define CHECK_CHECKSUM          62464
#define SIDES_CHECKSUM          21018
#define RAMP_CHECKSUM           14769
#define DOOM_CHECKSUM           11440

double LaserEngrave::get_pixel(int x, int y) {
    unsigned short check_sum = get_checksum( this->filename );

    // Act depending on command
    switch( check_sum ){
    case BLACK_CHECKSUM:
        return 0.0;
        break;
    case WHITE_CHECKSUM:
        return 1.0;
        break;
    case CHECK_CHECKSUM:
        if((x+y)%2 == 0) return 0.0;
        else return 1.0;
        break;
    case SIDES_CHECKSUM:
        if(x == 0 || x == 3 || x == this->image_width-1 || x == this->image_width-4
                || y == 0 || y == 3 || y == this->image_height-1 || y == this->image_height-4)
            return 0.0;
        else return 1.0;
        break;
    case RAMP_CHECKSUM:
        return double(x / this->image_width);
        break;
    case DOOM_CHECKSUM:
        double pixel = 1.0/((x-this->image_width/2)^2+(y-this->image_height/2)^2);
        pixel = this->engrave_brightness + pixel * this->engrave_contrast;
        return max(min(pixel,1.0),0.0);
        break;
    }
}

void LaserEngrave::send_gcode(string msg, StreamOutput* stream) {
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}

// We follow the stepper module here, so speed must be proportional
void LaserEngrave::on_speed_change(void* argument){
    if(this->laser_on) {
        this->set_proportional_power(this->current_power);
    }
}

void LaserEngrave::set_proportional_power(double rate){
    if( this->laser_on && this->kernel->stepper->current_block ){
        this->laser_pin = (double(this->kernel->stepper->trapezoid_adjusted_rate)/double(this->kernel->stepper->current_block->nominal_rate)) * rate;
    }
}
