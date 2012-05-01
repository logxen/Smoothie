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

    this->laser_pin.period_us(5);

    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);

    init_member_vars();

    // Initiate main_interrupt timer and step reset timer
    this->kernel->step_ticker->attach( this, &LaserEngrave::stepping_tick );
}

// Get config
void LaserEngrave::on_config_reload(void* argument){
    this->laser_width = this->kernel->config->value(laser_width_checksum)->by_default(0.25)->as_number();
    this->default_engrave_feedrate = this->kernel->config->value(laser_engrave_feedrate_checksum)->by_default(1200)->as_number();
    this->default_engrave_brightness = this->kernel->config->value(laser_engrave_brightness_checksum)->by_default(0.0)->as_number();
    this->default_engrave_contrast = this->kernel->config->value(laser_engrave_contrast_checksum)->by_default(1.0)->as_number();
    this->alpha_steps_per_mm = this->kernel->config->value(alpha_steps_per_mm_checksum)->by_default(1)->as_number();
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
    init_member_vars();

    // Get filename
    this->filename          = shift_parameter( parameters );
    this->file = fopen(this->filename.c_str(), "r");

    this->stream = stream;

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

    // Read fileheader
    if(this->file != NULL) {
        fseek(this->file, 10, SEEK_SET);
        fread(&this->image_array_offset, 4,1,this->file);
        fseek(this->file, 18, SEEK_SET);
        fread(&this->image_width,4,1,this->file);
        fread(&this->image_height,4,1,this->file);
        fseek(this->file, 2, SEEK_CUR);
        fread(&this->image_bpp,2,1,this->file);
        //fseek(this->file, this->image_array_offset, SEEK_SET);
        if(this->image_bpp != 8) {
            fclose(this->file);
            this->file = NULL;
        }
        this->stream->printf("DEBUG: loaded %s, %dx%d pixels, %d bpp\r\n", this->filename.c_str(), this->image_width, this->image_height, this->image_bpp);
    }

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

    this->target_scan_line = floor(this->engrave_y / this->laser_width); // total number of scan lines
    this->steps_per_pixel = floor(this->engrave_x / this->image_width) * alpha_steps_per_mm;
    this->pixels_per_scan_line = double(this->image_height) / double(target_scan_line);
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

    stream->printf("Engraving %s at %f mm/min with %d steps per pixel\r\n", this->filename.c_str(), this->engrave_feedrate, this->steps_per_pixel);
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
    while(this->pixel_queue.size() > 0)
        this->pixel_queue.delete_first();
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
    stream->printf("Engrave completed with %d pixels remaining in the queue\r\n", this->pixel_queue.size());

    // close file
    if(this->file != NULL) {
        fclose(this->file);
        this->file = NULL;
    }
}

void LaserEngrave::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if(this->mode == FOLLOW) {
        scanning = !scanning;
        if(scanning)
        {
            this->step_counter = 0;
            pop_pixel_to_laser();
        }
    }
}

// Set laser power at the beginning of a block
void LaserEngrave::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);
    if(this->mode == FOLLOW && scanning) {
        this->laser_on = true;
        this->set_proportional_power(this->current_power);
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

inline uint32_t LaserEngrave::stepping_tick(uint32_t dummy){
    if( this->paused || this->mode == OFF || !this->laser_on ){ return 0; }

    if(this->current_position != this->kernel->stepper->stepped[ALPHA_STEPPER])
    {
        this->current_position = this->kernel->stepper->stepped[ALPHA_STEPPER];
        this->step_counter++;
        if(this->step_counter >= this->steps_per_pixel) {
            this->step_counter = 0;
            pop_pixel_to_laser();
        }
    }
    return 0;
}

void LaserEngrave::init_member_vars() {
    // Start values
    this->current_scan_line = 0;
    this->current_pixel_col = 0;
    this->current_pixel_row = 0;
    this->target_scan_line = 0;
    this->start_position = 0;
    this->target_position = 0;
    this->current_position = 0;
    this->current_power = 0;
    this->mode = OFF;
    this->scanning = false;
    this->paused = false;
}

void LaserEngrave::fill_pixel_buffer() {
    // find how many pixels it will take to fill the buffer
    int n = this->pixel_queue.capacity() - this->pixel_queue.size();
    if(n > 0 && this->current_scan_line < this->target_scan_line) {
        // fill the buffer
        for(int i=0;i<n;i++) {
            // first push a pixel
            this->pixel_queue.push_back(get_pixel(this->current_pixel_col, this->current_pixel_row));
            // then advance to the next pixel for next pass
            if(this->current_scan_line%2 == 0){
                this->current_pixel_col++;
                if(this->current_pixel_col >= this->image_width) {
                    this->current_pixel_col = this->image_width-1;
                    advance_scan_line();
                }
            } else {
                this->current_pixel_col--;
                if(this->current_pixel_col < 0) {
                    this->current_pixel_col = 0;
                    advance_scan_line();
                }
            }
            if(this->current_scan_line >= this->target_scan_line)
                break;
        }
        //this->stream->printf("DEBUG: added %d pixels to the queue\r\n", n);
    }
}

void LaserEngrave::advance_scan_line() {
    this->current_scan_line++;
    if(floor(this->current_scan_line * this->pixels_per_scan_line) > this->current_pixel_row)
        this->current_pixel_row++;
}

void LaserEngrave::pop_pixel_to_laser() {
    double pixel;
    this->pixel_queue.pop_front(pixel);
    this->current_power = 1 - pixel;
    this->set_proportional_power(this->current_power);
}

#define BLACK_CHECKSUM          62462
#define WHITE_CHECKSUM          33059
#define CHECK_CHECKSUM          62464
#define SIDES_CHECKSUM          21018
#define RAMP_CHECKSUM           14769
#define DOOM_CHECKSUM           11440

double LaserEngrave::get_pixel(int x, int y) {
    double pixel = 1.0;

    if(this->file != NULL) {
        char c;
        int bytes_per_row = (this->image_width * this->image_bpp) / 8;
        bytes_per_row += bytes_per_row % 4 == 0 ? 0 : 4 - bytes_per_row % 4;
        int pixel_offset = y * bytes_per_row + x;
        fseek(this->file, this->image_array_offset + pixel_offset, SEEK_SET);
        fread(&c, 1,1,this->file);
        pixel = double(c) / 255.0;
    } else {
        // Act depending on command
        unsigned short check_sum = get_checksum( this->filename );
        switch( check_sum ){
        case BLACK_CHECKSUM:
            pixel = 0.0;
            break;
        case WHITE_CHECKSUM:
            pixel = 1.0;
            break;
        case CHECK_CHECKSUM:
            if((x+y)%2 == 0) pixel = 0.0;
            else pixel = 1.0;
            break;
        case SIDES_CHECKSUM:
            if(x == 0 || x == 3 || x == this->image_width-1 || x == this->image_width-4)
                pixel = 0.0;
            else pixel = 1.0;
            break;
        case RAMP_CHECKSUM:
            pixel = double(x) / double(this->image_width);
            break;
        case DOOM_CHECKSUM:
            pixel = double( ((x-this->image_width/2)^2) + ((y-this->image_height/2)^2) ) / 1250.0;
            break;
        }
    }

    pixel = max(min(pixel,1.0),0.0);
    if(pixel < 1.0) // a value of white should always be true white
        pixel = this->engrave_brightness + pixel * (1.0-this->engrave_brightness) * this->engrave_contrast;
    // logify!
    pixel *= pixel;
    //this->stream->printf("DEBUG: get_pixel(%d, %d) returned: %f\r\n", x, y, pixel);
    return pixel;
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
