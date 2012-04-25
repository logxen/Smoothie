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
    //this->kernel->step_ticker->attach( this, &LaserEngrave::stepping_tick );   
    //this->kernel->step_ticker->reset_attach( this, &LaserEngrave::reset_step_pin );
}

// Get config
void LaserEngrave::on_config_reload(void* argument){
    this->laser_width = this->kernel->config->value(laser_width_checksum)->by_default(0.25)->as_number();
//    this->microseconds_per_step_pulse = this->kernel->config->value(microseconds_per_step_pulse_ckeckusm)->by_default(5)->as_number();
//    this->steps_per_millimeter        = this->kernel->config->value(steps_per_millimeter_checksum       )->by_default(1)->as_number();
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

    // Get other parameters
    Gcode gcode = Gcode();
    gcode.command = parameters;
    gcode.stream = stream;

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

}

// Turn laser off laser at the end of a move
void  LaserEngrave::on_block_end(void* argument){
    this->laser_pin = 0;
}

// Set laser power at the beginning of a block
void LaserEngrave::on_block_begin(void* argument){
    this->set_proportional_power(1.0);
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

// We follow the stepper module here, so speed must be proportional
void LaserEngrave::on_speed_change(void* argument){
    this->set_proportional_power(1.0);
}

void LaserEngrave::set_proportional_power(double rate){
    if( this->laser_on && this->kernel->stepper->current_block ){ 
        this->laser_pin = double(this->kernel->stepper->trapezoid_adjusted_rate)/double(this->kernel->stepper->current_block->nominal_rate * rate);
    }
}
