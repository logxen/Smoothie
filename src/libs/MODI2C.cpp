#include "MODI2C.h"
#include "cmsis.h"
#include "pinmap.h"
 
static const PinMap PinMap_I2C_SDA[] = {
    {P0_0 , I2C_1, 3},
    {P0_10, I2C_2, 2},
    {P0_19, I2C_1, 3},
    {P0_27, I2C_0, 1},
    {NC   , NC   , 0}
};

static const PinMap PinMap_I2C_SCL[] = {
    {P0_1 , I2C_1, 3},
    {P0_11, I2C_2, 2},
    {P0_20, I2C_1, 3},
    {P0_28, I2C_0, 1},
    {NC   , NC,    0}
};

MODI2C::I2CBuffer MODI2C::Buffer0 = {0,0};        //Sets the initial buffer empty and count on zero
MODI2C::I2CBuffer MODI2C::Buffer1 = {0,0};        //Sets the initial buffer empty
MODI2C::I2CBuffer MODI2C::Buffer2 = {0,0};        //Sets the initial buffer empty
//int MODI2C::status=0;
int MODI2C::defaultStatus=0;
 
 
 
 
MODI2C::MODI2C(PinName sda, PinName scl) {
    // determine the I2C to use
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    this->I2CMODULE = (LPC_I2C_TypeDef *)pinmap_merge(i2c_sda, i2c_scl);
    
    if ((int)this->I2CMODULE == NC) {
        error("I2C pin mapping failed");
    }

    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);

//    led = LED3;
    //Check which connection we are using, if not correct, go to error status
//    if ((sda==p9) && (scl==p10))
//        I2CMODULE = LPC_I2C1;
//    else if ((sda==p28) && (scl==p27))
//        I2CMODULE = LPC_I2C2;
//    else
//        error("MODI2C pins not valid");
    switch ((int)this->I2CMODULE) {
        case I2C_0: LPC_SC->PCONP |= 1 << 7; break;
        case I2C_1: LPC_SC->PCONP |= 1 << 19; break;
        case I2C_2: LPC_SC->PCONP |= 1 << 26; break;
    }
 
    //Default settings:
    frequency(100000);
 
    writePinState();
}
 
int MODI2C::write(int address, char *data, int length, bool repeated, int *status) {
 
    I2CData Data;
    //Store relevant information
    address &= 0xFE;
    Data.caller = this;
    Data.address = address;
    Data.repeated = repeated;
    Data.data = data;
    Data.length = length;
    Data.status = status;
 
    while(!addBuffer(Data, I2CMODULE));
 
    return 0;
}
 
int MODI2C::write(int address, char *data, int length, int *status) {
    return write(address, data, length, false, status);
    }
 
int MODI2C::read_nb(int address, char *data, int length, bool repeated, int *status) {
    //Store relevant information
    address |= 0x01;
 
    //isIdle check here
    I2CData Data;
 
 
    Data.caller = this;
    Data.address = address;
    Data.repeated = repeated;
    Data.data = data;
    Data.length = length;
    Data.status = status;
 
    while(!addBuffer(Data, I2CMODULE));
 
    return 0;
}
 
int MODI2C::read_nb(int address, char *data, int length, int *status) {
    return read_nb(address, data, length, false, status);
    }
 
int MODI2C::read(int address, char *data, int length, bool repeated) {
    int stat;
    //Store relevant information
    address |= 0x01;
 
    //isIdle check here
    I2CData Data;
 
 
    Data.caller = this;
    Data.address = address;
    Data.repeated = repeated;
    Data.data = data;
    Data.length = length;
    Data.status = &stat;
 
    while(!addBuffer(Data, I2CMODULE));
    
    I2CBuffer *Buffer;
    switch ((int)this->I2CMODULE) {
        case I2C_0: Buffer = &Buffer0; break;
        case I2C_1: Buffer = &Buffer1; break;
        case I2C_2: Buffer = &Buffer2; break;
    }
/*
    if (I2CMODULE == LPC_I2C1) {
        Buffer = &Buffer1;
    } else {
        Buffer = &Buffer2;
    }
*/
    
    while(Buffer->queue!=0)
        wait_us(1);
    
    if (stat==0x58)         //Return zero if ended correctly, otherwise return return code.
        return 0;
    else
        return stat;
}
 
 
void MODI2C::start( void ) {
    _start(I2CMODULE);
}
 
void MODI2C::stop( void ) {
    _stop(I2CMODULE);
}
 
void MODI2C::frequency(int hz) {
    //The I2C clock by default runs on quarter of system clock, which is 96MHz
    //So to calculate high/low count times, we do 96MHz/4/2/frequency
    duty = SystemCoreClock/8/hz;
    if (duty>65535)
        duty=65535;
    if (duty<4)
        duty=4;
}
 
int MODI2C::getQueue( void ) {
    I2CBuffer *Buffer;
    switch ((int)this->I2CMODULE) {
        case I2C_0: Buffer = &Buffer0; break;
        case I2C_1: Buffer = &Buffer1; break;
        case I2C_2: Buffer = &Buffer2; break;
    }
/*
    if (I2CMODULE == LPC_I2C1) {
        Buffer = &Buffer1;
    } else {
        Buffer = &Buffer2;
    }
*/
    return Buffer->queue;
    }
 
 
 
//*******************************************
//***********Internal functions**************
//*******************************************
 
 
void MODI2C::writeSettings( void ) {
    I2CMODULE->I2CONSET = 1<<I2C_ENABLE;     //Enable I2C
    I2CMODULE->I2CONCLR = I2C_STOP;
    I2CMODULE->MMCTRL = 0;                   //Disable monitor mode
    I2CMODULE->I2SCLH = duty;
    I2CMODULE->I2SCLL = duty;
 
}
 
void MODI2C::writePinState( void ) {
    switch ((int)this->I2CMODULE) {
        case I2C_0:
            LPC_PINCON->PINSEL1 |= (1<<22)|(1<<24); //Same story, different register settings
            break;
        case I2C_1:
            LPC_PINCON->PINSEL0 |= 0x0000000F;       //Sets pins as I2C
            LPC_PINCON->PINMODE0 |= 0x0000000A;      //Neither pull up nor pull down
            LPC_PINCON->PINMODE_OD0 |= 0x00000003;   //Open drain mode enabled
            break;
        case I2C_2:
            LPC_PINCON->PINSEL0 |= (1<<21)|(1<<23); //Same story, different register settings
            LPC_PINCON->PINMODE0 |= (1<<21)|(1<<23);
            LPC_PINCON->PINMODE_OD0 |= (1<<10)|(1<<11);
            break;
    }
/*
    if (I2CMODULE == LPC_I2C1) {
        LPC_PINCON->PINSEL0 |= 0x0000000F;       //Sets pins as I2C
        LPC_PINCON->PINMODE0 |= 0x0000000A;      //Neither pull up nor pull down
        LPC_PINCON->PINMODE_OD0 |= 0x00000003;   //Open drain mode enabled
    } else if (I2CMODULE == LPC_I2C2) {
        LPC_PINCON->PINSEL0 |= (1<<21)|(1<<23); //Same story, different register settings
        LPC_PINCON->PINMODE0 |= (1<<21)|(1<<23);
        LPC_PINCON->PINMODE_OD0 |= (1<<10)|(1<<11);
    }
*/
}
 
 
 
void MODI2C::attach( void (*function)(void), int operation) {
    IRQOp = operation;
    callback.attach(function);
}
 
template<typename T>
void MODI2C::attach(T *object, void (T::*member)(void), int operation) {
    IRQOp = operation;
    callback.attach(object, member);
}
 
void MODI2C::detach( void ) {
    callback.attach(NULL);
}
 
void MODI2C::_start(LPC_I2C_TypeDef *I2CMODULE) {
    if (!(I2CMODULE->I2CONSET & 1<<I2C_START))  //If already sent, skip
        I2CMODULE->I2CONSET = 1<<I2C_START;     //Send start condition
}
 
void MODI2C::_stop(LPC_I2C_TypeDef *I2CMODULE) {
    I2CMODULE->I2CONSET = 1<<I2C_STOP;      //Send stop condition
    I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
}
 
//Set interrupt vector
void MODI2C::setISR(void) {
    _setISR(I2CMODULE);
}
 
void MODI2C::_setISR(LPC_I2C_TypeDef *I2CMODULE) {
    switch ((int)I2CMODULE) {
        case I2C_0: 
            NVIC_SetVector(I2C0_IRQn, (uint32_t)&IRQ0Handler);
            NVIC_EnableIRQ(I2C0_IRQn);
            break;
        case I2C_1: 
            NVIC_SetVector(I2C1_IRQn, (uint32_t)&IRQ1Handler);
            NVIC_EnableIRQ(I2C1_IRQn);
            break;
        case I2C_2: 
            NVIC_SetVector(I2C2_IRQn, (uint32_t)&IRQ2Handler);
            NVIC_EnableIRQ(I2C2_IRQn);
            break;
    }
/*
    if (I2CMODULE == LPC_I2C1) {
        NVIC_SetVector(I2C1_IRQn, (uint32_t)&IRQ1Handler);
        NVIC_EnableIRQ(I2C1_IRQn);
    } else if (I2CMODULE == LPC_I2C2) {
        NVIC_SetVector(I2C2_IRQn, (uint32_t)&IRQ2Handler);
        NVIC_EnableIRQ(I2C2_IRQn);
    }
*/
}
 
void MODI2C::clearISR( void ) {
    _clearISR(I2CMODULE);
}
 
void MODI2C::_clearISR( LPC_I2C_TypeDef *I2CMODULE ) {
    switch ((int)I2CMODULE) {
        case I2C_0: NVIC_DisableIRQ(I2C0_IRQn); break;
        case I2C_1: NVIC_DisableIRQ(I2C1_IRQn); break;
        case I2C_2: NVIC_DisableIRQ(I2C2_IRQn); break;
    }
/*
    if (I2CMODULE == LPC_I2C1) {
        NVIC_DisableIRQ(I2C1_IRQn);
    } else if (I2CMODULE == LPC_I2C2) {
        NVIC_DisableIRQ(I2C2_IRQn);
    }
*/
}

