#include "Radio.h"
#include "nRF24L01P_defs.h"



Radio::Radio(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq) 
: _spi(mosi, miso, sck), _csn(csn), _ce(ce), _irq(irq)
{
    // Disable nRF24L01+
    _ce = 0;
    
    // Disable chip select
    _csn = 1;
    
    // Set up SPI
    _spi.frequency(SPI_FREQUENCY);
    _spi.format(8, 0);
    
    // Set to use controller channel 0
    controller = 0;
    
    // Set up IRQ
    _irq.fall(this, &Radio::receive);
}



void Radio::reset()
{
    // Wait for power on reset
    wait_us(TIMING_Tpor);

    // Put into standby
    _ce = 0;
    
    // Configure registers
    setRegister(CONFIG, CONFIG_MASK_TX_DS | CONFIG_MASK_MAX_RT | CONFIG_EN_CRC | CONFIG_PWR_UP | CONFIG_PRIM_RX);
    setRegister(EN_AA, 0x00);
    setRegister(EN_RXADDR, ERX_P0 | ERX_P1);
    setRegister(SETUP_AW, SETUP_AW_3BYTES);
    setRegister(SETUP_RETR, 0x00);
    setRegister(RF_CH, RF_CHANNEL);
    setRegister(RF_SETUP, RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_PWR_0);
    setRegister(STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    setRegister(RX_PW_P0, 4);
    setRegister(RX_PW_P1, 4);
    setRegister(DYNPD, 0x00);
    setRegister(FEATURE, 0x00);
    
    // Set addresses
    _csn = 0;
    _spi.write(W_REGISTER | RX_ADDR_P0);
    _spi.write(CTRL_BASE_ADDRESS_1 + (controller & 0xf));
    _spi.write(CTRL_BASE_ADDRESS_2);
    _spi.write(CTRL_BASE_ADDRESS_3);
    _csn = 1;
    _csn = 0;
    _spi.write(W_REGISTER | RX_ADDR_P1);
    _spi.write(ROBOT_ADDRESS_1);
    _spi.write(ROBOT_ADDRESS_2);
    _spi.write(ROBOT_ADDRESS_3);
    _csn = 1;
    _csn = 0;
    _spi.write(W_REGISTER | TX_ADDR);
    _spi.write(ROBOT_ADDRESS_1);
    _spi.write(ROBOT_ADDRESS_2);
    _spi.write(ROBOT_ADDRESS_3);
    _csn = 1;
    
    // Put into PRX
    _ce = 1;
    wait_us(TIMING_Tstby2a);
    
    // Flush FIFOs
    _csn = 0;
    _spi.write(FLUSH_TX);
    _csn = 1;
    _csn = 0;
    _spi.write(FLUSH_RX);
    _csn = 1;
}



void Radio::transmit(uint32_t data)
{
    // Put into standby
    _ce = 0;
    
    // Configure for PTX
    int config = getRegister(CONFIG);
    config &= ~CONFIG_PRIM_RX;
    setRegister(CONFIG, config);
    
    // Write packet data
    _csn = 0;
    _spi.write(W_TX_PAYLOAD);
    _spi.write( (data>>0) & 0xff );
    _spi.write( (data>>8) & 0xff );
    _spi.write( (data>>16) & 0xff );
    _spi.write( (data>>24) & 0xff );
    _csn = 1;
    
    // Put into PTX
    _ce = 1;
    wait_us(TIMING_Tstby2a);
    _ce = 0;
    
    // Wait for message transmission and put into PRX
    wait_us(TIMING_Toa);
    config = getRegister(CONFIG);
    config |= CONFIG_PRIM_RX;
    setRegister(CONFIG, config);
    setRegister(STATUS, STATUS_TX_DS);
    _ce = 1;
}



void Radio::receive()
{
    uint32_t data = 0;
    int pipe;
    
    while (!(getRegister(FIFO_STATUS) & FIFO_STATUS_RX_EMPTY))
    {
        // Check data pipe
        //wait_us(1000);
        pipe = getStatus() & STATUS_RN_P_MASK;
        
        // Read data
        _csn = 0;
        _spi.write(R_RX_PAYLOAD);
        data |= _spi.write(NOP)<<0;
        data |= _spi.write(NOP)<<8;
        data |= _spi.write(NOP)<<16;
        data |= _spi.write(NOP)<<24;
        _csn = 1;
        
        // Sort into recieve buffer
        switch(pipe)
        {
        case STATUS_RN_P_NO_P0:
            rx_controller = data;
            clearTimeout.attach(this, &Radio::clear, 0.5f);
            break;
            
        case STATUS_RN_P_NO_P1:
            rx_robot[rx_robot_pos++ % RX_BUFFER_SIZE] = data;
            break;
            
        default:
            break;
        }
    }
    
    // Reset IRQ pin
    setRegister(STATUS, STATUS_RX_DR);
}



void Radio::clear()
{
    rx_controller = 0;
}



int Radio::getRegister(int address)
{
    _csn = 0;
    int rc = R_REGISTER | (address & REGISTER_ADDRESS_MASK);
    _spi.write(rc);
    int data = _spi.write(NOP);
    _csn = 1;
    return data;
}



int Radio::getStatus()
{
    _csn = 0;
    int status = _spi.write(NOP);
    _csn = 1;
    return status;
}


void Radio::setRegister(int address, int data)
{
    bool enabled = false;
    if (_ce == 1)
    {
        enabled = true;
        _ce = 0;
    }
    
    _csn = 0;
    int rc = W_REGISTER | (address & REGISTER_ADDRESS_MASK);
    _spi.write(rc);
    _spi.write(data & 0xff);
    _csn = 1;
    
    if (enabled)
    {
        _ce = 1;
        wait_us(TIMING_Tpece2csn);
    }
    
}



/************************************************************************/



RadioController::RadioController(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce) 
: _spi(mosi, miso, sck), _csn(csn), _ce(ce)
{
    // Disable nRF24L01+
    _ce = 0;
    
    // Disable chip select
    _csn = 1;
    
    // Set up SPI
    _spi.frequency(SPI_FREQUENCY);
    _spi.format(8, 0);
    
    // Set to use controller channel 0
    controller = 0;
}



void RadioController::reset()
{
    // Wait for power on reset
    wait_us(TIMING_Tpor);

    // Put into standby
    _ce = 0;
    
    // Configure registers
    setRegister(CONFIG, CONFIG_MASK_RX_DR | CONFIG_MASK_TX_DS | CONFIG_MASK_MAX_RT | CONFIG_EN_CRC | CONFIG_PWR_UP);
    setRegister(EN_AA, 0x00);
    setRegister(EN_RXADDR, 0x00);
    setRegister(SETUP_AW, SETUP_AW_3BYTES);
    setRegister(SETUP_RETR, 0x00);
    setRegister(RF_CH, RF_CHANNEL);
    setRegister(RF_SETUP, RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_PWR_0);
    setRegister(STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    setRegister(DYNPD, 0x00);
    setRegister(FEATURE, 0x00);
    
    // Set transmit address
    _csn = 0;
    _spi.write(W_REGISTER | TX_ADDR);
    _spi.write(CTRL_BASE_ADDRESS_1 + (controller & 0xf));
    _spi.write(CTRL_BASE_ADDRESS_2);
    _spi.write(CTRL_BASE_ADDRESS_3);
    _csn = 1;
    
    // Flush TX FIFO
    _csn = 0;
    _spi.write(FLUSH_TX);
    _csn = 1;
}



void RadioController::transmit(uint32_t data)
{   
    // Write packet data
    _csn = 0;
    _spi.write(W_TX_PAYLOAD);
    _spi.write( (data>>0) & 0xff );
    _spi.write( (data>>8) & 0xff );
    _spi.write( (data>>16) & 0xff );
    _spi.write( (data>>24) & 0xff );
    _csn = 1;
    
    // Put into PTX and transmit packet
    _ce = 1;
    wait_us(TIMING_Tstby2a);
    
    // Go back into standby
    _ce = 0;
}



void RadioController::setRegister(int address, int data)
{
    bool enabled = false;
    if (_ce == 1)
    {
        enabled = true;
        _ce = 0;
    }
    
    _csn = 0;
    int rc = W_REGISTER | (address & REGISTER_ADDRESS_MASK);
    _spi.write(rc);
    _spi.write(data & 0xff);
    _csn = 1;
    
    if (enabled)
    {
        _ce = 1;
        wait_us(TIMING_Tpece2csn);
    }
    
}
