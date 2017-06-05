/*****************************************************************************

****************************************************************************/
    
#include <avr/interrupt.h>       
#include "TWI_HW_Master.h"


/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initializing the TWI.
****************************************************************************/
    
// Init Bus
void I2C_HW_Init()
{
	_TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
	// TWSR = TWI_TWPS;                                  // Not used. Driver presumes prescaler to be 00.
	_TWDR = 0xFF;                                      // Default content = SDA released.
	_TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
	(0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
	(0<<TWWC);                                 //
}

// Start Transfer
void I2C_HW_Start()
{
	_TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

// Stop Transfer
void I2C_HW_Stop()
{
	_TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

// Write Transfer
unsigned int I2C_HW_Write(unsigned int a)
{

}

// Read Transfer
unsigned int I2C_HW_Read(unsigned int ack)
{

}

// Read Byte
unsigned int I2C_HW_ReadByte(unsigned int nAddress, unsigned int nRegister)
{

}

// Write Byte
void I2C_HW_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue)
{

}


/*
ISR(_TWI_vect)
{
  static unsigned char TWI_bufPtr;
  
  switch (_TWSR)
  {
    case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      TWI_bufPtr = 0;                                     // Set buffer pointer to the TWI Address location
    case TWI_MTX_ADR_ACK:       // SLA+W has been tramsmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been tramsmitted and ACK received
      if (TWI_bufPtr < TWI_msgSize)
      {
        _TWDR = TWI_buf[TWI_bufPtr++];
        _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
               (0<<TWWC);                                 //  
      }else                    // Send STOP after last byte
      {
        TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
        _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
               (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
               (0<<TWWC);                                 //
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK tramsmitted
      TWI_buf[TWI_bufPtr++] = _TWDR;
    case TWI_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received
      if (TWI_bufPtr < (TWI_msgSize-1) )                  // Detect the last byte to NACK it.
      {
        _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
               (0<<TWWC);                                 //  
      }else                    // Send NACK after next reception
      {
        _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
               (0<<TWWC);                                 // 
      }    
      break; 
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK tramsmitted
      TWI_buf[TWI_bufPtr] = _TWDR;
      TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
      _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
             (0<<TWWC);                                 //
      break;      
    case TWI_ARB_LOST:          // Arbitration lost
      _TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received    
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
//    case TWI_NO_STATE              // No relevant state information available; TWINT = “0”
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWI_state = _TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
                                                        // Reset TWI Interface
      _TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
             (0<<TWWC);                                 //
  }
}
*/
