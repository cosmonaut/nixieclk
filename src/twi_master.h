#ifndef _TWI_MASTER_H_
#define _TWI_MASTER_H_

#define SCL_CLOCK 400000L

#define TWI_TWSR_status_mask 0xF8

/* Status codes for TWI Master Mode (TWSR) */
#define TWI_start_sent 0x08
#define TWI_repeated_start_sent 0x10
#define TWI_arbitration_lost 0x38

/* Status codes for TWI Master Transmitter Mode */
#define TWI_SLA_W_sent_ack_received 0x18
#define TWI_SLA_W_sent_nack_received 0x20
#define TWI_data_sent_ack_received 0x28
#define TWI_data_sent_nack_received 0x30

/* Status codes for TWI Master Receiver Mode */
#define TWI_SLA_R_sent_ack_received 0x40
#define TWI_SLA_R_sent_nack_received 0x48
#define TWI_data_received_ack_returned 0x50
#define TWI_data_received_nack_returned 0x58

#define TWI_BUFFER_MAX 255
volatile uint8_t TWI_buffer_in[TWI_BUFFER_MAX];
volatile uint8_t TWI_buffer_out[TWI_BUFFER_MAX];
volatile uint8_t TWI_target_slave_addr;

volatile uint8_t TWI_status;
#define TWI_WRITE_STATE 0x01
#define TWI_READ_STATE 0x02
volatile uint8_t TWI_operation;

/* call types */
volatile uint8_t TWI_master_state;
#define TWI_OP_WRITE_ONLY 0x01
#define TWI_OP_READ_ONLY 0x02
#define TWI_OP_WRITE_THEN_READ 0x03

/* control variables */
volatile uint8_t TWI_operation;
volatile uint8_t TWI_busy;
volatile uint8_t TWI_error;

/* buffers and variables */
//volatile uint16_t TWI_buffer_max;
volatile uint16_t TWI_buffer_pos;
//volatile uint8_t TWI_buffer_len;
volatile uint16_t TWI_read_bytes;
volatile uint16_t TWI_write_bytes;

#define TWI_ENABLE _BV(TWEN) | _BV(TWINT) | _BV(TWIE)
#define TWI_ACK _BV(TWEA) | TWI_ENABLE
#define TWI_NACK TWI_ENABLE
#define TWI_START _BV(TWSTA) | TWI_ENABLE
#define TWI_STOP _BV(TWSTO) | TWI_ENABLE

void TWI_init(void);
void TWI_master_start_write(uint8_t slave_addr, uint16_t write_bytes);
void TWI_master_start_read(uint8_t slave_addr, uint16_t read_bytes);
void TWI_master_start_write_then_read(uint8_t slave_addr, uint16_t write_bytes, uint16_t read_bytes);

#endif

