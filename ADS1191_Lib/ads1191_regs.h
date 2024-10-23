/*
 * ads1191_regs.h
 *
 *  Created on: Sep 14, 2024
 *      Author: mfaruk
 */
#ifndef ADS1191_REGS_H
#define ADS1191_REGS_H

typedef unsigned char byte;
#define MAX_REG_LEN 			   ((byte)(0x1F))

// Opcode
#define ADS1191_OPC_RREG          ((byte)(0x20))  // Read data by command
#define ADS1191_OPC_WREG          ((byte)(0x40))  // Read data by command
// Commands (SPI Instructions)
#define ADS1191_CMD_WAKEUP         ((byte)(0x02))  // Wake-up from standby mode
#define ADS1191_CMD_STANDBY        ((byte)(0x04))  // Enter standby mode
#define ADS1191_CMD_RESET          ((byte)(0x06))  // Reset the device
#define ADS1191_CMD_START          ((byte)(0x08))  // Start/restart (synchronize) conversions
#define ADS1191_CMD_STOP           ((byte)(0x0A))  // Stop conversion
#define ADS1191_CMD_OFFSETCAL      ((byte)(0x1A))  // Stop conversion
#define ADS1191_CMD_RDATAC         ((byte)(0x10))  // Enable Read Data Continuous mode
#define ADS1191_CMD_SDATAC         ((byte)(0x11))  // Stop Read Data Continuous mode
#define ADS1191_CMD_RDATA          ((byte)(0x12))  // Read data by command
// Register Addresses
#define ADS1191_REG_ID             ((byte)(0x00))  // ID Register
#define ADS1191_REG_CONFIG1        ((byte)(0x01))  // Configuration Register 1
#define ADS1191_REG_CONFIG2        ((byte)(0x02))  // Configuration Register 2
#define ADS1191_REG_LOFF           ((byte)(0x03))  // Lead-Off Control Register
#define ADS1191_REG_CH1SET         ((byte)(0x04))  // Channel 1 Set Register
#define ADS1191_REG_CH2SET         ((byte)(0x05))  // Channel 2 Set Register
#define ADS1191_REG_RLD_SENS       ((byte)(0x06))  // Right Leg Drive Sense Register
#define ADS1191_REG_LOFF_SENS      ((byte)(0x07))  // Lead-Off Sense Register
#define ADS1191_REG_LOFF_STAT      ((byte)(0x08))  // Lead-Off Status Register
#define ADS1191_REG_RESP1          ((byte)(0x09))  // Respiration Control Register 1
#define ADS1191_REG_RESP2          ((byte)(0x0A))  // Respiration Control Register 2
#define ADS1191_REG_GPIO           ((byte)(0x0B))  // General-Purpose I/O Register
#define ADS1191_REG_PACE           ((byte)(0x0C))  // PACE Detect Register
#define ADS1191_REG_CONFIG3        ((byte)(0x0D))  // Configuration Register 3

// Register Bit Definitions

// CONFIG1 Register (0x01)
#define ADS1191_CONFIG1_DR_MASK     ((byte)(0x07))  // Data Rate
#define ADS1191_CONFIG1_DR_125SPS   ((byte)(0x00))  // 125 Samples per Second
#define ADS1191_CONFIG1_DR_250SPS   ((byte)(0x01))  // 250 Samples per Second
#define ADS1191_CONFIG1_DR_500SPS   ((byte)(0x02))  // 500 Samples per Second
#define ADS1191_CONFIG1_DR_1KSPS    ((byte)(0x03))  // 1000 Samples per Second
#define ADS1191_CONFIG1_DR_2KSPS    ((byte)(0x04))  // 2000 Samples per Second
#define ADS1191_CONFIG1_DR_4KSPS    ((byte)(0x05))  // 4000 Samples per Second
#define ADS1191_CONFIG1_DR_8KSPS    ((byte)(0x06))  // 8000 Samples per Second

// CONFIG2 Register (0x02)
#define ADS1191_CONFIG2_TEST_FREQ  	((byte)(0x81))  //
#define ADS1191_CONFIG2_INT_TEST    ((byte)(0x82))  //
#define ADS1191_CONFIG2_CLK_EN      ((byte)(0x88))  //
#define ADS1191_CONFIG2_VREF_4V     ((byte)(0x90))  // Internal Reference Voltage (4V)
#define ADS1191_CONFIG2_PDB_REFBUF  ((byte)(0xA0))
#define ADS1191_CONFIG2_PDB_LOFF_CP ((byte)(0xC0))

// LOFF Register (0x03)
#define ADS1191_LOFF_COMP_TH_MASK   ((byte)(0xE0))  // Lead-Off Comparator Threshold
#define ADS1191_LOFF_COMP_TH_95     ((byte)(0x00))  // 95% of Supply
#define ADS1191_LOFF_COMP_TH_92_5   ((byte)(0x20))  // 92.5% of Supply
#define ADS1191_LOFF_COMP_TH_90     ((byte)(0x40))  // 90% of Supply
#define ADS1191_LOFF_COMP_TH_87_5   ((byte)(0x60))  // 87.5% of Supply
#define ADS1191_LOFF_COMP_TH_85     ((byte)(0x80))  // 85% of Supply
#define ADS1191_LOFF_COMP_TH_80     ((byte)(0xA0))  // 80% of Supply
#define ADS1191_LOFF_COMP_TH_75     ((byte)(0xC0))  // 75% of Supply
#define ADS1191_LOFF_ILEAD_OFF_MASK ((byte)(0x18))  // Lead-Off Current Magnitude
#define ADS1191_LOFF_ILEAD_OFF_6nA  ((byte)(0x00))  // 6 nA
#define ADS1191_LOFF_ILEAD_OFF_22nA ((byte)(0x08))  // 22 nA
#define ADS1191_LOFF_ILEAD_OFF_6uA  ((byte)(0x10))  // 6 μA
#define ADS1191_LOFF_ILEAD_OFF_22uA ((byte)(0x18))  // 22 μA

// CH1SET and CH2SET Registers (0x04, 0x05)
#define ADS1191_CHSET_PD           ((byte)(0x80))  // Power Down
#define ADS1191_CHSET_GAIN_MASK    ((byte)(0x70))  // Gain Settings
#define ADS1191_CHSET_GAIN_6       ((byte)(0x00))  // Gain = 6
#define ADS1191_CHSET_GAIN_1       ((byte)(0x10))  // Gain = 1
#define ADS1191_CHSET_GAIN_2       ((byte)(0x20))  // Gain = 2
#define ADS1191_CHSET_GAIN_3       ((byte)(0x30))  // Gain = 3
#define ADS1191_CHSET_GAIN_4       ((byte)(0x40))  // Gain = 4
#define ADS1191_CHSET_GAIN_8       ((byte)(0x50))  // Gain = 8
#define ADS1191_CHSET_GAIN_12      ((byte)(0x60))  // Gain = 12
#define ADS1191_CHSET_MUX_MASK     ((byte)(0x07))  // MUX Channel
#define ADS1191_CHSET_MUX_NORMAL   ((byte)(0x00))  // Normal Electrode Input
#define ADS1191_CHSET_MUX_SHORT    ((byte)(0x01))  // Input Shorted
#define ADS1191_CHSET_MUX_RLD_MEAS ((byte)(0x02))  // Right Leg Drive Measurement
#define ADS1191_CHSET_MUX_MVDD     ((byte)(0x03))  // Supply Measurement (AVDD)
#define ADS1191_CHSET_MUX_TEMP     ((byte)(0x04))  // Temperature Sensor
#define ADS1191_CHSET_MUX_TEST_SIG ((byte)(0x05))  // Test Signal
#define ADS1191_CHSET_MUX_RLD_DRP  ((byte)(0x06))  // RLD_DRP
#define ADS1191_CHSET_MUX_RLD_DRN  ((byte)(0x07))  // RLD_DRN

// CONFIG3 Register (0x0D)
#define ADS1191_CONFIG3_PD_REFBUF   ((byte)(0x80))  // Power Down Reference Buffer
#define ADS1191_CONFIG3_RLD_MEAS    ((byte)(0x40))  // RLD Measurement
#define ADS1191_CONFIG3_RLDREF_INT  ((byte)(0x20))  // RLDREF Internal
#define ADS1191_CONFIG3_RLD_EN      ((byte)(0x10))  // RLD Enable
#define ADS1191_CONFIG3_SINGLE_SHOT ((byte)(0x08))  // Single Shot Mode

#endif // ADS1191_REGS_H
