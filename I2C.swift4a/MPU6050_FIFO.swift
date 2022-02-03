// Author: David Peterson
// Date: 28/11/2020
// IDE Version: 4.5
// Description: This adds configuration and access support for the FIFO buffer on the MPU-6050.

import AVR


//------------------------------------------------------------------------------
// FIFO
//------------------------------------------------------------------------------

extension MPU6050 {

    /**
     * This register determines which sensor measurements are loaded into the FIFO buffer.
     *
     * Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the
     * FIFO buffer if a sensor’s respective `FIFO_EN` bit is set to `1` in this register.
     *
     * When a sensor’s `FIFO_EN` bit is enabled in this register, data from the sensor data
     * registers will be loaded into the FIFO buffer. The sensors are sampled at the Sample Rate
     * as defined in Register 25. For further information regarding sensor data registers,
     * please refer to Registers 59 to 96.
     *
     * When an external Slave’s corresponding `FIFO_EN` bit (`SLVx_FIFO_EN`, where
     * `x`=`0`, `1`, or `2`) is set to `1`, the data stored in its corresponding data registers
     * (`EXT_SENS_DATA` registers, Registers 73 to 96) will be written into the FIFO buffer at
     * the Sample Rate. `EXT_SENS_DATA` register association with I2C Slaves is determined by
     * the `I2C_SLVx_CTRL` registers (where `x`=`0`, `1`, or `2`; Registers 39, 42, and 45).
     * For information regarding `EXT_SENS_DATA` registers, please refer to Registers 73 to 96.
     *
     * Note that the corresponding `FIFO_EN` bit (`SLV3_FIFO_EN`) is found in `I2C_MST_CTRL`
     * (Register 36). Also note that Slave 4 behaves in a different manner compared to Slaves 0-3.
     * Please refer to Registers 49 to 53 for further information regarding Slave 4 usage.
     */
    public var fifoEnable: FifoEnable {
        get { read(from: 0x23) }
        set { write(to: 0x23, value: newValue) }
    }
}

public struct FifoEnable: I2CMutableRegisterValue {

    /// The register value.
    public var registerValue: UInt8

    /// If `true`, the `tempOut` values will be loaded into the FIFO buffer.
    public var temp: Bool {
        get { hasBit(at: 7) }
        set { setBit(at: 7, to: newValue) }
    }

    /// If `true`, the `GYRO_XOUT_H/L` values will be loaded into the FIFO buffer.
    public var gyroX: Bool {
        get { hasBit(at: 6) }
        set { setBit(at: 6, to: newValue) }
    }

    /// If `true`, the `GYRO_YOUT_H/L` values will be loaded into the FIFO buffer.
    public var gyroY: Bool {
        get { hasBit(at: 5) }
        set { setBit(at: 5, to: newValue) }
    }

    /// If `true`, the `GYRO_ZOUT_H/L` values will be loaded into the FIFO buffer.
    public var gyroZ: Bool {
        get { hasBit(at: 4) }
        set { setBit(at: 4, to: newValue) }
    }

    /// If `true`, the `ACCEL_XOUT_H/L`, `ACCEL_YOUT_H/L`, `ACCEL_ZOUT_H/L` values
    /// will be loaded into the FIFO buffer.
    public var accel: Bool {
        get { hasBit(at: 3) }
        set { setBit(at: 3, to: newValue) }
    }

    /// When set to `true`, this bit enables `extraSensorData` registers (Registers 73 to 96)
    /// associated with Slave 2 to be written into the FIFO buffer.
    public var slave2: Bool {
        get { hasBit(at: 2) }
        set { setBit(at: 2, to: newValue) }
    }

    /// When set to `true`, this bit enables `extraSensorData` registers (Registers 73 to 96)
    /// associated with Slave 1 to be written into the FIFO buffer.
    public var slave1: Bool {
        get { hasBit(at: 1) }
        set { setBit(at: 1, to: newValue) }
    }

    /// When set to `true`, this bit enables `extraSensorData` registers (Registers 73 to 96)
    /// associated with Slave 0 to be written into the FIFO buffer.
    public var slave0: Bool {
        get { hasBit(at: 0) }
        set { setBit(at: 0, to: newValue) }
    }

    /// Initializes the `FifoEnable` with the specified register value.
    ///
    /// - Parameter registerValue: The register value.
    public init(registerValue: UInt8) {
        self.registerValue = registerValue
    }
}

// The flag for Slave 3 is in the `I2MasterControl` register
extension I2CMasterControl {
    /// When  set to `true`, Slave 3 sensor measurement data will be loaded into the
    /// FIFO buffer each time. `extraSensorData` register association with I2C Slaves
    /// is determined by `Slave.control` (Register 48).
    ///
    /// For further information regarding `extraSensorData` registers, please refer to
    /// Registers 73 to 96.
    ///
    /// The corresponding settings for Slave 0, Slave 1, and Slave 2 can be found
    /// in `FifoEnable` (Register 35).
    public var slave3FifoEnabled: Bool {
        get { hasBit(at: 5) }
        set { setBit(at: 5, to: newValue) }
    }
}


extension InterruptEnable {
    /// When set to `true`, this bit enables a FIFO buffer overflow to generate an interrupt.
    public var fifoOverflowEnabled: Bool {
        get { hasBit(at: 4) }
        set { setBit(at: 4, to: newValue) }
    }
}

extension InterruptStatus {
    /// This automatically sets to `true` when a FIFO buffer overflow interrupt has been generated.
    ///
    /// The bit clears `false` after the register has been read.
    public var fifoOverflowInterrupt: Bool { hasBit(at: 4) }
}

extension UserControl {
    /// When set to `true`, this bit enables FIFO operations.
    ///
    // When this is `false`, the FIFO buffer is disabled. The FIFO buffer cannot be written to or read from while disabled.
    ///
    /// The FIFO buffer’s state does not change unless the MPU-60X0 is power cycled.
    public var fifoEnabled: Bool {
        get { hasBit(at: 6) }
        set { setBit(at: 6, to: newValue) }
    }

    /// This bit resets the FIFO buffer when set to `true` while `fifoEnabled` is `false`. This bit automatically resets
    /// to `false` after the reset has been triggered.
    public var fifoReset: Bool {
        get { hasBit(at: 2) }
        set { setBit(at: 2, to: newValue) }
    }
}

extension MPU6050 {
    // # Register 114 and 115: FIFO Count Registers
    //
    // These registers keep track of the number of samples currently in the FIFO buffer.
    //
    // These registers shadow the FIFO Count value. Both registers are loaded with the current sample count when
    // `fifoCountHighReg` (Register 72) is read.
    //
    // Note: Reading only `fifoCountLowReg` will not update the registers to the current sample count.
    // `fifoCountHighReg` must be accessed first to update the contents of both these registers.
    //
    // `fifoCount` should always be read in high-low order in order to guarantee that the most current FIFO Count value is read.

    /// Indicates the number of bytes stored in the FIFO buffer. This number is
    /// in turn the number of bytes that can be read from the FIFO buffer and it is directly proportional
    /// to the number of samples available given the set of sensor data bound to be stored in the FIFO (register 35 and 36).
    public var fifoCount: UInt16 {
        get { read(from: 0x72...0x73) }
    }

    /// This register is used to read and write data from the FIFO buffer.
    ///
    /// Data is written to the FIFO in order of register number (from lowest to highest). If all the FIFO enable flags
    /// (see below) are enabled and all External Sensor Data registers (Registers 73 to 96) are associated with a
    /// Slave device, the contents of registers 59 through 96 will be written in order at the Sample Rate.
    ///
    /// The contents of the sensor data registers (Registers 59 to 96) are written into the FIFO buffer when their
    /// corresponding FIFO enable flags are set to 1 in FIFO_EN (Register 35). An additional flag for the sensor data
    /// registers associated with I2C Slave 3 can be found in `I2CMasterControl` (Register 36).
    ///
    /// If the FIFO buffer has overflowed, the status bit `fifoOverflowInterrupt` is automatically set to `true`.
    /// This is located in `InterruptStatus` (Register 58). When the FIFO buffer has overflowed, the oldest data will
    /// be lost and new data will be written to the FIFO.
    ///
    /// If the FIFO buffer is empty, reading this register will return the last byte that was previously read from the
    /// FIFO until new data is available. The user should check `fifoCount` to ensure that the FIFO buffer is not read when empty.
    public var fifoData: UInt8 {
        get { read(from: 0x74) }
        set { write(to: 0x74, value: newValue) }
    }
}
