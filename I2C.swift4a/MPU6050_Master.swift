// Author: David Peterson
// Date: 28/11/2020
// IDE Version: 4.5
// Description: This library adds support for the MPU-6050 to act as I2C Master for other external devices. It requires the main MPU6050 module to be installed.

import AVR

//------------------------------------------------------------------------------
// I2C Master Control
//------------------------------------------------------------------------------

extension MPU6050 {
    public var i2CMasterControl: I2CMasterControl {
        get { read(from: 0x24) }
        set { write(to: 0x24, value: newValue) }
    }
}

/// This register configures the auxiliary I2C bus for single-master or multi-master control. In addition,
/// the register is used to delay the Data Ready interrupt, and also enables the writing of Slave 3 data
/// into the FIFO buffer. The register also configures the auxiliary I2C Master’s transition from one
/// slave read to the next, as well as the MPU-60X0’s 8MHz internal clock.
///
/// Multi-master capability allows multiple I2C masters to operate on the same bus. In circuits where
/// multi-master capability is required, set `multiMasterEnabled` to `true`. This will increase current
/// drawn by approximately 30µA.
///
/// In circuits where multi-master capability is required, the state of the I2C bus must always be monitored
/// by each separate I2C Master. Before an I2C Master can assume arbitration of the bus, it must first
/// confirm that no other I2C Master has arbitration of the bus. When `multiMasterEnabled` is set to `true`,
/// the MPU-60X0’s bus arbitration detection logic is turned on, enabling it to detect when the bus is available.
///
/// When the `waitForExternalSensor` is set to `true`, the Data Ready interrupt will be delayed until External
/// Sensor data from the Slave Devices are loaded into the `extraSensorData` registers. This is used to ensure
/// that both the internal sensor data (i.e. from gyro and accel) and external sensor data have been loaded to
/// their respective data registers (i.e. the data is synced) when the Data Ready interrupt is triggered.
///
/// For further information regarding `extraSensorData` registers, please refer to Registers 73 to 96.
///
/// The I2C_MST_P_NSR bit configures the I2 C Master’s transition from one slave read to the next slave read. If the bit equals 0, there will be a restart between reads. If the bit equals 1, there will be a stop followed by a start of the following read. When a write transaction follows a read transaction, the stop followed by a start of the successive write will be always used.
public struct I2CMasterControl: I2CMutableRegisterData {
    public var registerValue: UInt8

    public init(registerValue: UInt8) {
        self.registerValue = registerValue
    }

    /// Multi-master capability allows multiple I2C masters to operate on the same bus.
    /// In circuits where multi-master capability is required, set this to `true`.
    /// This will increase current drawn by approximately 30µA.
    public var multiMasterEnabled: Bool {
        get { hasBit(at: 7) }
        set { setBit(at: 7, to: newValue) }
    }

    /// When set to `true, this bit delays the Data Ready interrupt until External Sensor
    /// data from the Slave devices have been loaded into the `extraSensorData` registers.
    public var waitForExternalSensor: Bool {
        get { hasBit(at: 6) }
        set { setBit(at: 6, to: newValue) }
    }

    /// Controls the I2 C Master’s transition from one slave read to the next slave read.
    ///
    /// - When this is `.restart`, there is a restart between reads.
    /// - When this is `.stopAndStart`, there is a stop and start marking the beginning of the next read.
    ///
    /// When a write follows a read, a stop and start is always enforced.
    public var masterTransition: Bool {
        get { hasBit(at: 4) }
        set { setBit(at: 4, to: newValue) }
    }

    /// Configures the I2C master clock speed divider.
    /// It sets the I2C master clock speed according to the following table:
    ///
    /// ||   Clock   || I2C Master Clock Speed || 8MHz Clock Divider ||
    /// |  .zero     |           348 kHz       |         23           |
    /// |  .one      |           333 kHz       |         24           |
    /// |  .two      |           320 kHz       |         25           |
    /// |  .three    |           308 kHz       |         26           |
    /// |  .four     |           296 kHz       |         27           |
    /// |  .five     |           286 kHz       |         28           |
    /// |  .six      |           276 kHz       |         29           |
    /// |  .seven    |           267 kHz       |         30           |
    /// |  .eight    |           258 kHz       |         31           |
    /// |  .nine     |           500 kHz       |         16           |
    /// |  .ten      |           471 kHz       |         17           |
    /// |  .eleven   |           444 kHz       |         18           |
    /// |  .twelve   |           421 kHz       |         19           |
    /// |  .thirteen |           400 kHz       |         20           |
    /// |  .fourteen |           381 kHz       |         21           |
    /// |  .fifteen  |           364 kHz       |         22           |
    public var masterClock: Clock {
        get { getBits(from: 0...3) }
        set { setBits(from: 0...3, to: newValue) }
    }

    public enum SlaveTransition: UInt8 {
        /// Indicates there is a restart between reads.
        case restart = 0

        /// Indicates there is a stop and start marking the beginning of the next read.
        case stopAndStart = 1
    }

    /// A 4 bit unsigned value which configures a divider on the MPU-60X0 internal 8MHz clock.
    public enum Clock: UInt8 {
        case zero = 0
        case one = 1
        case two = 2
        case three = 3
        case four = 4
        case five = 5
        case six = 6
        case seven = 7
        case eight = 8
        case nine = 9
        case ten = 10
        case eleven = 11
        case twelve = 12
        case thirteen = 13
        case fourteen = 14
        case fifteen = 15

        var speed: UInt16 {
            switch self {
                case .zero: return 348
                case .one: return 333
                case .two: return 320
                case .three: return 308
                case .four: return 296
                case .five: return 286
                case .six: return 276
                case .seven: return 267
                case .eight: return 258
                case .nine: return 500
                case .ten: return 471
                case .eleven: return 444
                case .twelve: return 421
                case .thirteen: return 400
                case .fourteen: return 381
                case .fifteen: return 364
            }
        }

        var divider: UInt16 {
            switch self {
                case .zero: return 23
                case .one: return 24
                case .two: return 25
                case .three: return 26
                case .four: return 27
                case .five: return 28
                case .six: return 29
                case .seven: return 30
                case .eight: return 31
                case .nine: return 16
                case .ten: return 17
                case .eleven: return 18
                case .twelve: return 19
                case .thirteen: return 20
                case .fourteen: return 21
                case .fifteen: return 22
            }
        }

    }
}

/// Slave 0-3 Configuration

//------------------------------------------------------------------------------
// I2C Slave Control
//------------------------------------------------------------------------------

public extension MPU6050 {
    /// Returns a `Slave` instance pointing at the Slave 0 registers.
    var slave0: Slave {
        get { read(from: 0x25...0x27) }
        set { write(to: 0x25...0x27, value: newValue) }
    }

    /// Returns a `Slave` instance pointing at the Slave 1 registers.
    var slave1: Slave {
        get { read(from: 0x28...0x2A) }
        set { write(to: 0x28...0x2A, value: newValue) }
    }

    /// Returns a `Slave` instance pointing at the Slave 2 registers.
    var slave2: Slave {
        get { read(from: 0x2B...0x2D) }
        set { write(to: 0x2B...0x2D, value: newValue) }
    }

    /// Returns a `Slave` instance pointing at the Slave 3 registers.
    var slave3: Slave {
        get { read(from: 0x2E...0x30) }
        set { write(to: 0x2E...0x30, value: newValue) }
    }

    /// Returns a `Slave` instance pointing at the Slave 4 registers.
    var slave4: Slave4 {
        get { read(from: 0x31...0x35) }
        set { write(to: 0x31...0x35, value: newValue) }
    }

    /// Configures Slave 0-3 connections.
    struct Slave: I2CRegisterBlock {

        /// Contains read/write bit and the slave's 7-bit address.
        private var address: UInt8

        /// The register to read from/write to in the slave.
        public var slaveRegister: UInt8

        /// Contains bits for configuring the slave.
        private var control: UInt8

        public init() {
            address = 0
            slaveRegister = 0
            control = 0
        }

        /// If `true` the slave is reading, if `false` it is writing.
        /// I2C slave data transactions between the MPU-60X0 and Slave 0 are set as either
        /// read or write operations this `Bool`. When this is `true`, the transfer is a read operation.
        /// When it is `false`, the transfer is a write operation.
        public var reading: Bool {
            get { address.hasBit(at: 7) }
            set { address.setBit(at: 7, to: newValue) }
        }

        /// The address of the Slave unit.
        public var slaveAddress: UInt8 {
            get { address.getBits(from: 0...6) }
            set { address.setBits(from: 0...6, to: newValue) }
        }

        /// Enables the Slave for I2C data transaction. A data transaction is performed
        /// only if more than zero bytes are to be transferred (`length > 0`)
        /// between an enabled slave device (`enabled = true`).
        public var enabled: Bool {
            get { control.hasBit(at: 7) }
            set { control.setBit(at: 7, to: newValue) }
        }

        /// Configures byte swapping of word pairs. When byte swapping is enabled, the
        /// high and low bytes of a word pair are swapped. Please refer to `groupingOrder`
        /// for the pairing convention of the word pairs. When this bit is set to `false`,
        /// bytes transferred to and from the Slave will be written to `extraSensorData`
        /// registers in the order they were transferred.
        public var swapBytes: Bool {
            get { control.hasBit(at: 6) }
            set { control.setBit(at: 6, to: newValue) }
        }

        /// When set to `true`, the transaction will read or write data only.
        ///
        /// When cleared to `false`, the transaction will write a register address
        /// prior to reading or writing data.
        public var registerTransaction: Bool {
            get { control.hasBit(at: 5) }
            set { control.setBit(at: 5, to: newValue) }
        }

        /// Value specifying the grouping order of word pairs received from registers.
        /// When set to `.evenThenOdd`, bytes from register addresses `0` and `1`, `2` and `3`,
        /// etc (even, then odd register addresses) are paired to form a word. When set to
        /// `.oddThenEven`, bytes from register addresses are paired `1` and `2`, `3`
        /// and `4`, etc. (odd, then even register addresses) are paired to form a word.
        public var groupingOrder: Slave.GroupingOrder {
            get { Slave.GroupingOrder(rawValue: control.getBit(at: 1)) ?? .evenThenOdd }
            set { control.setBit(at: 4, to: newValue.rawValue) }
        }

        /// 4-bit unsigned value. Specifies the number of bytes transferred to and from the Slave.
        ///
        /// Clearing this bit to `0` is equivalent to disabling the register by setting `enabled` to `false`.
        public var length: UInt8 {
            get { control.getBits(from: 0...3) }
            set { control.setBits(from: 0...3, to: newValue) }
        }

        public enum GroupingOrder: UInt8 {
            case evenThenOdd = 0
            case oddThenEven = 1
        }
    }

    /// These registers describe the data transfer sequence for Slave 4. The characteristics of Slave 4 differ greatly from
    /// those of Slaves 0-3. For further information regarding the characteristics of Slaves 0-3, please refer to Registers 37 to 48.
    ///
    /// I2C slave data transactions between the MPU-60X0 and Slave 4 are set as either read or write operations by the `reading` bit.
    /// When this is `true`, the transfer is a read operation. When `false`, the transfer is a write operation.
    ///
    /// `slaveAddress` is used to specify the I2C slave address of Slave 4.
    ///
    /// Data transfer starts at an internal register within Slave 4. This register address is specified by `slaveRegister`.
    ///
    /// In read mode, the result of the read will be available in `dataInput`. In write mode, the contents of `dataOutput` will
    /// be written into the slave device.
    ///
    /// A data transaction is performed only if `enabled` bit is set to `true`. The data transaction should be enabled once its
    /// parameters are configured in the internal `address` and `slaveRegister` registers. For write, the `dataOutput` register
    /// is also required. `enabled` will be cleared after the transaction is performed once.
    ///
    /// An interrupt is triggered at the completion of a Slave 4 data transaction if the interrupt is enabled . The status of this
    /// interrupt can be observed in Register 54.
    ///
    /// When `I2C_SLV4_REG_DIS` is set to 1, the transaction will read or write data instead of writing a register address.
    /// This should equal false when specifying the register address within the Slave device to/from which the ensuing data
    ///  transaction will take place.
    ///
    /// `I2C_MST_DLY` configures the reduced access rate of I2C slaves relative to the Sample Rate. When a slave’s access rate is
    /// decreased relative to the Sample Rate, the slave is accessed every
    ///
    ///     1 / (1 + I2C_MST_DLY) samples
    ///
    /// This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and DLPF_CFG (register 26). Whether a slave’s
    /// access rate is reduced relative to the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103).
    ///
    /// For further information regarding the Sample Rate, please refer to register 25.
    ///
    /// Slave 4 transactions are performed after Slave 0, 1, 2 and 3 transactions have been completed. Thus the maximum rate
    /// for Slave 4 transactions is determined by the Sample Rate as defined in Register 25.
    struct Slave4: I2CRegisterBlock {

        // Contains the read/write bit and the 7-bit slave address.
        private var address: UInt8

        /// The register to read to/write from on the slave device.
        public var slaveRegister: UInt8

        /// This register stores the data to be written into the Slave 4.
        ///
        /// If `reading` is `true`, this register has no effect.
        public var dataOutput: UInt8

        // Contains various configuration values.
        private var control: UInt8

        /// This register stores the data read from Slave 4.
        ///
        /// This field is populated after a read transaction.
        public var dataInput: UInt8

        public init() {
            address = 0
            slaveRegister = 0
            dataOutput = 0
            control = 0
            dataInput = 0
        }

        /// If `true` the slave is reading, if `false` it is writing.
        /// I2C slave data transactions between the MPU-60X0 and Slave 0 are set as either
        /// read or write operations this `Bool`. When this is `true`, the transfer is a read operation.
        /// When it is `false`, the transfer is a write operation.
        public var reading: Bool {
            get { address.hasBit(at: 7) }
            set { address.setBit(at: 7, to: newValue) }
        }

        /// The address of the Slave unit.
        public var slaveAddress: UInt8 {
            get { address.getBits(from: 0...6) }
            set { address.setBits(from: 0...6, to: newValue) }
        }

        /// When set to `true`, this bit enables Slave 4 for data transfer operations.
        ///
        /// When `false`, this bit disables Slave 4 from data transfer operations.
        public var enabled: Bool {
            get { control.hasBit(at: 7) }
            set { control.setBit(at: 7, to: newValue) }
        }

        /// When set to `true`, this bit enables the generation of an interrupt
        /// signal upon completion of a Slave 4 transaction.
        ///
        /// When is `false`, it disables the generation of an interrupt signal
        /// upon completion of a Slave 4 transaction.
        ///
        /// The interrupt status can be observed in Register 54.
        public var interruptEnabled: Bool {
            get { control.hasBit(at: 6) }
            set { control.setBit(at: 6, to: newValue) }
        }

        /// When set to `.data`, the transaction will read or write data.
        ///
        /// When `.registerAddress`, the transaction will read or write a register address.
        public var transferMode: Slave4.Mode {
            get { control.getBit(at: 5) }
            set { control.setBit(at: 5, to: newValue) }
        }

        /// Configures the decreased access rate of slave devices relative to the Sample Rate.
        public var delay: UInt8 {
            get { control.getBits(from: 0...4) }
            set { control.setBits(from: 0...4, to: newValue) }
        }

        public enum Mode: UInt8 {
            case data = 1
            case registerAddress = 0
        }
    }
}

public extension MPU6050 {
    var masterStatus: MasterStatus {
        get { read(from: 0x37) }
        set { write(to: 0x37, value: newValue)}
    }

    struct MasterStatus: I2CMutableRegisterData {
        public var registerValue: UInt8 = 0

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// This reflects the status of the FSYNC interrupt from an
        /// external device into the MPU-60X0. This is used as a way
        /// to pass an external interrupt through the MPU-60X0 to the
        /// host application processor.
        ///
        /// When set to `true`, this bit will cause an interrupt if
        /// `FSYNC_INT_EN` is asserted in `INT_PIN_CFG` (Register 55).
        public var passThrough: Bool {
            get { hasBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }

        /// Automatically sets to `true` when a Slave 4 transaction
        /// has completed. This triggers an interrupt if the
        /// `I2C_MST_INT_EN` bit in the `INT_ENABLE` register (Register 56)
        /// is asserted and if the `SLV_4_DONE_INT` bit is asserted in the
        /// `I2C_SLV4_CTRL` register (Register 52).
        public var slave4Done: Bool {
            get { hasBit(at: 6) }
            set { setBit(at: 6, to: newValue) }
        }

        /// This bit automatically sets to `true` when the I2C Master
        /// has lost arbitration of the auxiliary I2C bus
        /// (an error condition). This triggers an interrupt if the
        /// `I2C_MST_INT_EN` bit in the `INT_ENABLE` register (Register 56)
        /// is asserted.
        public var lostArbitration: Bool {
            get { hasBit(at: 5) }
            set { setBit(at: 5, to: newValue) }
        }

        ///
        public var slave4NACK: Bool {
            get { hasBit(at: 4) }
            set { setBit(at: 4, to: newValue) }
        }

        public var slave3NACK: Bool {
            get { hasBit(at: 3) }
            set { setBit(at: 3, to: newValue) }
        }

        public var slave2NACK: Bool {
            get { hasBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        public var slave1NACK: Bool {
            get { hasBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        public var slave0NACK: Bool {
            get { hasBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }
    }
}

//------------------------------------------------------------------------------
// Registers 67-72: External Sensor Data
//------------------------------------------------------------------------------

/**
 * These registers store data read from external sensors by the Slave 0, 1, 2, and 3 on the auxiliary I2C interface.
 * Data read by Slave 4 is stored in `I2C_SLV4_DI` (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as defined in Register 25. This access rate
 * can be reduced by using the Slave Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement registers, accelerometer measurement registers,
 * and temperature measurement registers, are composed of two sets of registers: an internal register set and a
 * user-facing read register set.
 *
 * The data within the external sensors’ internal register set is always updated at the Sample Rate (or the reduced
 * access rate) whenever the serial interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible
 * for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to `I2C_SLV0_CTRL`, `I2C_SLV1_CTRL`, `I2C_SLV2_CTRL`,
 * and `I2C_SLV3_CTRL` (Registers 39, 42, 45, and 48). When more than zero bytes are read (`I2C_SLVx_LEN > 0`) from an
 * enabled slave (`I2C_SLVx_EN = 1`), the slave is read at the Sample Rate (as defined in Register 25) or delayed rate
 * (if specified in Register 52 and 103). During each Sample cycle, slave reads are performed in order of Slave number.
 * If all slaves are enabled with more than zero bytes to be read, the order will be Slave 0, followed by Slave 1,
 * Slave 2, and Slave 3.

 * Each enabled slave will have `EXT_SENS_DATA` registers associated with it by number of bytes read (`I2C_SLVx_LEN`)
 * in order of slave number, starting from `EXT_SENS_DATA_00`. Note that this means enabling or disabling a slave
 * may change the higher numbered slaves’ associated registers. Furthermore, if fewer total bytes are being read
 * from the external sensors as a result of such a change, then the data remaining in the registers which no longer
 * have an associated slave device (i.e. high numbered registers) will remain in these previously allocated
 * registers unless reset.
 *
 * If the sum of the read lengths of all `SLVx` transactions exceed the number of available `EXT_SENS_DATA` registers,
 * the excess bytes will be dropped. There are 24 `EXT_SENS_DATA` registers and hence the total read lengths between
 * all the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4’s behavior is distinct from that of Slaves 0-3. For further information regarding the characteristics
 * of Slave 4, please refer to Registers 49 to 53.
 */

extension MPU6050 {
    /// The `I2CRegister` for the first external sensor data register.
    internal static let externalSensorData0Reg = 0x49

    /// 8 bit unsigned value that is written into Slave 0 when Slave 0 is set to write mode.
    public var slave0DataOut: UInt8 {
        get { read(from: 0x63) }
        set { write(to: 0x63, value: newValue) }
    }

    /// 8 bit unsigned value that is written into Slave 1 when Slave 1 is set to write mode.
    public var slave1DataOut: UInt8 {
        get { read(from: 0x64) }
        set { write(to: 0x64, value: newValue) }
    }

    /// 8 bit unsigned value that is written into Slave 2 when Slave 2 is set to write mode.
    public var slave2DataOut: UInt8 {
        get { read(from: 0x65) }
        set { write(to: 0x65, value: newValue) }
    }

    /// 8 bit unsigned value that is written into Slave 3 when Slave 3 is set to write mode.
    public var slave3DataOut: UInt8 {
        get { read(from: 0x66) }
        set { write(to: 0x66, value: newValue) }
    }

    /// This register is used to specify the timing of external sensor data shadowing.
    /// The register is also used to decrease the access rate of slave devices relative
    ///to the Sample Rate.
    public var i2CMasterDelayControl: I2CMasterDelayControl {
        get { read(from: 0x67) }
        set { write(to: 0x67, value: newValue) }
    }

    /// This register is used to specify the timing of external sensor data shadowing.
    /// The register is also used to decrease the access rate of slave devices relative to the Sample Rate.
    ///
    /// When `delayExternalSensorShadow` is set to `true`, shadowing of external sensor data is delayed
    /// until all data has been received.
    ///
    /// When `slave4DelayEnabled`, `slave3DelayEnabled`, `slave2DelayEnabled`, `slave1DelayEnabled`,
    /// and `slave0DelayEnabled` are enabled, the rate of access for the corresponding slave devices is
    /// reduced. When a slave’s access rate is decreased relative to the Sample Rate, the slave is accessed
    /// every `1 / (1 + I2C_MST_DLY)` samples.
    ///
    /// This base Sample Rate in turn is determined by `sampleRateDivisor` (register 25) and `DLPF_CFG` (register 26).
    ///
    /// For further information regarding `I2C_MST_DLY`, please refer to register 52.
    ///
    /// For further information regarding the Sample Rate, please refer to register 25.
    ///
    /// Bits 6 and 5 are reserved.
    public struct I2CMasterDelayControl: I2CMutableRegisterData {
        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// When set, delays shadowing of external sensor data until all data has been received.
        public var delayExternalSensorShadow: Bool {
            get { hasBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }

        /// When enabled, slave 4 will only be accessed at a decreased rate.
        public var slave4DelayEnabled: Bool {
            get { hasBit(at: 4) }
            set { setBit(at: 4, to: newValue) }
        }

        /// When enabled, slave 3 will only be accessed at a decreased rate.
        public var slave3DelayEnabled: Bool {
            get { hasBit(at: 3) }
            set { setBit(at: 3, to: newValue) }
        }

        /// When enabled, slave 2 will only be accessed at a decreased rate.
        public var slave2DelayEnabled: Bool {
            get { hasBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        /// When enabled, slave 1 will only be accessed at a decreased rate.
        public var slave1DelayEnabled: Bool {
            get { hasBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// When enabled, slave 0 will only be accessed at a decreased rate.
        public var slave0DelayEnabled: Bool {
            get { hasBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }
    }
}
