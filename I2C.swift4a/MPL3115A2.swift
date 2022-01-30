// Author: David Peterson
// Date: 28/01/2021
// IDE Version: 4.5
// Description: This library allows you to read altitude, pressure and temperature
// on an MPL3115A2 sensor via the I2C bus.
// Based on Carl Peto's MPL3115A2 example library and the MPL3115A2 Data Sheet

import AVR

fileprivate func waitUntil(_ done: () -> Bool, delaying ms: () -> UInt16 = { 10 }) {
    while !done() {
        delay(milliseconds: ms())
    }
}

/// Represents the MPL3115A2 barometer/altimeter sensor.
public struct MPL3115A2: I2CSlaveNode {
    public let address: UInt8 = 0x60

}

extension MPL3115A2 {
    public enum Enabled: UInt8 {
        case disabled = 0
        case enabled = 1
    }
}

extension MPL3115A2 {
    static let readAddress: UInt8 = 0xC1
    static let writeAddress: UInt8 = 0xC0

    static let whoamiID: UInt8 = 0xC4

    /// Once I2C has been set up, check that the sensor is available and connected.
    public var ready: Bool {
        read(from: 0x0C) == Self.whoamiID
    }

    /// Waits for the device to be `ready`, then setup standard flags.
    public mutating func begin() {
        waitUntil { ready }
        sensorData.enableAllFlags()
    }

    /// Get the current altitude from a running sensor.
    public mutating func currentAltitude() -> Float {
        // start the altimeter, turn on the analog systems, ADC, and set oversampling rate to 128x, read as altitude
        control1 = .init(mode: .altimeter, oversampleRatio: .x128, active: true)

        // wait for the altimeter to be ready
        waitUntil { dataReadyStatus.pressureTemperatureDataReady }

        let rawValue: UInt32 = read(from: 0x01...0x03)
        let value = Int32(rawValue & 0x80000 > 0 ? rawValue | 0xFFF00000 : rawValue)
        return Float(value>>4)/16.0
    }

    /// Get the current pressure from a running sensor.
    public var currentPressure: Float {
        let pressureDataReadyFlag: UInt8 = 0x04

        // start the altimeter, turn on the analog systems, ADC, and set oversampling rate to 128x, read as pressure
        blockingWriteControlReg1(value: 0x39)
        blockingWaitForStatusFlag(flag: pressureDataReadyFlag)

        let rawValue: UInt32 = read(from: 0x01...0x03)
        return Float(rawValue>>4)/4.0
    }

    /// Get the current temperature from a running sensor.
    public var currentTemperature: Float {
        let temperatureDataReadyFlag: UInt8 = 0x02

        // start the altimeter, turn on the analog systems, ADC, and set oversampling rate to 128x

        blockingWriteControlReg1(value: 0x39)
        blockingWaitForStatusFlag(flag: temperatureDataReadyFlag)

        let rawValue: UInt16 = read(from: 0x04...0x05)
        return Float(rawValue)/16.0
    }

    private func blockingWriteControlReg1(value: UInt8) {
        write(to: 0x26, value: value)
    }

    private func blockingWaitForStatusFlag(flag: UInt8) {
        var status: UInt8 = read(from: 0x0)
        while status & flag == 0 {
            delay(milliseconds: 10)
            status = read(from: 0x0)
        }
    }
}

extension MPL3115A2 {
    /// The aliases allow the STATUS register to be read easily before reading the current pressure/altitude or
    /// temperature data, the delta pressure/altitude or temperature data, or the FIFO data, using the register
    /// address auto-incrementing mechanism.
    public var sensorStatus: UInt8 {
        get { read(from: 0x0) }
    }

    /// The DR_STATUS register provides the acquisition status information on a per sample basis, and reflects
    /// real-time updates to the OUT_P and OUT_T registers. The same STATUS register can be read through an
    /// alternate address 00h (F_Mode = 00).
    public var dataReadyStatus: DataReadyStatus {
        get { read(from: DataReadyStatus.address) }
    }

    /// The pressure data is stored as a 20-bit unsigned integer with a fractional part. The OUT_P_MSB (01h),
    /// OUT_P_CSB (02h) and bits 7 to 6 of the OUT_P_LSB (03h) registers contain the integer part in Pascals.
    /// Bits 5 to 4 of OUT_P_LSB contain the fractional component. This value is representative as a
    /// Q18.2 fixed point format where there are 18 integer bits and two fractional bits.
    public var pressureData: Float? {
        get {
            guard control1.mode == .barometer else {
                return nil
            }
            let value: Int32 = read(from: 0x01...0x03) >> 4
            // let frac: UInt8 = value & 0x03
            // let int: Int32 = value >> 2
            return Float(value)/4.0
        }
    }

    /// The altitude data is stored as a 20-bit signed integer with a fractional part. The OUT_P_MSB (01h) and
    /// OUT_P_CSB (02h) registers contain the integer part in meters and the OUT_P_LSB (03h) register contains
    /// the fractional part. This value is represented as a Q16.4 fixed-point format where there are 16 integer
    /// bits (including the signed bit) and four fractional bits.
    public var altitudeData: Float? {
        get {
            guard control1.mode == .altimeter else {
                return nil
            }
            let rawValue: UInt32 = read(from: 0x01...0x03)
            let value = UInt32(rawValue & 0x80000 > 0 ? rawValue | 0xFFF00000 : rawValue)
            return Float(value>>4)/16.0
        }
    }

    public var temperatureData: UInt16 {
        get { read(from: 0x04...0x05) }
    }

    public struct DataReadyStatus: I2CMutableRegisterData {

        public static let address: UInt8 = 0x06

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// Pressure/altitude or temperature data overwrite. PTOW is set to `true` whenever new data is acquired
        /// before completing the retrieval of the previous set. This event occurs when the content of at least
        /// one data register (OUT_P, OUT_T) has been overwritten. PTOW is cleared when the high-bytes of the data
        /// (OUT_P_MSB or OUT_T_MSB) are read, when F_MODE is zero. PTOW is cleared by reading F_DATA register when
        /// F_MODE > 0.
        ///
        /// * `false` — No data overwrite has occurred (reset value)
        /// * `true` — Previous pressure/altitude or temperature data was overwritten by new pressure/altitude or
        ///   temperature data before it was read
        public var pressureTemperatureOverwrite: Bool {
            get { hasBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }

        /// Pressure/altitude data overwrite. POW is set to 1 whenever a new pressure/altitude acquisition is completed
        /// before the retrieval of the previous data. When this occurs the previous data is overwritten. POW is cleared
        /// anytime OUT_P_MSB register is read, when F_MODE is zero. POW is cleared by reading F_DATA register when F_MODE > 0.
        ///
        /// * `false` — No data overwrite has occurred (reset value)
        /// * `true` — Previous pressure/altitude data was overwritten by new pressure/altitude data before it was read
        public var pressureOverwrite: Bool {
            get { hasBit(at: 6) }
            set { setBit(at: 6, to: newValue) }
        }

        /// Temperature data overwrite. TOW is set to 1 whenever a new temperature acquisition is completed before the
        /// retrieval of the previous data. When this occurs the previous data is overwritten. TOW is cleared anytime
        /// OUT_T_MSB register is read, when F_MODE is zero. TOW is cleared by reading F_DATA register when F_MODE > 0.
        ///
        /// * `false` — No data overwrite has occurred (reset value)
        /// * `true` — Previous temperature data was overwritten by new temperature data before it was read
        public var temperatureOverwrite: Bool {
            get { hasBit(at: 5) }
            set { setBit(at: 5, to: newValue) }
        }

        /// Pressure/altitude or temperature data ready. PTDR signals that a new acquisition for either pressure/altitude
        /// or temperature is available. PTDR is cleared anytime OUT_P_MSB or OUT_T_MSB register is read, when F_MODE is zero.
        /// PTDR is cleared by reading F_DATA register when F_MODE > 0.
        ///
        /// * `false` — No new set of data ready (reset value)
        /// * `true` — A new set of data is ready
        public var pressureTemperatureDataReady: Bool {
            get { hasBit(at: 3) }
            set { setBit(at: 3, to: newValue) }
        }

        /// New pressure/altitude data available. PDR is set to 1 whenever a new pressure/altitude data acquisition is completed.
        /// PDR is cleared anytime OUT_P_MSB register is read, when F_MODE is zero. PDR is cleared by reading F_DATA register
        /// when F_MODE > 0.
        ///
        /// * `false` — No new pressure/altitude data is available (reset value)
        /// * `true` — A new set of pressure/altitude data is ready
        public var pressureDataReady: Bool {
            get { hasBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        /// New temperature data available. TDR is set to 1 whenever a temperature data acquisition is completed. TDR is cleared
        /// anytime OUT_T_MSB register is read, when F_MODE is zero. TDR is cleared by reading F_DATA register when F_MODE > 0.
        ///
        /// * `false` — No new temperature data ready (reset value)
        /// * `true` — A new temperature data is ready
        public var temperatureDataReady: Bool {
            get { hasBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }
    }
}

extension MPL3115A2 {

    public var sensorData: SensorData {
        get { read(from: SensorData.address) }
        set { write(to: SensorData.address, value: newValue) }
    }
    public struct SensorData: I2CMutableRegisterData {

        public static let address: UInt8 = 0x13

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// Enables `dataReadyEventMode`, along with the `pressureDataReadyFlag` and `temperatureDataReadyFlag`.
        public mutating func enableAllFlags() {
            registerValue = 0x07
        }

        /// Data ready event mode.
        ///
        /// * 0 — Event detection disabled (reset value) If the DREM bit is cleared logic '0' and one or
        ///   more of the data ready event flags are enabled, then an event flag will be raised whenever
        ///   the system acquires a new set of data.
        /// * 1 — Generate data ready event flag on new pressure/altitude or temperature data. If the
        ///   DREM bit is set logic '1' and one or more of the data ready event flags (PDEFE, TDEFE) are
        ///   enabled, then an event flag will be raised upon change in state of the data.
        public var dataReadyEventMode: Enabled {
            get { getBit(at: 0x02) }
            set { setBit(at: 0x02, to: newValue) }
        }

        /// Data event flag enable on new pressure/altitude
        public var pressureAltitudeFlag: Enabled {
            get { getBit(at: 0x01) }
            set { setBit(at: 0x01, to: newValue) }
        }

        /// Data event flag enable on new temperature data
        public var temperatureFlag: Enabled {
            get { getBit(at: 0x00) }
            set { setBit(at: 0x00, to: newValue) }
        }
    }
}

extension MPL3115A2 {

    /// The mode that the MPL3115A2 is operating in.
    public enum Mode: UInt8 {
        case barometer = 0x00
        case altimeter = 0x01
    }

    public enum OversampleRatio: UInt8 {
        case x1 = 0x00
        case x2 = 0x01
        case x4 = 0x02
        case x8 = 0x03
        case x16 = 0x04
        case x32 = 0x05
        case x64 = 0x06
        case x128 = 0x07
    }

    /// The `CTRL_REG1` register.
    public var control1: Control1 {
        get { read(from: Control1.address) }
        set { write(to: Control1.address, value: newValue) }
    }

    /// The `CTRL_REG2` register.
    public var control2: Control2 {
        get { read(from: Control2.address) }
        set { write(to: Control2.address, value: newValue) }
    }

    /// The `CTRL_REG3` register - interrupt control.
    public var interrupt: Interrupt {
        get { read(from: Interrupt.address) }
        set { write(to: Interrupt.address, value: newValue) }
    }

    /// The `CTRL_REG4` register - interrupt enable control.
    public var interruptEnable: InterruptEnable {
        get { read(from: InterruptEnable.address) }
        set { write(to: InterruptEnable.address, value: newValue) }
    }

    /// The `CTRL_REG5` register - interrupt configuration.
    public var interruptConfig: InterruptConfig {
        get { read(from: InterruptConfig.address) }
        set { write(to: InterruptConfig.address, value: newValue) }
    }

    public struct Control1: I2CMutableRegisterData {

        public static let address: UInt8 = 0x26

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        public init(
            mode: Mode = .barometer,
            oversampleRatio: OversampleRatio = .x1,
            softwareResetting: Bool = false,
            oneShotTest: Bool = false,
            active: Bool = false
        ) {
            self.registerValue = 0x00

            self.mode = mode
            self.oversampleRatio = oversampleRatio
            self.softwareResetting = softwareResetting
            self.oneShotTest = oneShotTest
            self.active = active
        }

        /// This bit is sets the mode to ACTIVE, where the system will make measurements at periodic times based
        /// on the value of ST bits.
        public var active: Bool {
            get { hasBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }

        /// OST bit will initiate a measurement immediately. If the SBYB bit is set to active,
        /// setting the OST bit will initiate an immediate measurement, the part will then return
        /// to acquiring data as per the setting of the ST bits in CTRL_REG2. In this mode, the OST
        /// bit does not clear itself and must be cleared and set again to initiate another immediate measurement.
        ///
        /// In one-shot mode, when SBYB is 0, the OST bit is an auto-clear bit. When OST is set, the device initiates
        /// a measurement by going into active mode. Once a pressure/altitude and temperature measurement is completed,
        /// it clears the OST bit and comes back to STANDBY mode. User shall read the value of the OST bit before
        /// writing to this bit again.
        public var oneShotTest: Bool {
            get { hasBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// Software reset. This bit is used to activate the software reset. The boot mechanism can be enabled in STANDBY and ACTIVE mode.
        ///
        /// When the boot bit is enabled the boot mechanism resets all functional block registers and loads the respective internal registers with default values.
        ///
        /// If the system was already in STANDBY mode, the reboot process will immediately begin, or else if the system was in ACTIVE mode, the boot mechanism will automatically transition the system from ACTIVE mode to STANDBY mode. Only then can the reboot process begin. The I2C communication system is reset to avoid accidental corrupted data access.
        ///
        /// At the end of the boot process the RST bit is de-asserted to 0. Reading this bit will return a value of zero.
        public var softwareResetting: Bool {
            get { hasBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        /// These bits select the oversampling ratio. Value is 2`OS`. The default value is `000`` for a ratio of `1`.
        public var oversampleRatio: OversampleRatio {
            get { getBits(from: 3...5) }
            set { setBits(from: 3...5, to: newValue) }
        }

        /// Altimeter/barometer mode.
        public var mode: Mode {
            get { getBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }
    }

    public enum AlarmSelector: UInt8 {
        case target = 0x00
        case source = 0x01
    }

    public struct Control2: I2CMutableRegisterData {

        public static let address: UInt8 = 0x27

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// Auto acquisition time step.
        /// Step value is 2ST — Giving a range of 1 second to 215 seconds (9 hours).
        public var autoAcquisitionTimeStep: UInt8 {
            get { getBits(from: 0...3) }
            set { setBits(from: 0...3, to: newValue) }
        }

        /// The bit selects the target value for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH.
        ///
        /// * 0 — (reset value) The values in P_TGT_MSB, P_TGT_LSB and T_TGT are used.
        /// * 1 — The values in OUT_P/OUT_T are used for calculating the interrupts SRC_PW/SRC_TW and SRC_PTH/SRC_TTH.
        public var alarmSelector: AlarmSelector {
            get { getBit(at: 4) }
            set { setBit(at: 4, to: newValue) }
        }

        /// This is to load the target values for `SRC_PW`/`SRC_TW` and `SRC_PTH`/`SRC_TTH`.
        ///
        /// * 0 — Do not load `OUT_P`/`OUT_T` as target values (reset value)
        /// * 1 — The next values of `OUT_P`/`OUT_T` are used to set the target values for the interrupts.
        ///
        /// Notes:
        /// * This bit must be set at least once if `alarmSelector` is set to `source`.
        /// * To reload the next `OUT_P`/`OUT_T` as the target values, clear and set again.
        public var loadOutput: Bool {
            get { hasBit(at: 5) }
            set { setBit(at: 5, to: newValue) }
        }
    }

    public enum InterruptActive: UInt8 {
        case low = 0
        case high = 1
    }

    public enum InterruptMode: UInt8 {
        case internalPullup = 0
        case openDrain = 1
    }

    public struct Interrupt: I2CMutableRegisterData {

        public static let address: UInt8 = 0x28

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// The IPOL bit selects the polarity of the interrupt signal. When IPOL is '0' (default value)
        /// any interrupt event will signalled with a logical `'0'`. Interrupt Polarity active high,
        /// or active low on interrupt pad INT1.
        ///
        /// * 0 — Active low (reset value)
        /// * 1 — Active high
        public var interrupt1Active: InterruptActive {
            get { getBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }

        /// This bit configures the interrupt pin to push-pull or in open drain mode. The default value is
        /// 0 which corresponds to push-pull mode. The open drain configuration can be used
        /// for connecting multiple interrupt signals on the same interrupt line. push-pull/open drain
        /// selection on interrupt pad INT1.
        ///
        /// * 0 — Internal pullup (reset value)
        /// * 1 — Open drain
        public var interrupt1Mode: InterruptMode {
            get { getBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// Interrupt polarity active high, or active low on interrupt pad INT2.
        ///
        /// * 0 — Active low (reset value)
        /// * 1 — Active high
        public var interrupt2Active: InterruptActive {
            get { getBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// Push-pull/open drain selection on interrupt pad INT2.
        /// * 0 — Internal pullup (reset value)
        /// * 1 — Open drain
        public var interrupt2Mode: InterruptMode {
            get { getBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }
    }

    public struct InterruptEnable: I2CMutableRegisterData {

        public static let address: UInt8 = 0x29

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// Temperature change interrupt
        public var temperatureChange: Enabled {
            get { getBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }

        /// Pressure change interrupt
        public var pressureChange: Enabled {
            get { getBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// Pressure threshold interrupt
        public var pressureThreshold: Enabled {
            get { getBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        /// Temperature threshold interrupt
        public var temperatureThreshold: Enabled {
            get { getBit(at: 3) }
            set { setBit(at: 3, to: newValue) }
        }

        /// Temperature window interrupt
        public var temperatureWindow: Enabled {
            get { getBit(at: 4) }
            set { setBit(at: 4, to: newValue) }
        }

        /// Pressure window interrupt
        public var pressureWindow: Enabled {
            get { getBit(at: 5) }
            set { setBit(at: 5, to: newValue) }
        }

        /// FIFO interrupt
        public var fifo: Enabled {
            get { getBit(at: 6) }
            set { setBit(at: 6, to: newValue) }
        }

        /// Data ready interrupt
        public var dataReady: Enabled {
            get { getBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }
    }

    public enum InterruptSelector: UInt8 {
        case int2 = 0
        case int1 = 1
    }

    public struct InterruptConfig: I2CMutableRegisterData {

        public static let address: UInt8 = 0x2A

        public var registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// Temperature change interrupt
        public var temperatureChange: InterruptSelector {
            get { getBit(at: 0) }
            set { setBit(at: 0, to: newValue) }
        }

        /// Pressure change interrupt
        public var pressureChange: InterruptSelector {
            get { getBit(at: 1) }
            set { setBit(at: 1, to: newValue) }
        }

        /// Temperature threshold interrupt
        public var temperatureThreshold: InterruptSelector {
            get { getBit(at: 2) }
            set { setBit(at: 2, to: newValue) }
        }

        /// Pressure threshold interrupt
        public var pressureThreshold: InterruptSelector {
            get { getBit(at: 3) }
            set { setBit(at: 3, to: newValue) }
        }

        /// Temperature window interrupt
        public var temperatureWindow: InterruptSelector {
            get { getBit(at: 4) }
            set { setBit(at: 4, to: newValue) }
        }

        /// Pressure window interrupt
        public var pressureWindow: InterruptSelector {
            get { getBit(at: 5) }
            set { setBit(at: 5, to: newValue) }
        }

        /// FIFO interrupt
        public var fifo: InterruptSelector {
            get { getBit(at: 6) }
            set { setBit(at: 6, to: newValue) }
        }

        /// Data ready interrupt
        public var dataReady: InterruptSelector {
            get { getBit(at: 7) }
            set { setBit(at: 7, to: newValue) }
        }
    }
}

extension MPL3115A2 {

    /// Pressure user accessible offset trim value expressed as an 8-bit, 2's complement number.
    /// The user offset registers may be adjusted to enhance accuracy and optimize the system performance.
    /// Range is from −512 to +508 Pa, 4 Pa/LSB.
    public var offsetPressureCorrection: Int8 {
        get { read(from: 0x2B) }
        set { write(to: 0x2B, value: newValue) }
    }

    /// Temperature user accessible offset trim value expressed as an 8-bit, 2's complement number.
    /// The user offset registers may be adjusted to enhance accuracy and optimize the system performance.
    /// Range is from −8 to +7.9375 °C, 0.0625 °C/LSB.
    public var offsetTemperatureCorrection: Int8 {
        get { read(from: 0x2C) }
        set { write(to: 0x2C, value: newValue) }
    }

    /// Altitude data user offset register (OFF_H) is expressed as a 2's complement number
    /// in meters. See Section 9.1.3 "Pressure/altitude". The user offset register provides
    /// user adjustment to the vertical height of the altitude output. The range of values are from −128 to +127 meters.
    public var offsetAltitude: Int8 {
        get { read(from: 0x2D) }
        set { write(to: 0x2D, value: newValue) }
    }
}

/* Snippets:
 {
        "MPL3115A2":[

            {"partName":"Setup MPL3115A2",
                "partCode":"// setup I2C with standard parameters\nsetupI2C(speed: 0x47, premultiplier: 0)\n\n// set up the device\nlet mlp3115a2 = MLP3115A2()\n\n// enable the MPL3115A2 sensor for reading oversampled 128x\nwhile (!mlp3115a2.ready) {}\nmlp3115a2.begin()"
            },

            {"partName":"Get temperature reading",
                "partCode":"let temp: Float = mlp3115a2.currentTemperature"
            },

            {"partName":"Get altitude reading",
                "partCode":"let alt: Float = mlp3115a2.currentAltitude"
            },

            {"partName":"Get pressure reading",
                "partCode":"let press: Float = mlp3115a2.currentPressure"
            }
        ]
 }
*/
