// Author: David Peterson
// Date: 28/11/2020
// IDE Version: 4.5
// Description: This library adds support for the MPU-6050 to run self-tests on the accelerometer/gyro. It requires the main MPU6050 module to be installed.

import AVR

private func pow(_ a: Float, _ ex: Float) -> Float {
    // TODO: Figure out how to calculate this.
    fatalError()
}


/// Provides read-only access to the self-test results.
extension MPU6050 {

    /// Provides read-only access to a `SELF_TEST_X|Y|Z` value.
    fileprivate struct SelfTestXYZ: I2CRegisterData {
        let registerValue: UInt8

        init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        /// The high 3 bits of the `X|Y|ZA_TEST` value. The low 2 bits come from `SelfTestA`
        var accelHigh: UInt8 { getBits(from: 5...7) }

        /// The full 5 bits of the `X|Y|ZG_TEST` value.
        var gyro: UInt8 { getBits(from: 0...4) }
    }

    /// Provides read-only access to the `SELF_TEST_A` value, which provides the bottom 2 bits for `X|Y|ZA_TEST`
    fileprivate struct SelfTestA: I2CRegisterData {
        public let registerValue: UInt8

        public init(registerValue: UInt8) {
            self.registerValue = registerValue
        }

        var xLow: UInt8 { getBits(from: 0...1) }
        var yLow: UInt8 { getBits(from: 2...3) }
        var zLow: UInt8 { getBits(from: 4...5) }
    }

    fileprivate struct SelfTestResults {
        private let selfTestX: SelfTestXYZ
        private let selfTestY: SelfTestXYZ
        private let selfTestZ: SelfTestXYZ
        private let selfTestA: SelfTestA

        /// A five-bit unsigned integer.
        var accelX: UInt8 { selfTestX.accelHigh << 2 | selfTestA.xLow }

        /// A five-bit unsigned integer.
        var accelY: UInt8 { selfTestY.accelHigh << 2 | selfTestA.yLow }

        /// A five-bit unsigned integer.
        var accelZ: UInt8 { selfTestZ.accelHigh << 2 | selfTestA.zLow }

        /// A five-bit unsigned integer.
        var gyroX: UInt8 { selfTestX.gyro }

        /// A five-bit unsigned integer.
        var gyroY: UInt8 { selfTestY.gyro }

        /// A five-bit unsigned integer.
        var gyroZ: UInt8 { selfTestY.gyro }
    }

    /// Provides read-only access to the results from the most recent self-test, if available.
    fileprivate var selfTestResults: SelfTestResults {
        read(from: 0x0D...0x10)
    }
}

/// This register is used to trigger gyroscope self-test.
///
/// Gyroscope self-test permits users to test the mechanical and electrical portions of the gyroscope.
/// The self-test for each gyroscope axis can be activated by controlling the `testX`, `testY`, and `testZ`
/// bits of this register. Self-test for each axis may be performed independently or all at the same time.
///
/// When self-test is activated, the on-board electronics will actuate the appropriate sensor.
/// This actuation will move the sensor’s proof masses over a distance equivalent to a pre-defined Coriolis force.
/// This proof mass displacement results in a change in the sensor output, which is reflected in the output signal.
/// The output signal is used to observe the self-test response.
///
/// The self-test response is defined as follows:
///
///     Self-test response = Sensor output with self-test enabled – Sensor output without selftest enabled
///
/// The self-test limits for each gyroscope axis is provided in the electrical characteristics tables of the
/// MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within the
/// min/max limits of the product specification, the part has passed self test. When the self-test response
/// exceeds the min/max values specified in the document, the part is deemed to have failed self-test.
extension GyroConfig {
    /// Setting this to `true` causes the X axis gyroscope to perform self test.
    private var testX: Bool {
        get { hasBit(at: 7) }
        set { setBit(at: 7, to: newValue) }
    }

    /// Setting this to `true` causes the Y axis gyroscope to perform self test.
    private var testY: Bool {
        get { hasBit(at: 6) }
        set { setBit(at: 6, to: newValue) }
    }

    /// Setting this to `true` causes the Z axis gyroscope to perform self test.
    private var testZ: Bool {
        get { hasBit(at: 5) }
        set { setBit(at: 5, to: newValue) }
    }

    /// Initializes the gyros to run the self-test. Each axis can be tested independently, or all at once.
    /// Test results are available via `selfTestResults`.
    ///
    /// - Parameter testX: If `true` (the default), the X axis gyro will be tested.
    /// - Parameter testY: If `true` (the default), the Y axis gyro will be tested.
    /// - Parameter testZ: If `true` (the default), the Z axis gyro will be tested.
    ///
    /// - Note: The gyro FSR will be set to ± 250 ˚/s.
    fileprivate init(testX: Bool = true, testY: Bool = true, testZ: Bool = true) {
        self.init(registerValue: 0)
        self.testX = testX
        self.testY = testY
        self.testZ = testZ
        self.fullScaleRange = .fsr250DPS
    }
}

/// This register is used to trigger accelerometer self test and configure the accelerometer full scale range.
/// This register also configures the Digital High Pass Filter (DHPF).
///
/// Accelerometer self-test permits users to test the mechanical and electrical portions of the accelerometer.
/// The self-test for each accelerometer axis can be activated by controlling the `testX`, `testY`, and `testZ`
/// fields of this register. Self-test for each axis may be performed independently or all at the same time.
///
/// When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation
/// simulates an external force. The actuated sensor, in turn, will produce a corresponding output signal.
/// The output signal is used to observe the self-test response.
///
/// The self-test response is defined as follows:
///
///     Self-test response = Sensor output with self-test enabled – Sensor output without self-test enabled
///
/// The self-test limits for each accelerometer axis is provided in the electrical characteristics tables of
/// the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within
/// the min/max limits of the product specification, the part has passed self test. When the selftest response
/// exceeds the min/max values specified in the document, the part is deemed to have failed self-test.
extension AccelConfig {

    /// Initializes the accelerometers to run the self-test. Each axis can be tested independently, or all at once.
    /// Test results are available via `selfTestResults`.
    ///
    /// - Parameter testX: If `true` (the default), the X axis accelerometer will be tested.
    /// - Parameter testY: If `true` (the default), the Y axis accelerometer will be tested.
    /// - Parameter testZ: If `true` (the default), the Z axis accelerometer will be tested.
    ///
    /// - Note: The accelerometer FSR will be set to ± 8g.
    fileprivate init(testX: Bool = false, testY: Bool = false, testZ: Bool = false) {
        registerValue = 0
        self.testX = testX
        self.testY = testY
        self.testZ = testZ
        self.fullScaleRange = .fsr8g
    }

    var testX: Bool {
        get { hasBit(at: 7) }
        set { setBit(at: 7, to: newValue) }
    }

    var testY: Bool {
        get { hasBit(at: 6) }
        set { setBit(at: 6, to: newValue) }
    }

    var testZ: Bool {
        get { hasBit(at: 5) }
        set { setBit(at: 5, to: newValue) }
    }
}

extension MPU6050 {
    /// Struct used to send a gyro+accelerometer self test command in one operation.
    fileprivate struct SelfTestConfig {
        var gyroConfig: GyroConfig
        var accelConfig: AccelConfig

        public init(
            gyro: (x: Bool, y: Bool, z: Bool) = (x: true, y: true, z: true),
            accel: (x: Bool, y: Bool, z: Bool) = (x: true, y: true, z: true)
        ) {
            gyroConfig = GyroConfig(testX: gyro.x, testY: gyro.y, testZ: gyro.z)
            accelConfig = AccelConfig(testX: accel.x, testY: accel.y, testZ: accel.z)
        }
    }

    /// Sets the `gyroConfig` and `accelConfig` in a single operation, configured to run the self-test with correct FSR settings.
    fileprivate var selfTestConfig: SelfTestConfig {
        get { read(from: 0x1B...0x1C) }
        set { write(to: 0x1B...0x1C, value: newValue) }
    }
}

private func selfTestResult(value: UInt8, factoryTrim: Float) -> Float {
    return (Float(value) - factoryTrim)/factoryTrim
}

private func selfTestGyroFT(gyroValue: UInt8) -> Float {
    guard gyroValue != 0 else {
        return 0.0
    }
    return (25.0*131.0)*(pow(1.046, (Float(gyroValue) - 1.0)))
}

private func selfTestAccFT(accValue: UInt8) -> Float {
    guard accValue == 0 else {
        return 0.0
    }
    return (4096.0*0.34)*(pow((0.92/0.34), (Float(accValue) - 1.0)/30.0))
}

extension MPU6050 {

    /// Accelerometer and gyroscope self test; check calibration wrt factory settings
    /// Should return percent deviation from factory trim values, `+/-14%` or less deviation is a pass.
    ///
    /// - Returns: a tuple containing the the `accel` results as an `(x:,y:,z:)` tuple of `Float`s,
    ///     and the `gyro` results, also as an `(x:,y:,z:)` tuple of `Float`s.
    ///     All results percentages, scaled from `0.0` to `1.0`, and should be `0.14` or less to pass.
    mutating public func selfTest() -> (accel: (x: Float, y: Float, z: Float), gyro: (x: Float, y: Float, z: Float) ) {

        // Configure the accelerometer for self-test
        selfTestConfig = SelfTestConfig()

        // Delay a while to let the device execute the self-test
        delay(milliseconds: 250)

        let results = selfTestResults

        // Process results to allow final comparison with factory set values
        let ftAccX = selfTestAccFT(accValue: results.accelX)      // FT[Xa] factory trim calculation
        let ftAccY = selfTestAccFT(accValue: results.accelY)      // FT[Ya] factory trim calculation
        let ftAccZ = selfTestAccFT(accValue: results.accelZ)      // FT[Za] factory trim calculation
        let ftGyroX = selfTestGyroFT(gyroValue: results.gyroX)    // FT[Xg] factory trim calculation
        let ftGyroY = -selfTestGyroFT(gyroValue: results.gyroY)   // FT[Yg] factory trim calculation
        let ftGyroZ = selfTestGyroFT(gyroValue: results.gyroZ)    // FT[Zg] factory trim calculation

        //  Output self-test results and factory trim calculation if desired
        //  print(accX); print(accY); print(accZ);
        //  print(gyroX); print(gyroY); print(gyroZ);
        //  print(ftAccX); print(ftAccY); print(ftAccZ);
        //  print(ftGyroX); print(ftGyroY); print(ftGyroZ);

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get to percent, must multiply by 100 and subtract result from 100
        return (
            accel: (
                x: selfTestResult(value: results.accelX, factoryTrim: ftAccX),
                y: selfTestResult(value: results.accelY, factoryTrim: ftAccY),
                z: selfTestResult(value: results.accelZ, factoryTrim: ftAccZ)
            ),
            gyro: (
                x: selfTestResult(value: results.gyroX, factoryTrim: ftGyroX),
                y: selfTestResult(value: results.gyroY, factoryTrim: ftGyroY),
                z: selfTestResult(value: results.gyroZ, factoryTrim: ftGyroZ)
            )
        )
    }
}
