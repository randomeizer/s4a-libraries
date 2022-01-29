// Author: David Peterson
// Date: 28/01/2021
// IDE Version: 4.5
// Description: This library allows you to read altitude, pressure and temperature
// on an MPL3115A2 sensor via the I2C bus.
// Based on Carl Peto's MPL3115A2 example library.

import AVR

public struct MPL3115A2: I2CSlaveNode {
    public let address: UInt8 = 0x60 // or is it 0xC0?
}

extension MPL3115A2 {
    /// Once I2C has been set up, check that the sensor is available and connected.
    public var ready: Bool {
        let whoami: UInt8 = read(from: 0x0C)
        return whoami == 0xC4
    }

    /// Waits for the device to be `ready`, then setup standard flags.
    public func begin() {
        while !ready {
            delay(milliseconds: 10)
        }
        write(to: 0x13, value: 0x07) // set all flags enabled for data retrieval
    }

    /// Get the current altitude from a running sensor.
    public var currentAltitude: Float {
        let altitudeDataReadyFlag: UInt8 = 0x08

        // start the altimeter, turn on the analog systems, ADC, and set oversampling rate to 128x, read as altitude
        blockingWriteControlReg1(value: 0xB9)
        blockingWaitForStatusFlag(flag: altitudeDataReadyFlag)

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
