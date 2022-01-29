// Author: David Peterson
// Date: 29/11/2020
// IDE Version: 4.5
// Description: This library provides basic configuration for motor shields which have speed and direction pins.

import AVR

//------------------------------------------------------------------------------
// Motor Shield Configuration
//------------------------------------------------------------------------------

/// Describes the pin configuration for motors.
public struct Motor {
    /// The analogue pin controlling speed
    public let speedPin: Pin

    /// The digital pin indicating direction.
    public let directionPin: Pin

    /// The `HIGH`/`LOW` value for 'forward' on this motor.
    public let forwardValue: Bool

    /// The `HIGH`/`LOW` value for 'backward' on this motor.
    public var backwardValue: Bool {
        !forwardValue
    }

    /// Setting this to `true` will configure the pins with the current direction/speed values. Setting it to `false` will stop the motor if it is running.
    public var enabled: Bool = false {
        didSet {
            guard enabled else {
                analogWrite(pin: speedPin, value: 0)
                return
            }

            guard enabled != oldValue else {
                return
            }

            pinMode(pin: directionPin, mode: OUTPUT)
            digitalWrite(pin: directionPin, value: directionValue)
            pinMode(pin: speedPin, mode: OUTPUT)
            analogWrite(pin: speedPin, value: speed)
        }
    }

    /// Sets the speed to run the motor at. If `enabled` the change is updated on the motor immediately.
    public var speed: UInt8 = 0 {
        didSet {
            guard enabled else { return }

            analogWrite(pin: speedPin, value: speed)
        }
    }

    /// Sets the direction to run the motor at. If `enabled` the change is updated on the motor immediately.
    public var direction: Direction = Motor.Direction.forward {
        didSet {
            guard enabled else { return }

            digitalWrite(pin: directionPin, value: directionValue)
        }
    }

    /// Returns the `direction` as a `Bool` value for `digitalWrite`.
    private var directionValue: Bool {
        switch direction {
        case .forward:
            return forwardValue
        case .backward:
            return backwardValue
        }
    }

    /// The direction of the motor.
    public enum Direction {
        case forward
        case backward
    }
}

