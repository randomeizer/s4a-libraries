import AVR

/// Represents the DFR Quad Motor Shield.
enum DFRQuadMotorShield {
    /// Returns a `Motor` configured to run the `M1` motor.
    static var m1: Motor {
        Motor(speedPin: 3, directionPin: 4, forwardValue: LOW)
    }

    /// Returns a `Motor` configured to run the `M2` motor.
    static var m2: Motor {
        Motor(speedPin: 11, directionPin: 12, forwardValue: HIGH)
    }

    /// Returns a `Motor` configured to run the `M3` motor.
    static var m3: Motor {
        Motor(speedPin: 5, directionPin: 8, forwardValue: LOW)
    }

    /// Returns a `Motor` configured to run the `M4` motor.
    static var m4: Motor {
        Motor(speedPin: 6, directionPin: 7, forwardValue: HIGH)
    }
}
