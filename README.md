# Swift For Arduino Libraries

A collection of [Swift For Arduino](https://www.swiftforarduino.com) (S4A) libraries developed
to make life simpler for certain tasks. No warranty provided.

## Logger

The [Logger](Libraries/Logger.swift) provides a wrapper over `AVR.print(...)` to make life simpler
for enabling/disabling debug printing, or for outputting plotted data at a controlled rate.

Example usage:

```swift
let debug = Logger(active: false, period: 0)
let plotter = Logger(active: true, period: 100)

debug.println("Only output when debugging")

var count = 0

while(true) {
    // do stuff
    count = count + 1
    delay(10)
    // only output every 10th time (roughly)
    plotter("count", count)
}
```

## I2C Extensions

The [I2CExtensions](I2C/I2CExtensions.swift) library makes it (relatively) easy to define an `I2C` device interface.

The basic concept is that you define a `struct` which represents an I2C device. The root device is the `I2CSlaveNode`. It only requires an `address` to be defined, which is generally one or more hard-coded values for the specific device.

For example:

```swift
public struct MPL3115A2: I2CSlaveNode {
    public let address: UInt8 = 0x60
}
```

This is enough to initialize the device structure, pointing it at the specified address (A `UInt8` of `0x60` in this case). Note that it is a constant (via `let`.)

Some devices allow more than one possible address, to enable having duplicate devices in the I2C chain. For example, the `MPU6050` 6-axis gyro has two possible addresses. This is catered for by using an `enum` like so:

```swift
public struct MPU6050: I2CSlaveNode {
    /// Specify which MPU6050 unit this is, if there is more than one.
    /// The actual unit of the module is defined by the logic state of the `AD0` (pin 9).
    public enum Address: UInt8, I2CSlaveAddress {
        /// Address when `AD0` pin is set to `0`.
        case zero = 0b110_100_0

        /// Address when `AD0` pin is set to `1`.
        case one  = 0b110_100_1
    }

    /// Indicates which unit this is, if more than one MPU6050 is configured.
    public let address: Address

    ...
}
```

You can then provide computed properties or functions which use several lower-level functions to access registers. THey are listed in the [I2CExtensions.swift](I2C.swift4a/I2CExtensions.swift) file. Key functions are:

* `let value: UInt8 = read(from: 0x00)`: Reads a single [I2CRegisterValue](I2C.swift4a/I2CExtensions.swift) at the specified address.
* `let sixteenBits: UInt16 = read(from: 0x01..0x02)`: Reads two or more values into a (hopefully) large enough value type.
* `write(to: 0x00, 7)`: Writes a single `I2CRegisterValue` at the specified registry address.
* `write(to: 0x01..0x04, value: Int32(4))`: Writes a single multi-byte value to the specified range of registry addresses.

These will generally not be used directly when operating the device. Instead, computed properties and functions will call them. For example:

```swift
extension MPU6050 {
    public var sampleRateDivider: UInt8 {
        get { read(from: 0x19) }
        set { write(to: 0x19, value: newValue) }
    }
}
```

If the register is made up of smaller flags or data sequences, you can create a `struct` that conforms to either [I2CRegisterValue](I2C.swift4a/I2CExtensions.swift) if it's read-only, or an [I2CMutableRegisterValue](I2C.swift4a/I2CExtensions.swift) if it's read-write. These two protocols provide their own low-level functions, in particular:

* `hasBit(at:) -> Bool`: Checks if the register has the specified bit number set to `1`.
* `getBit(at:) -> UInt8`: Returns `1` or `0` for the specific bit number.
* `getBits(from:) -> UInt8`: Returns a single `UInt8` with all bit values applied.
* `setBit(at:,to:)`: Attempts to set the provided `value` to the register at the specified bit number, without changing the existing value.
* `setBits(from:) -> T`: Attempts to read from multiple adjacent registers and set the provided value.

Again, these are generally not accessed directly. Instead, wrap the function in the new struct:

```swift
extension MPU6050 {
    public struct Control: I2CMutableRegisterBlock {
        public var register1: UInt8
        public var register2: UInt8

        // ...

        public var active: Bool {
            get { register1.hasBit(at: 0) }
            set { register1.setBit(at: 0, to: newValue) }
        }
    }

    /// The `CTRL_REG1` and `CTRL_REG2` registers.
    public var control: Control {
        get { read(from: 0x26...0x27) }
        set { write(to: 0x26...0x27, value: newValue) }
    }
}
```

Take a look at the [MPL3115A2](I2C.swift4a/MPL3115A2.swift) and [MPU6050](I2C.swift4a/MPU6050.swift) files for more in-depth examples.

Once this is done, you will typically want to provide some user-facing functions that make it easier to perform specific tasks on the device.
