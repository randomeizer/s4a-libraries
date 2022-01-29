// Author: David Peterson
// Date: 29/11/2020
// IDE Version: 4.5
// Description: This library provides extensions to basic I2C functions to allow for `struct` values to be created
//              that communicate with specific, well-defined I2C-complient devices.

import AVR

/// Represents a 7-bit I2C Slave Address
public protocol I2CSlaveAddress {
    /// The raw 7-bit I2C slave address as a `UInt8`. The lowest 7 bits contain the address.
    var addressValue: UInt8 { get }
}

// Allows any `RawRepresentable` type with a `RawValue` of `UInt8` to conform to `I2CSlaveAddress`.
// This includes `enum` values.
extension I2CSlaveAddress where Self: RawRepresentable, Self.RawValue == UInt8 {
    /// The address value for `RawRepresentable` instances whose `RawValue` is `UInt8`.
    public var addressValue: UInt8 {
        return rawValue
    }
}

/// Allow `UInt8` values to be `I2CSlaveAddress` values.
extension UInt8: I2CSlaveAddress {
    public var addressValue: UInt8 { self }
}

/// Represents a connection to an I2C slave node (a.k.a. a device attached to the I2C bus).
/// It has an `I2CSlaveAddress` and provides support methods for reading and writing to the device at that address.
public protocol I2CSlaveNode {
    associatedtype Address: I2CSlaveAddress

    /// The slave address for the device.
    var address: Address { get }
}

// Adds support methods for read/write to the slave node.
public extension I2CSlaveNode {
    /// Reads sequential bytes from the register range, returning access to them via an `UnsafeRawBufferPointer`.
    /// Not all I2C registers are necessarily contiguous, but if it's supported by the device, the block of bytes is
    /// copied into the AVR buffer and pointed at by the resulting `UnsafeRawBufferPointer`.
    ///
    /// - Parameter start: The initial register to retrieve bytes from.
    /// - Parameter count: The number of contiguous registers to read from.
    /// - Returns: An `UnsafeRawBufferPointer` that points at the retrieved block of memory.
    func readBytes(from start: UInt8, count: UInt8) -> UnsafeRawBufferPointer {
        guard count > 0 else {
            return UnsafeRawBufferPointer(start: nil, count: 0)
        }
        let pointer = blockingReadMultipleI2CRegisters(slaveAddress: address.addressValue, registerStart: start, registerCount: count)
        return UnsafeRawBufferPointer(pointer)
    }

    /// Writes sequential bytes from the `UnsafeRawBufferPointer` to the series of registers starting with `start`.
    /// The total number of registers written to will be the `count` of the `UnsafeRawBufferPointer`.
    ///
    /// - Parameter start: The starting register. The first byte of the pointer data will be written here.
    /// - Parameter value: The buffer of bytes to write.
    func writeBytes(to start: UInt8, value: UnsafeRawBufferPointer) {
        for (i, v) in value.enumerated() {
            blockingWriteSingleI2CRegister(slaveAddress: address.addressValue, register: start + UInt8(i), value: v)
        }
    }

    /// Reads a single `UInt8` value from the specified register.
    ///
    /// - Parameter register: The register number to read from.
    /// - Returns: The value as a `UInt8` byte.
    func read<T: I2CRegisterData>(from register: UInt8) -> T {
        let value = blockingReadSingleI2CRegister(slaveAddress: address.addressValue, register: register)
        return T(registerValue: value)
    }

    /// Reads the number of registers required to fill out the `T` type value.
    ///
    /// - Parameter registers: The range of register numbers to read from.
    /// - Parameter type: The `Type` being read. Will attempt to infer this automatically, but in some contexts it must be provided explicitly.
    /// - Returns: An instance of the type `T`, initialized from the required number of bytes for the type.
    /// - Throws: Precondition failure if the range of registers does not match the size of the type being returned.
    func read<T>(from registers: ClosedRange<UInt8>, as type: T.Type = T.self) -> T {
        let registersCount = registers.upperBound - registers.lowerBound
        let count = UInt8(MemoryLayout<T>.stride)
        precondition(registersCount == count)

        let rawBuffPointer = readBytes(from: registers.lowerBound, count: count)
        return rawBuffPointer.load(as: type)
    }

    func write(to register: UInt8, value: I2CRegisterData) {
        blockingWriteSingleI2CRegister(slaveAddress: address.addressValue, register: register, value: value.registerValue)
    }

    /// Writes the provided value to the specified `register`. If the type is more than one byte, subsequent bytes are
    /// written to subsequent register numbers. For example, if a `UInt16` is written to register `10`, the high byte
    /// is written in register `10` and the low byte is in register `11`.
    ///
    /// - Parameter register: The register to write into.
    /// - Parameter value: The value to write.
    func write<T>(to registers: ClosedRange<UInt8>, value: T) {
        let registersCount = registers.count
        let count = MemoryLayout<T>.stride
        precondition(registersCount == count)

        var value = value
        let buff = UnsafeRawBufferPointer(start: &value, count: count)
        for (i, v) in buff.enumerated() {
            let register = registers.lowerBound + UInt8(i)
            blockingWriteSingleI2CRegister(slaveAddress: address.addressValue, register: register, value: v)
        }
    }
}

/// Values which can read from a register conform to this protocol.
public protocol I2CRegisterData {
    /// Initializes from the specified `UInt8` value.
    ///
    /// - Parameter registerValue: The `UInt8` value to initialize with.
    init(registerValue: UInt8)

    /// Returns the current register value.
    var registerValue: UInt8 { get }
}

/// Values which can read and write to a register conform to this protocol.
public protocol I2CMutableRegisterData: I2CRegisterData {
    var registerValue: UInt8 { get set }
}

/// Creates an 8-bit bitmask within the specified range. The range must be between 0 and 7.
///
/// - Parameter range: The range of bits to include in the mask. (eg. `0...3`)
/// - Returns: The bitmasked `UInt8`.
private func bitmask(from range: ClosedRange<UInt8>) -> UInt8 {
    precondition(range.lowerBound < 8 && range.upperBound < 8, "Range must be between 0 and 7")

    var mask: UInt8 = 0

    for _ in range {
        mask = mask << 1 | 1
    }
    mask = mask << range.lowerBound

    return mask
}

public extension I2CRegisterData {
    /// Checks if the `registerValue` has a value of `1` at the specified `index`.
    ///
    /// - Parameter index: The bit number between `0` and `7` to check.
    /// - Returns: `true` if the bit at the specified index is set to `1`.
    func hasBit(at index: UInt8) -> Bool {
        precondition(index < 8)
        return registerValue & (1 << index) != 0
    }

    /// Gets the bit value at the specified index. The result will always be `1` or `0`.
    ///
    /// - Parameter index: The index to retrieve from, between `0` and `7`.
    /// - Returns: The `1` or `0` value for the specified bit.
    func getBit(at index: UInt8) -> UInt8 {
        precondition(index < 8)
        return (registerValue >> index) & 0b1
    }

    /// Gets the `RawRepresentable` with a `UInt8` `RawValue` that matches the bit at the specified index.
    /// The `RawRepresentable` is assumed to have valid values for `0` and `1`. If not, a `defaultValue` can be provided.
    ///
    /// This could be any `RawRepresentable`, but in practice this is typically an `enum` whose value type is `UInt8`. Eg:
    ///
    /// ```lua
    /// enum Active: UInt8 {
    ///     case inactive = 0
    ///     case active = 1
    /// }
    /// ```
    ///
    /// - Parameter index: The index to retrive from, between `0` and `7`.
    /// - Parameter defaultValue: The `RawRepresentable` value to use if none is found matching the bit value.
    /// - Returns: The `RawRepresentable` instance.
    func getBit<T: RawRepresentable>(at index: UInt8, defaultValue: T? = nil) -> T where T.RawValue == UInt8 {
        precondition(index < 8)

        // return the value, or the default, or if all else fails, force the value
        guard let value = T(rawValue: getBit(at: index)) ?? defaultValue else {
            fatalError()
        }
        return value
    }

    /// Reads the bits in the specified range and returns them as a `UInt8`, shifted to the right.
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Returns: The bits, shifted to the right.
    func getBits(from range: ClosedRange<UInt8>) -> UInt8 {
        precondition(range.lowerBound < 8)
        precondition(range.upperBound < 8)

        let mask = bitmask(from: range)
        return (registerValue & mask) >> range.lowerBound
    }

    /// Gets the `RawRepresentable` with a `UInt8` `RawValue` that matches the bits at the specified index.
    /// The `RawRepresentable` is assumed to have valid values for all possible values within those bits.
    /// If not, a `defaultValue` can be provided, which will be used if there is a gap.
    ///
    /// This could be any `RawRepresentable`, but in practice this is typically an `enum` whose value type is `UInt8`. Eg:
    ///
    /// ```lua
    /// enum Options: UInt8 {
    ///     case zero = 0
    ///     case one = 1
    ///     case two = 2
    ///     case three = 3
    /// }
    ///
    /// registerValue = 0b00000100
    /// let option: Options = getBits(from: 2...3)) // `0b01`
    /// option == .one // true
    /// ```
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Parameter defaultValue: The `RawRepresentable` value to use if none is found matching the bit value.
    /// - Returns: The `RawRepresentable` instance.
    /// - Throws: Fatal error if the range is outside `0` and `7`, or no matching value was found and no default provided.
    func getBits<T: RawRepresentable>(from range: ClosedRange<UInt8>, defaultValue: T? = nil) -> T where T.RawValue == UInt8 {
        // return the value, or the default, or if all else fails, force the value
        guard let value = T(rawValue: getBits(from: range)) ?? defaultValue else {
            fatalError()
        }
        return value
    }
}

public extension I2CMutableRegisterData {

    /// Sets the bit at the specified index to `1` if the `value` is `true`.
    ///
    /// - Parameter index: The index to set, between `0` and `7`.
    /// - Parameter value: The value.
    mutating func setBit(at index: UInt8, to value: Bool) {
        // index is between 0 and 7
        precondition(index < 8)

        setBit(at: index, to: UInt8(value ? 1 : 0))
    }

    /// Sets the bit at the specified index to the `value`.
    ///
    /// - Parameter index: The index to set, between `0` and `7`.
    /// - Parameter value: The value.
    /// - Throws a fatal error if the index is out of range, or the value is greater than `1`.
    mutating func setBit(at index: UInt8, to value: UInt8) {
        precondition(index < 8)
        precondition(value <= 1)

        registerValue = registerValue & ~(1 << index) | (value & 0b1) << index
    }

    /// Sets the bit at the specified index to the `rawValue` of the `RawRepresentable`
    ///
    /// - Parameter index: The index to set, between `0` and `7`.
    /// - Parameter value: The value.
    /// - Throws a fatal error if the index is out of range, or the `rawValue` is greater than `1`.
    mutating func setBit<T: RawRepresentable>(at index: UInt8, to value: T) where T.RawValue == UInt8 {
        setBit(at: index, to: value.rawValue)
    }

    /// Sets the bits in the specified `range` to the provided `value`. Only the lowest-ranking bits
    /// from `value` matching the size of the `range` will be set - higher bits will be ignored.
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Parameter value: The `UInt8` value to read bits from.
    mutating func setBits(from range: ClosedRange<UInt8>, to value: UInt8) {
        // all values are between 0 and 7
        precondition(range.lowerBound < 8)
        precondition(range.upperBound < 8)

        let mask = bitmask(from: range)
        let value = (value << range.lowerBound) & mask
        registerValue = registerValue & ~mask | value
    }

    /// Sets the bits in the specified `range` to the `rawValue` provided `RawRepresentable`.
    /// Only the lowest-ranking bits from `value` matching the size of the `range` will be set - higher bits will be ignored.
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Parameter value: The `RawRepresentable` value to read bits from.
    mutating func setBits<T: RawRepresentable>(from range: ClosedRange<UInt8>, to value: T) where T.RawValue == UInt8 {
        setBits(from: range, to: value.rawValue)
    }

}

// Extends `UInt8` to be `I2CMutableRegisterData`. Simply initializes and returns itself.
extension UInt8: I2CMutableRegisterData {

    /// Initializes the value to `1` or `0` for `true` or `false`, respectively.
    ///
    /// - Parameter value: The boolean value.    
    public init(_ value: Bool) {
        self = value ? 1 : 0
    }

    /// Initialises the value based on a given register value.
    public init(registerValue: UInt8) {
        self = registerValue
    }

    /// The current register value.
    public var registerValue: UInt8 {
        get { self }
        set { self = newValue }
    }
}
