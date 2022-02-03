// Author: David Peterson
// Date: 29/11/2020
// IDE Version: 4.5
// Description: This library provides extensions to basic I2C functions to allow for `struct` values to be created
//              that communicate with specific, well-defined I2C-complient devices.

import AVR

public enum I2C {}

extension I2C {
    /// Creates an 8-bit bitmask within the specified range. The range must be between 0 and 7.
    ///
    /// - Parameter range: The range of bits to include in the mask. (eg. `0...3`)
    /// - Returns: The bitmasked `UInt8`.
    @inlinable
    public static func bitmask(from range: ClosedRange<UInt8>) -> UInt8 {
        precondition(range.lowerBound < 8 && range.upperBound < 8)

        var mask: UInt8 = 0

        for _ in range {
            mask = mask << 1 | 1
        }
        mask = mask << range.lowerBound

        return mask
    }

    /// Byte order when reading/writing multiple registers into a ``FixedWidthInteger``.
    public enum ByteOrder {
        case bigEndian
        case littleEndian
    }
}

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
/// It has an ``I2CSlaveAddress`` and provides support methods for reading and writing to the device at that address.
public protocol I2CSlaveNode {
    associatedtype Address: I2CSlaveAddress

    /// The slave address for the device.
    var address: Address { get }
}

// Adds support methods for read/write to the slave node.
public extension I2CSlaveNode {

    /// Attempts to reads sequential bytes from the register range, passing them to the closure as an ``UnsafeRawBufferPointer``.
    /// Not all I2C registers are necessarily contiguous, but if it's supported by the device, the block of bytes is
    /// copied into the AVR buffer and pointed at by the resulting ``UnsafeRawBufferPointer``. The buffer is only valid
    /// for the duration of the closure's execution.
    ///
    /// - Parameter registers: The range of registers to read from.
    /// - Parameter handler: A closure receiving an `UnsafeRawBufferPointer` for the retrieved block of memory, or `nil` if unable to allocate it.
    @inlinable
    func readBytes<T>(from registers: ClosedRange<UInt8>, into handler: (UnsafeRawBufferPointer) throws -> T) rethrows -> T {
        let registersCount = registers.count

        // NOTE: I would prefer this was an `UnsafeMutableBufferPointer` with no associated type.
        guard let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: registersCount) else {
            // out of memory
            fatalError()
        }

        buffer.initialize(repeating: 0, count: registersCount)
        defer {
            buffer.deinitialize(count: registersCount)
            buffer.deallocate()
        }

        let pointer = blockingReadMultipleI2CRegisters(
            slaveAddress: address.addressValue,
            registerStart: registers.lowerBound,
            registerCount: UInt8(registersCount),
            buffer: buffer
        )

        return try handler(UnsafeRawBufferPointer(pointer))
    }

    /// Reads a single `UInt8` value from the specified register.
    ///
    /// - Parameter register: The register number to read from.
    /// - Parameter as: The `Type` being read. Will attempt to infer this automatically, but in some contexts it must be provided explicitly.
    /// - Returns: The value as a `UInt8` byte.
    @inlinable
    func read<T>(from register: UInt8, as: T.Type = T.self) -> T where T: I2CRegisterValue {
        let value = blockingReadSingleI2CRegister(slaveAddress: address.addressValue, register: register)
        return T(registerValue: value)
    }

    /// Reads the number of registers required to fill out the `T` type value, where `T` is a ``FixedWidthInteger`` (numerics like `Int`, `UInt32`, etc).
    ///
    /// - Parameter registers: The range of register numbers to read from.
    /// - Parameter byteOrder: The byte order to use when reading the registers. Defaults to ``I2C/ByteOrder/bigEndian``, which seems to be the most
    ///   common ordering for I2C devices.
    /// - Parameter as: The `Type` being read. Will attempt to infer this automatically, but in some contexts it must be provided explicitly.
    ///
    /// - Returns: An instance of the type `T`, initialized from the required number of bytes for the type.
    /// - Throws: Precondition failure if the range of registers does not match the size of the type being returned.
    @inlinable
    func read<T>(from registers: ClosedRange<UInt8>, withByteOrder byteOrder: I2C.ByteOrder = .bigEndian, as: T.Type = T.self) -> T where T: FixedWidthInteger {
        let registersCount = UInt8(registers.count)
        let typeWidth = UInt8(MemoryLayout<T>.stride)
        precondition(registersCount <= typeWidth)

        var result = readBytes(from: registers) { buffer in
            buffer.load(as: T.self)
        }

        switch byteOrder {
        case .bigEndian:
            result = T(bigEndian: result)
            result = result >> (typeWidth - registersCount) * 8
        case .littleEndian:
            result = T(littleEndian: result)
        }

        return result
    }

    /// Reads the number of registers required to fill out the `T` type value, where `T` is an implementation of `I2CRegisterBlock`.
    ///
    /// - Parameter registers: The range of register numbers to read from.
    /// - Parameter as: The `Type` being read. Will attempt to infer this automatically, but in some contexts it must be provided explicitly.
    ///
    /// - Returns: An instance of the type `T`, initialized from the required number of bytes for the type.
    /// - Throws: Precondition failure if the range of registers does not match the size of the type being returned.
    @inlinable
    func read<T>(from registers: ClosedRange<UInt8>, as: T.Type = T.self) -> T where T: I2CRegisterBlock {
        let registersCount = UInt8(registers.count)
        let typeWidth = UInt8(MemoryLayout<T>.stride)
        precondition(registersCount <= typeWidth)

        return readBytes(from: registers) { buffer in
            buffer.load(as: T.self)
        }
    }

    /// Writes sequential bytes from the `UnsafeRawBufferPointer` to the series of registers starting with `start`.
    /// The total number of registers written to will be the `count` of the `UnsafeRawBufferPointer`.
    ///
    /// - Parameter registers: The range of registers to write to.
    /// - Parameter buffer: The buffer containing the bytes to write. Note: the caller is responsible for deallocation.
    /// - Parameter start: The index within the `buffer` to start reading from.
    @inlinable
    func writeBytes(to registers: ClosedRange<UInt8>, from buffer: UnsafeRawBufferPointer, startAt start: Int = 0) {
        let bufferCount = buffer.count
        precondition(Int(registers.count) <= bufferCount - start)
        var index = start
        for register in registers {
            blockingWriteSingleI2CRegister(slaveAddress: address.addressValue, register: register, value: buffer[index])
            index = index + 1
        }
    }

    /// Writes the provided `value` to the specified `register`.
    ///
    /// - Parameter register: The register to write into.
    /// - Parameter value: The
    @inlinable
    func write(to register: UInt8, value: I2CRegisterValue) {
        blockingWriteSingleI2CRegister(slaveAddress: address.addressValue, register: register, value: value.registerValue)
    }

    /// Writes the provided value to the specified `register`. If the type is more than one byte, subsequent bytes are
    /// written to subsequent register numbers. For example, if a `UInt16` is written to register `10`, the high byte
    /// is written in register `10` and the low byte is in register `11`.
    ///
    /// - Parameter register: The register to write into.
    /// - Parameter byteOrder: The byte order to use when writing the value. Defaults to ``ByteOrder/bigEndian``, which seems to be the
    ///   most common order for I2C devices.
    /// - Parameter value: The value to write.
    ///
    /// - Note: This will not work for types that are not contiguous in memory, such as `Array`s and other `class`es.
    /// - Note: If the register count is less than the width of the type (eg, writing 3 registers from an Int32), the lowest three bytes will be written.
    @inlinable
    func write<T>(to registers: ClosedRange<UInt8>, withByteOrder byteOrder: I2C.ByteOrder = .bigEndian, value: T) where T: FixedWidthInteger {
        let registersCount = registers.count
        let typeWidth = MemoryLayout<T>.stride
        precondition(registersCount <= typeWidth)

        var value = value
        switch byteOrder {
        case .bigEndian:
            value = value.bigEndian
            value = value >> (typeWidth - registersCount) * 8
        case .littleEndian:
            value = value.littleEndian
        }

        withUnsafeBytes(of: &value) { buffer in
            writeBytes(to: registers, from: buffer)
        }
    }

    /// Writes the provided value to the specified `register`. If the type is more than one byte, subsequent bytes are
    /// written to subsequent register numbers.
    ///
    /// - Parameter register: The register to write into.
    /// - Parameter value: The `I2CRegisterBlock` value to write.
    ///
    /// - Note: This will not work for types that are not contiguous in memory, such as `Array`s and other `class`es.
    /// - Note: If the register count is less than the width of the type (eg, writing 3 registers from an Int32), the lowest three bytes will be written.
    @inlinable
    func write<T>(to registers: ClosedRange<UInt8>, value: T) where T: I2CMutableRegisterBlock {
        let registersCount = registers.count
        let typeWidth = MemoryLayout<T>.stride
        precondition(registersCount <= typeWidth)

        var value = value
        withUnsafeBytes(of: &value) { buffer in
            writeBytes(to: registers, from: buffer)
        }
    }

}

/// Values which can read from a register conform to this protocol.
public protocol I2CRegisterValue {
    /// Initializes from the specified `UInt8` value.
    ///
    /// - Parameter registerValue: The `UInt8` value to initialize with.
    init(registerValue: UInt8)

    /// Returns the current register value.
    var registerValue: UInt8 { get }
}

/// Values which can read and write to a register conform to this protocol.
public protocol I2CMutableRegisterValue: I2CRegisterValue {
    var registerValue: UInt8 { get set }
}

/// A marker protocol for values which are safe to be read into
/// from multiple registers.
///
/// Implementors of this protocol will generally be structs that
/// contain the backing store for bytes read from registers.
///
/// They will also generally provide accessors for the particular data
/// represented in a given set of registers. Sometimes it will be
/// a multi-byte value, sometimes it will be flags or bits from a single byte.
///
/// Registers are populated directly into memory via `UnsafeRawBufferPointer`,
/// in the order that they occur in the register. This means that, for example,
/// if the register `0x01` is the high bit and `0x02` is the low bit of a 16-bit
/// integer, then they will be added into the value in that order, even though
/// most Swift platforms use little-endian formatting.
///
/// It will also do the same for signed values. For example, registers `0x01` to
/// `0x03` contains a signed, big-endian 24-bit number. There are no native 24-bit integers,
/// so we will read it into an `Int32`, which results in the first 3 bytes of the
/// `I2CRegisterBlock` being the high, mid, and low bytes, when it should be the reverse.
///
/// The ``I2CSlaveNode/read(from:withByteOrder:as)`` method will determine the
/// target ``FixedWidthInteger`` size, put convert the register order, and correctly
/// fill out the extra byte with either `0` or `1` values (depending on the sign value).
///
/// For this reason, it's recommended that structs containing multi-byte integer
/// values are initialised by reading the individual properties, rather than
/// as an `I2CRegisterBlock`.
public protocol I2CRegisterBlock {}

/// A marker protocol that indicates that the type can be read from or written
/// to as a block of register bytes.
///
/// Implementors of this protocol will generally be structs that
/// contain the backing store for bytes read from registers.
///
/// They will also generally provide accessors for the particular data
/// represented in a given set of registers. Sometimes it will be
/// a multi-byte value, sometimes it will be flags or bits from a single byte.
///
/// Registers are populated directly into memory via `UnsafeMutableRawBufferPointer`,
/// in the order that they occur in the register. This means that, for example,
/// if the register `0x01` is the high bit and `0x02` is the low bit of a 16-bit
/// integer, then they will be added into the value in that order, even though
/// most Swift platforms use little-endian formatting.
///
/// The ``I2CSlaveNode/write(to:withByteOrder:value)`` method will determine the
/// target ``FixedWidthInteger`` size, put convert the register order, and correctly
/// fill out the extra byte with either `0` or `1` values (depending on the sign value).
///
/// For this reason, it's recommended that structs containing multi-byte integer
/// values are initialised by reading the individual properties, rather than
/// as an `I2CRegisterBlock`.
public protocol I2CMutableRegisterBlock: I2CRegisterBlock {}

/// Values which read and write multiple sequential registers.
public extension I2CRegisterValue {
    /// Checks if the `registerValue` has a value of `1` at the specified `index`.
    ///
    /// - Parameter index: The bit number between `0` and `7` to check.
    /// - Returns: `true` if the bit at the specified index is set to `1`.
    @inlinable
    func hasBit(at index: UInt8) -> Bool {
        precondition(index < 8)
        return registerValue & (1 << index) != 0
    }

    /// Gets the bit value at the specified index. The result will always be `1` or `0`.
    ///
    /// - Parameter index: The index to retrieve from, between `0` and `7`.
    /// - Returns: The `1` or `0` value for the specified bit.
    @inlinable
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
    @inlinable
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
    @inlinable
    func getBits(from range: ClosedRange<UInt8>) -> UInt8 {
        precondition(range.lowerBound < 8)
        precondition(range.upperBound < 8)

        let mask = I2C.bitmask(from: range)
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
    @inlinable
    func getBits<T: RawRepresentable>(from range: ClosedRange<UInt8>, defaultValue: T? = nil) -> T where T.RawValue == UInt8 {
        // return the value, or the default, or if all else fails, force the value
        guard let value = T(rawValue: getBits(from: range)) ?? defaultValue else {
            fatalError()
        }
        return value
    }
}

public extension I2CMutableRegisterValue {

    /// Sets the bit at the specified index to `1` if the `value` is `true`.
    ///
    /// - Parameter index: The index to set, between `0` and `7`.
    /// - Parameter value: The value.
    @inlinable
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
    @inlinable
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
    @inlinable
    mutating func setBit<T: RawRepresentable>(at index: UInt8, to value: T) where T.RawValue == UInt8 {
        setBit(at: index, to: value.rawValue)
    }

    /// Sets the bits in the specified `range` to the provided `value`. Only the lowest-ranking bits
    /// from `value` matching the size of the `range` will be set - higher bits will be ignored.
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Parameter value: The `UInt8` value to read bits from.
    @inlinable
    mutating func setBits(from range: ClosedRange<UInt8>, to value: UInt8) {
        // all values are between 0 and 7
        precondition(range.lowerBound < 8)
        precondition(range.upperBound < 8)

        let mask = I2C.bitmask(from: range)
        let value = (value << range.lowerBound) & mask
        registerValue = registerValue & ~mask | value
    }

    /// Sets the bits in the specified `range` to the `rawValue` provided `RawRepresentable`.
    /// Only the lowest-ranking bits from `value` matching the size of the `range` will be set - higher bits will be ignored.
    ///
    /// - Parameter range: The `a...b` range (inclusive) to read bits from.
    /// - Parameter value: The `RawRepresentable` value to read bits from.
    @inlinable
    mutating func setBits<T: RawRepresentable>(from range: ClosedRange<UInt8>, to value: T) where T.RawValue == UInt8 {
        setBits(from: range, to: value.rawValue)
    }

}

// Extends `UInt8` to be `I2CMutableRegisterValue`. Simply initializes and returns itself.
extension UInt8: I2CMutableRegisterValue {

    /// Initializes the value to `1` or `0` for `true` or `false`, respectively.
    ///
    /// - Parameter value: The boolean value.
    @inlinable
    public init(_ value: Bool) {
        self = value ? 1 : 0
    }

    /// Initialises the value based on a given register value.
    ///
    /// - Parameter registerValue: The register value.
    @inlinable
    public init(registerValue: UInt8) {
        self = registerValue
    }

    /// The current register value.
    public var registerValue: UInt8 {
        get { self }
        set { self = newValue }
    }
}

extension Int8: I2CMutableRegisterValue {

    /// Initialises the value based on a given register value.
    ///
    /// - Parameter registerValue: The register value.
    @inlinable
    public init(registerValue: UInt8) {
        self = Int8(bitPattern: registerValue)
    }

    /// The current register value.
    public var registerValue: UInt8 {
        get { UInt8(bitPattern: self) }
        set { self = Int8(bitPattern: newValue) }
    }
}
