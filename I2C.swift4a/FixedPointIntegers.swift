/// An extension to `BinaryFloatingPoint` that allows it to be initialised from a fixed-point integer value.
extension BinaryFloatingPoint where Self.RawSignificand: FixedWidthInteger {
    /// Initialises a `BinaryFloatingPoint` from a fixed-point integer value.
    ///
    /// - Parameter value: The fixed-point integer value to initialise from.
    /// - Parameter fractionalBits: The number of fractional bits to use.
    public init<T>(fromFixedPoint value: T, fractionalBits: UInt8) where T: FixedWidthInteger {
        self = Self(value) / Self(1 << fractionalBits)
    }
}

extension FixedWidthInteger {
    /// Initialises a `FixedWidthInteger` as a fixed-point value with the specified fractional bits.
    ///
    /// - Parameter value: The floating point value to initialise from.
    /// - Parameter fractionalBits: The number of fractional bits in the final digit.
    public init<T>(toFixedPoint value: T, fractionalBits: UInt8) where T: BinaryFloatingPoint, T.RawSignificand: FixedWidthInteger {
        self = Self(value * T(1 << fractionalBits))
    }
}
