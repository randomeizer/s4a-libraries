/// An extension to `Float` that allows it to be initialised from a fixed-point integer value.
extension Float {
    /// Initialises a `Float` from a fixed-point integer value.
    ///
    /// - Parameter value: The fixed-point integer value to initialise from.
    /// - Parameter fractionalBits: The number of fractional bits to use.
    public init<T,FB>(fromFixedPoint value: T, fractionalBits: FB) where T: FixedWidthInteger, FB: UnsignedInteger {
        self = Float(value) / Float(1 << fractionalBits)
    }
}
