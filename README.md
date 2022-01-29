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

The [I2CExtensions](Libraries/I2CExtensions.swift) library makes it (relatively) easy to define an `I2C` device interface.
