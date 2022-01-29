import AVR

/// Used to measure clock cycles.
typealias Ticks = UInt32

/// Provides support for sending messages to the serial port.
/// It can be enabled and disabled to control whether the messages are sent.
/// It can have a `period` set, which determines the minimum time between messages that
/// are actually sent.
struct Logger {
    /// Indicates if Logging in general is enabled.
    static var enabled = true

    /// Indicates if the specific `Logger` is active.
    let active: Bool

    /// Indicates the number of Ticks between outputs.
    /// This roughly equates to milliseconds on a 16Hz processor.
    /// Set to `0` for every value to get logged, or larger numbers to log
    /// periodically.
    let period: Ticks

    private var prevTicks: Ticks = 0

    private var ready: Bool = true

    private var plotted = false

    init(active: Bool, period: Ticks) {
        self.active = active
        self.period = period
    }

    mutating func check() {
        guard Logger.enabled && self.active else {
            ready = false
            return
        }

        let now = ticks()
        ready = (period == 0 || now - prevTicks > period)

    }

    /// Prints the provided `StaticString`, if the `Logger` is active and ready to print.
    /// - Parameter message: The `StaticString` to print. Does *not* output a newline.
    func print(_ message: @autoclosure () -> StaticString) {
        guard ready else { return }
        AVR.print(staticString: message(), addNewline: false)
    }

    func print(_ message: @autoclosure () -> Float) {
        guard ready else { return }
        AVR.print(message(), addNewline: false)
    }

    /// Prints the provided `UInt8` value, if the `Logger` is active and ready to print.
    /// - Parameter message: A closure returning the `UInt8` to print.
    func print(_ message: @autoclosure () -> UInt8) {
        guard ready else { return }
        AVR.print(message(), addNewline: false)
    }

    mutating func println(_ message: StaticString = "") {
        guard ready else { return }
        AVR.print(message)
    }

    private func plot(_ label: StaticString, value: () -> Void) {
        guard ready else { return }
        self.print(label)
        self.print(": ")
        value()
        print("\t")
    }

    func plot(_ label: StaticString, _ value: StaticString) {
        self.plot(label) { print(value) }
    }

    func plot(_ label: StaticString, _ value: Float) {
        self.plot(label) { print(value) }
    }

    func plot(_ label: StaticString, _ value: UInt8) {
        self.plot(label) { print(value) }
    }

    func endPlot() {
        self.println()
    }
}

/* Snippets:
 {
        "Logger":[

            {"partName":"Setup Logger",
                "partCode":"// setup Logger which will output every message\nlet firehose = Logger(active: true, period: 0)"
            },

            {"partName":"Setup Rated Logger",
                "partCode":"// setup Logger which will output every 1 second\nlet trickle = Logger(active: true, period: 1000)"
            },

            {"partName":"Print message",
                "partCode":"firehose.println(\"Hello World\")"
            },

            {"partName":"Print message without a newline",
                "partCode":"firehose.print(\"Hello World\")"
            },

            {"partName":"Plot values",
                "partCode":"trickle.plot(\"Speed\", speed)\ntrickle.plot(\"Direction\", direction)\ntrickle.endPlot()"
            },
        ]
 }
*/
