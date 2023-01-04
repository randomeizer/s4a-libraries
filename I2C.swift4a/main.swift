//------------------------------------------------------------------------------
//
// This project contains I2C extensions and implementations.
// Swift For Arduino
//
// Created by David Peterson on 30/1/22.
// Copyright © 2022 David Peterson. All rights reserved.
//
//------------------------------------------------------------------------------

import AVR

// Note: This file just exists so that Swift 4 Arduino will load the project.
// If you duplicate the project, remove tests and add your application code here.

while(true) {}

func testMPL3115A2() {
    SetupSerial()
    print("Testing MPL3115A2")

    var dev = MPL3115A2()

    // Just run it once
    dev.altitudeViaPolling { altitude in
        print("Altitude: ")
        print(altitude)
        print(" m")
        return .done
    }

    // Get temperature 5 times
    var count = 5
    dev.temperatureViaPolling { temperature in
        print("Temperature: ")
        print(temperature)
        print("ºC")
        count = count-1
        return count == 0 ? .done : .continue
    }
}
