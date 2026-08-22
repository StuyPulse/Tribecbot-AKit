/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj.Timer;

public class DualDebouncer {
    private final double riseTime;
    private final double fallTime;
    private final Timer timer = new Timer();
    private boolean baseline = false;

    public DualDebouncer(double riseSeconds, double fallSeconds) {
        riseTime = riseSeconds;
        fallTime = fallSeconds;
        timer.start();
    }

    public DualDebouncer(Time riseTime, Time fallTime) {
        this(riseTime.in(Seconds), fallTime.in(Seconds));
    }

    public boolean calculate(boolean input) {
        if (input == baseline) {
            // no change from current output, reset the timer
            timer.reset();
        } else {
            double requiredTime = input ? riseTime : fallTime;
            if (timer.hasElapsed(requiredTime)) {
                baseline = input;
                timer.reset();
            }
        }
        return baseline;
    }
}
