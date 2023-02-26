package org.randyzhu.claw;

import com.arcrobotics.ftclib.hardware.ServoEx;

public class LoonyClaw {
    // tune later
    private final double closedDegrees;
    private final double openDegrees;

    private final ServoEx clawServo;

    public LoonyClaw(ServoEx clawServo, double openDegrees, double closedDegrees) {
        this.openDegrees = openDegrees;
        this.closedDegrees = closedDegrees;
        this.clawServo = clawServo;
    }

    public void open() {
        clawServo.turnToAngle(openDegrees);
    }

    public void close() {
        clawServo.turnToAngle(closedDegrees);
    }
}
