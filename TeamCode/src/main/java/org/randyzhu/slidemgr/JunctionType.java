package org.randyzhu.slidemgr;

public enum JunctionType {
    LOW(13.5),
    MEDIUM(23.5),
    HIGH(33.5);

    final double heightInches;
    final double heightMillimeters;

    JunctionType(double heightInches) {
        this.heightInches = heightInches;
        this.heightMillimeters = heightInches * 25.4;
    }

    double getHeightInches() {
        return this.heightInches;
    }

    double getHeightMillimeters() {
        return this.heightMillimeters;
    }
}
