package org.randyzhu.slidemgr;

import org.jetbrains.annotations.Contract;

/**
 * In the FIRST Tech Challenge POWERPLAY game, there are three different heights of the junctions.
 * This enum represents the three different heights.
 * - LOW: 13.5 inches
 * - MEDIUM: 23.5 inches
 * - HIGH: 33.5 inches
 * - GROUND: (effectively) 0 inches
 */
public enum JunctionType {
    GROUND(0), LOW(13.5), MEDIUM(23.5), HIGH(33.5);

    final double heightInches;
    final double heightMillimeters;

    /**
     * @param heightInches The height of the junction in inches
     */
    JunctionType(double heightInches) {
        this.heightInches = heightInches;
        this.heightMillimeters = heightInches * 25.4;
    }

    /**
     * @return The height of the junction in inches
     */
    @Contract(pure = true)
    double getHeightInches() {
        return this.heightInches;
    }

    /**
     * @return The height of the junction in millimeters
     */
    @Contract(pure = true)
    double getHeightMillimeters() {
        return this.heightMillimeters;
    }
}
