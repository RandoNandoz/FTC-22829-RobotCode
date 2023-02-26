package org.randyzhu.slidemgr;

import android.util.Log;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.Contract;

/**
 * This class manages a four stage GoBilda Viper slide.
 */
public class FourStageViperSlide {
    /**
     * Motor that drives the slide
     */
    private final MotorEx driveMotor;

    /**
     * Current height of the object that's mounted on the slide
     */
    private final double mountHeightMM;

    /**
     * Maximum extension length of the gobilda 4 stage slide
     */
    private final double MAX_EXTENSION_LENGTH_MM = 1360;

    /**
     * Number of encoder ticks to fully extend the slide
     */
    private final double encoderTicksToFull;
    /**
     * Power coefficient for the slide motor
     */
    private final double powerCoefficient;
    /**
     * Telemetry object to send data to the driver station
     */
    Telemetry telemetry;
    /**
     * Current height of the mounted object in millimeters
     */
    private double targetHeight;
    /**
     * Raw extended of the slide in mm, reported by the encoder
     */
    private double encoderHeight;
    /**
     * The thread that runs the slide movement code, so that it doesn't block the drive thread
     */
    private Thread slideThread;


    /**
     * @param mountHeightMM      The height of the object that's mounted on the slide
     * @param encoderTicksToFull The number of encoder ticks to fully extend the slide
     * @param driveMotor         The motor that drives the slide
     * @param telemetry          The telemetry object to send data to the driver station
     */
    public FourStageViperSlide(double mountHeightMM, double encoderTicksToFull, MotorEx driveMotor, double powerCoefficient, Telemetry telemetry) {
        this.mountHeightMM = mountHeightMM;
        // the target height is the same as the mount height, at the beginning
        this.targetHeight = mountHeightMM;
        this.encoderHeight = mountHeightMM;
        this.encoderTicksToFull = encoderTicksToFull;
        this.driveMotor = driveMotor;
        this.powerCoefficient = powerCoefficient;
        this.telemetry = telemetry;
        this.driveMotor.setRunMode(MotorEx.RunMode.PositionControl);
    }

    /**
     * Holds the slide in place, don't hold for too long as it will burn out the motor
     *
     * @param maxPower The maximum power to use to hold the slide in place
     */
    public void holdPosition(double maxPower) {
        driveMotor.setTargetPosition(driveMotor.getCurrentPosition() + 20);
        driveMotor.set(maxPower);
    }

    /**
     * Stops the slide motor.
     */
    public void stopMotor() {
        driveMotor.stopMotor();
    }

    /**
     * Sets the power of the slide motor, consequently moving the slide up or down.
     *
     * @param power The power to set the slide motor to, between -1 and 1
     */
    public void setPower(double power) {
        if (power > 1 || power < -1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }

        driveMotor.setRunMode(Motor.RunMode.RawPower);

        // if the slide is at the top or bottom, don't let it move any further
        if (driveMotor.getCurrentPosition() > encoderTicksToFull && power < 0) {
            driveMotor.set(power * powerCoefficient);
        } else if (driveMotor.getCurrentPosition() < -80 && power > 0) {
            driveMotor.set(power * powerCoefficient );
        } else {
            driveMotor.set(0);
        }

        driveMotor.setRunMode(Motor.RunMode.PositionControl);

        encoderHeight = (driveMotor.getCurrentPosition() / encoderTicksToFull) * MAX_EXTENSION_LENGTH_MM;
        targetHeight = encoderHeight;
    }

    /**
     * Moves the slide to a specified height
     *
     * @param height The height to move the slide to, in millimeters. Relative to total slide extension.
     */
    public void setHeight(double height, boolean rstPos) {
        if (height > MAX_EXTENSION_LENGTH_MM || height < 0) {
            throw new IllegalArgumentException("Height must be between 0 and " + MAX_EXTENSION_LENGTH_MM);
        }

        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // calculate the target encoder of the slide motor
        // percentage of the max extension length * number of encoder ticks to fully extend the slide
        double targetPosition = (height / MAX_EXTENSION_LENGTH_MM) * encoderTicksToFull;

        // set the target position of the slide motor
        driveMotor.setTargetPosition((int) targetPosition);

        // reset the motor power
        driveMotor.set(0);

        // set the tolerance of the motor to 60 encoder ticks
        driveMotor.setPositionTolerance(60);

        // create the thread that will move the slide to the target position
        ElapsedTime timer = new ElapsedTime();
        // wait until the motor is at the target position or 5 second time out has passed
        telemetry.addData("Beginning to move slide: ", reportPosition());
        while (!driveMotor.atTargetPosition() && timer.seconds() < 5) {
            driveMotor.set(powerCoefficient);
        }

        if (timer.seconds() >= 5) {
            telemetry.addData("ERROR: ", "Slide motor timed out " + reportPosition());
        }

        driveMotor.stopMotor();

        encoderHeight = (driveMotor.getCurrentPosition() / encoderTicksToFull) * MAX_EXTENSION_LENGTH_MM;
        targetHeight = height;

        telemetry.addData("Target Height: ", reportPosition());
        telemetry.addData("Delta between target and encoder: ", targetHeight - encoderHeight);

        if (Math.abs(encoderHeight - targetHeight) > 50) {
            if (Math.abs(encoderHeight) > targetHeight) {
                telemetry.addData("Slide motor overshot: ", reportPosition());
            } else if (Math.abs(encoderHeight) < targetHeight) {
                telemetry.addData("Slide motor undershot: ", reportPosition());
            }
        }

        if (rstPos) {
            manualReset();
        }
    }

    /**
     * Moves the slide all the way down, then resets the encoder to 0.
     */
    public void autoReset() {
        telemetry.addData("Auto Reset", "Starting");
        goToJunction(JunctionType.GROUND);
        telemetry.addData("Auto Reset", "Done");
    }

    /**
     * Resets the encoder to 0, without moving the slide, use in case of encoder drift/jank
     */
    public void manualReset() {
        telemetry.addData("MANUAL RESET: ", "THE SLIDE MUST NOT BE EXTENDED");
        driveMotor.resetEncoder();
        encoderHeight = mountHeightMM;
        targetHeight = mountHeightMM;
    }

    /**
     * Moves the slide to a specified junction
     *
     * @param junction The junction to move the slide to, see {@link JunctionType}
     */
    public void goToJunction(@NonNull JunctionType junction) {
        Log.d("USERCODE: ", String.valueOf(junction.getHeightMillimeters() - mountHeightMM));
        telemetry.addData("Going to junction: ", junction.name());
        if (junction == JunctionType.GROUND) {
            setHeight(0, true);
        } else {
            setHeight(Math.min(junction.getHeightMillimeters() - mountHeightMM + 650, MAX_EXTENSION_LENGTH_MM), false);
        }
    }

    /**
     * @return The current height of the slide in millimeters
     */
    @Contract(pure = true)
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * @return The "actual" current height of the slide in millimeters, reported by the encoder
     */
    @Contract(pure = true)
    public double getEncoderHeight() {
        return encoderHeight;
    }

    /**
     * @return The number of encoder ticks the slide motor has moved by
     */
    @Contract(pure = true)
    public double getMotorEncoderTicks() {
        return driveMotor.getCurrentPosition();
    }

    /**
     * @return The power of the slide motor
     */
    @Contract(pure = true)
    public double getMotorPower() {
        return driveMotor.motorEx.getPower();
    }

    /**
     * @return The angular velocity of the slide motor
     */
    @Contract(pure = true)
    public double getMotorVelocity() {
        return driveMotor.getCorrectedVelocity();
    }

    /**
     * @return The target and encoder reported position the slide is at
     */
    @NonNull
    @Contract(pure = true)
    private String reportPosition() {
        return "Target Height: " + targetHeight + "mm" + "Encoder Height: " + encoderHeight + "mm";
    }
}
