package org.randyzhu.slidemgr;

import androidx.annotation.NonNull;

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

    public FourStageViperSlide(double mountHeightMM, double encoderTicksToFull, MotorEx driveMotor, Telemetry telemetry) {
        this.mountHeightMM = mountHeightMM;
        // the target height is the same as the mount height, at the beginning
        this.targetHeight = mountHeightMM;
        this.encoderHeight = mountHeightMM;
        this.encoderTicksToFull = encoderTicksToFull;
        this.driveMotor = driveMotor;
        this.powerCoefficient = 1;
        this.telemetry = telemetry;
        this.driveMotor.setRunMode(MotorEx.RunMode.PositionControl);
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

        driveMotor.setRunMode(MotorEx.RunMode.RawPower);
        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // if the slide is at the top or bottom, don't let it move any further
        if (driveMotor.getCurrentPosition() > encoderTicksToFull && power < 0) {
            driveMotor.set(power * powerCoefficient);
        } else if (driveMotor.getCurrentPosition() < -100 && power > 0) {
            driveMotor.set(power * powerCoefficient);
        }
        driveMotor.setRunMode(MotorEx.RunMode.PositionControl);

        encoderHeight = (driveMotor.getCurrentPosition() / encoderTicksToFull) * MAX_EXTENSION_LENGTH_MM;
        targetHeight = encoderHeight;
    }

    /**
     * Moves the slide to a specified height
     *
     * @param height The height to move the slide to, in millimeters. Relative to total slide extension.
     */
    private void setHeight(double height) {
        if (height > MAX_EXTENSION_LENGTH_MM || height < mountHeightMM) {
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
        var moveThread = new Thread(() -> {
            ElapsedTime timer = new ElapsedTime();
            // wait until the motor is at the target position or 5 second time out has passed
            telemetry.addData("Beginning to move slide: ", reportPosition());
            while (!driveMotor.atTargetPosition() || timer.seconds() < 5) {
                driveMotor.set(powerCoefficient * powerCoefficient);
            }

            if (timer.seconds() >= 5) {
                telemetry.addData("ERROR: ", "Slide motor timed out " + reportPosition());
                telemetry.update();
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

            telemetry.update();
        });

        // if the current slide thread is still running, wait for it to finish
        if (slideThread != null && slideThread.isAlive()) {
            try {
                slideThread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else {
            // otherwise, set the current slide thread to the new thread
            slideThread = moveThread;
        }

        // start the thread
        slideThread.start();
    }

    /**
     * Moves the slide all the way down, then resets the encoder to 0.
     */
    public void autoReset() {
        telemetry.addData("Auto Reset", "Starting");
        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
        goToJunction(JunctionType.GROUND);

        manualReset();
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
        setHeight(Math.min(Math.max(junction.getHeightMillimeters() - mountHeightMM, 0), MAX_EXTENSION_LENGTH_MM));
        telemetry.addData("Going to: ", junction.name().toLowerCase() + " junction");
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

    public String reportPosition() {
        return "Target Height: " + targetHeight + "mm" + "Encoder Height: " + encoderHeight + "mm";
    }
}
