package org.randyzhu.slidemgr;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourStageViperSlide {
    // motor that drives the slide
    private final MotorEx driveMotor;

    // current height of the object that's mounted on the slide
    private final double mountHeightMM;

    // maximum extension length of the slide
    private final double maxExtensionLengthMM = 1360;

    // number of encoder ticks to fully extend the slide
    private final double encoderTicksToFull;
    Telemetry telemetry;
    // current height of the mounted object in millimeters
    private double targetHeight;
    // raw extended of the slide in mm, reported by the encoder
    private double encoderHeight;


    public FourStageViperSlide(double mountHeightMM, double encoderTicksToFull, MotorEx driveMotor, Telemetry telemetry) {
        this.mountHeightMM = mountHeightMM;
        // the target height is the same as the mount height, at the beginning
        this.targetHeight = mountHeightMM;
        this.encoderHeight = mountHeightMM;
        this.encoderTicksToFull = encoderTicksToFull;
        this.driveMotor = driveMotor;
        this.telemetry = telemetry;
        this.driveMotor.setRunMode(MotorEx.RunMode.PositionControl);
    }

    public void setPower(double power) {
        if (power > 1 || power < -1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }

        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // if the slide is at the top or bottom, don't let it move any further
        if (driveMotor.getCurrentPosition() > -4400 && power < 0) {
            driveMotor.set(power);
        } else if (driveMotor.getCurrentPosition() < -100 && power > 0) {
            driveMotor.set(power);
        }

        encoderHeight = (driveMotor.getCurrentPosition() / encoderTicksToFull) * maxExtensionLengthMM;
        targetHeight = encoderHeight;
    }

    public void setHeight(double height /* height that is, away from the origin mount point. */) {
        if (height > maxExtensionLengthMM || height < mountHeightMM) {
            throw new IllegalArgumentException("Height must be between 0 and " + maxExtensionLengthMM);
        }

        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        double targetPosition = (height / maxExtensionLengthMM) * encoderTicksToFull;
        driveMotor.setTargetPosition((int) targetPosition);
        driveMotor.set(0);
        driveMotor.setPositionTolerance(100);

        new Thread(() -> {
            ElapsedTime timer = new ElapsedTime();
            // wait until the motor is at the target position or 5 second time out has passed
            while (!driveMotor.atTargetPosition() || timer.seconds() < 5) {
                driveMotor.set(1);
            }

            if (timer.seconds() >= 5) {
                telemetry.addData("Error", "Slide motor timed out, current position: " + encoderHeight, "target position: " + height);
                telemetry.update();
            }

            driveMotor.stopMotor();
        }).start();

        encoderHeight = (driveMotor.getCurrentPosition() / encoderTicksToFull) * maxExtensionLengthMM;
        targetHeight = height;

        telemetry.addData("Target Height", targetHeight);
        telemetry.addData("Encoder Height", encoderHeight);
        telemetry.addData("Delta between target and encoder", targetHeight - encoderHeight);
        telemetry.update();
    }

    // RUNS ONLY AFTER DRIVER HAS MOVED SLIDE ALL THE WAY DOWN, OR ELSE YOU WILL BUG THE SLIDE
    // DO NOT RUN THIS METHOD UNLESS YOU ARE SURE THE SLIDE IS ALL THE WAY DOWN
    public void calibrateRestingState() {
        driveMotor.resetEncoder();
        encoderHeight = mountHeightMM;
        targetHeight = mountHeightMM;
    }

    public void rest() {
        setHeight(mountHeightMM);
        driveMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void goToJunction(@NonNull JunctionType junction) {
        setHeight(Math.min(junction.getHeightMillimeters() - mountHeightMM, maxExtensionLengthMM));
    }

}
