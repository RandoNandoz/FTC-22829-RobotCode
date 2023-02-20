package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@TeleOp(name = "GodWheels")
public class MecanumDrive extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // initialize motors
        MotorEx mFrontLeft = new MotorEx(hardwareMap, "mFrontLeft", Motor.GoBILDA.RPM_312);
        MotorEx mFrontRight = new MotorEx(hardwareMap, "mFrontRight", Motor.GoBILDA.RPM_312);
        MotorEx mBackLeft = new MotorEx(hardwareMap, "mBackLeft", Motor.GoBILDA.RPM_312);
        MotorEx mBackRight = new MotorEx(hardwareMap, "mBackRight", Motor.GoBILDA.RPM_312);

        MotorEx mSlide = new MotorEx(hardwareMap, "mSlide", Motor.GoBILDA.RPM_312);

        for (MotorEx motor : new MotorEx[]{mFrontLeft, mFrontRight, mBackLeft, mBackRight, mSlide}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        mSlide.setRunMode(Motor.RunMode.PositionControl);


        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime);

            com.arcrobotics.ftclib.drivebase.MecanumDrive mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(mFrontLeft, mFrontRight, mBackLeft, mBackRight);

            GamepadEx driverOp = new GamepadEx(gamepad1);

            mecanumDrive.driveRobotCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX());

            // motor for linear slide
            if (driverOp.getButton(GamepadKeys.Button.A)) {
                mSlide.setTargetPosition(-4400);
                mSlide.set(0);
                mSlide.setPositionTolerance(100);

                while (!mSlide.atTargetPosition()) {
                    mSlide.set(1);
                }
                mSlide.stopMotor();
            } else if (driverOp.getButton(GamepadKeys.Button.B)) {
                mSlide.setTargetPosition(0);
                mSlide.set(0);
                mSlide.setPositionTolerance(100);

                while (!mSlide.atTargetPosition()) {
                    mSlide.set(1);
                }
                mSlide.stopMotor();
            }

            if (driverOp.getButton(GamepadKeys.Button.DPAD_UP) && mSlide.getCurrentPosition() < -100) {
                mSlide.set(0.8);
            } else if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && mSlide.getCurrentPosition() > -4400) {
                mSlide.set(-0.8);
            } else {
                mSlide.set(0);
            }


            telemetry.addData("slide position", mSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
