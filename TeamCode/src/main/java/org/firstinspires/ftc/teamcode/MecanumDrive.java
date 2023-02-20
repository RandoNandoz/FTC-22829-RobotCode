package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.randyzhu.slidemgr.FourStageViperSlide;
import org.randyzhu.slidemgr.JunctionType;

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


        for (MotorEx motor : new MotorEx[]{mFrontLeft, mFrontRight, mBackLeft, mBackRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }

        // initialize slide, according to fusion 360 our claw is 63.447mm above the ground
        FourStageViperSlide slide = new FourStageViperSlide(63.447, -4400, mSlide, telemetry);


        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime);

            com.arcrobotics.ftclib.drivebase.MecanumDrive mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(mFrontLeft, mFrontRight, mBackLeft, mBackRight);

            GamepadEx driverOp = new GamepadEx(gamepad1);

            GamepadEx operatorOp = new GamepadEx(gamepad2);

            mecanumDrive.driveRobotCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX());

            if (driverOp.getButton(GamepadKeys.Button.X)) {
                slide.goToJunction(JunctionType.LOW);
            } else if (driverOp.getButton(GamepadKeys.Button.Y)) {
                slide.goToJunction(JunctionType.MEDIUM);
            } else if (driverOp.getButton(GamepadKeys.Button.B)) {
                slide.goToJunction(JunctionType.HIGH);
            } else if (driverOp.getButton(GamepadKeys.Button.A)) {
                slide.rest();
            }

            slide.setPower(operatorOp.getLeftY());

            // calibrate resting state, done ONLY when the slide is at the ground
            if (driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && driverOp.getButton(GamepadKeys.Button.B)) {
                slide.calibrateRestingState();
            }

            telemetry.addData("slide position", mSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
