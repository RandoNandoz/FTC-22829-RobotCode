package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.randyzhu.slidemgr.FourStageViperSlide;
import org.randyzhu.slidemgr.JunctionType;

import java.util.Locale;

@SuppressWarnings("unused")
@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // initialize motors
        var mFrontLeft = new MotorEx(hardwareMap, "mFrontLeft", Motor.GoBILDA.RPM_312);
        var mFrontRight = new MotorEx(hardwareMap, "mFrontRight", Motor.GoBILDA.RPM_312);
        var mBackLeft = new MotorEx(hardwareMap, "mBackLeft", Motor.GoBILDA.RPM_312);
        var mBackRight = new MotorEx(hardwareMap, "mBackRight", Motor.GoBILDA.RPM_312);

        // initialize imu
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        Orientation robotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        AngularVelocity robotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // initialize drivetrain
        var mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(mFrontLeft, mFrontRight, mBackLeft, mBackRight);

        // initialize gamepads
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx operatorOp = new GamepadEx(gamepad2);

        // set motors to brake when power is 0, this is to prevent the robot from moving when the driver lets go of the sticks
        // gets rid of the "drift" effect, and safer for operation
        for (MotorEx motor : new MotorEx[]{mFrontLeft, mFrontRight, mBackLeft, mBackRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        // initialize slide, according to fusion 360 our claw is 63.447mm above the ground
        FourStageViperSlide slide = new FourStageViperSlide(63.447, -4400, new MotorEx(hardwareMap, "mSlide", Motor.GoBILDA.RPM_312), 2.0 / 3.0, telemetry);


        while (opModeIsActive()) {
            var xGyro = robotOrientation.firstAngle;
            var yGyro = robotOrientation.secondAngle;
            var zGyro = robotOrientation.thirdAngle;

            var xVel = robotAngularVelocity.xRotationRate;
            var yVel = robotAngularVelocity.yRotationRate;
            var zVel = robotAngularVelocity.zRotationRate;


            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Gyro Data: ", String.format(Locale.CANADA, "X: %.2f, Y: %.2f, Z: %.2f", xGyro, yGyro, zGyro));
            telemetry.addData("Gyro Velocities: ", String.format(Locale.CANADA, "X: %.2f, Y: %.2f, Z: %.2f", xVel, yVel, zVel));
            telemetry.addData("Slide Target Height: ", slide.getTargetHeight());
            telemetry.addData("Slide Encoder Reported Height: ", slide.getEncoderHeight());

            mecanumDrive.driveRobotCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX(), true);

            if (driverOp.getButton(GamepadKeys.Button.X)) {
                slide.goToJunction(JunctionType.LOW);
            } else if (driverOp.getButton(GamepadKeys.Button.Y)) {
                slide.goToJunction(JunctionType.MEDIUM);
            } else if (driverOp.getButton(GamepadKeys.Button.B)) {
                slide.goToJunction(JunctionType.HIGH);
            } else if (driverOp.getButton(GamepadKeys.Button.A)) {
                slide.autoReset();
            }

            if (driverOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slide.setPower(driverOp.getRightY());
            }

            // calibrate resting state, done ONLY when the slide is at the ground
            if (operatorOp.getButton(GamepadKeys.Button.B)) {
                // hopefully the operator doesn't fat finger left dpad and right stick
                if (operatorOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    slide.manualReset();
                } else if (operatorOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    slide.autoReset();
                }
            }

            telemetry.update();
        }
    }
}
