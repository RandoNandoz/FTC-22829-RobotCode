package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.randyzhu.claw.LoonyClaw;
import org.randyzhu.slidemgr.FourStageViperSlide;
import org.randyzhu.slidemgr.JunctionType;

import java.util.Locale;

@SuppressWarnings("unused")
@TeleOp(name = "MecanumDriveJovan")
@Disabled
public class MecanumDriveJovan extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();

        // initialize motors
        var mFrontLeft = new MotorEx(hardwareMap, "mFrontLeft", Motor.GoBILDA.RPM_312);
        var mFrontRight = new MotorEx(hardwareMap, "mFrontRight", Motor.GoBILDA.RPM_312);
        var mBackLeft = new MotorEx(hardwareMap, "mBackLeft", Motor.GoBILDA.RPM_312);
        var mBackRight = new MotorEx(hardwareMap, "mBackRight", Motor.GoBILDA.RPM_312);

        double powerCoefficient = 0.85;

        var sClaw = new SimpleServo(hardwareMap, "sClaw", 0, 300, AngleUnit.DEGREES);

        // initialize imu
        var imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        Orientation robotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        AngularVelocity robotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // initialize drivetrain
        var mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(mFrontLeft, mFrontRight, mBackLeft, mBackRight);

        // initialize gamepads
        var driverOp = new GamepadEx(gamepad1);
        var operatorOp = new GamepadEx(gamepad2);

        // set motors to brake when power is 0, this is to prevent the robot from moving when the driver lets go of the sticks
        // gets rid of the "drift" effect, and safer for operation
        for (MotorEx motor : new MotorEx[]{mFrontLeft, mFrontRight, mBackLeft, mBackRight}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        // initialize slide, according to fusion 360 our claw is 63.447mm above the ground
        var slide = new FourStageViperSlide(63.447, -4000, new MotorEx(hardwareMap, "mSlide", Motor.GoBILDA.RPM_312), 2.0 / 3.0, telemetry);

        var claw = new LoonyClaw(sClaw, 0, 28);

        // reset the position of our slide so it thinks its the right pos.
        slide.manualReset();

        waitForStart();

        claw.open();

        while (opModeIsActive()) {
            // imu data
            var xGyro = robotOrientation.firstAngle;
            var yGyro = robotOrientation.secondAngle;
            var zGyro = robotOrientation.thirdAngle;
            var xVel = robotAngularVelocity.xRotationRate;
            var yVel = robotAngularVelocity.yRotationRate;
            var zVel = robotAngularVelocity.zRotationRate;

            // report runtime
            telemetry.addData("Status", "Run Time: " + runtime);

            // report imu data
            telemetry.addData("Gyro Data: ", String.format(Locale.CANADA, "X: %.2f deg, Y: %.2f deg, Z: %.2f deg", xGyro, yGyro, zGyro));
            telemetry.addData("Gyro Velocities: ", String.format(Locale.CANADA, "X: %.2f deg/s, Y: %.2f deg/s, Z: %.2f deg/s", xVel, yVel, zVel));

            // report slide data
            telemetry.addData("Slide Power: ", String.format(Locale.CANADA, "%.2f%%", slide.getMotorPower() * 100));
            telemetry.addData("Slide Motor Angular Velocity: ", String.format(Locale.CANADA, "%.2f ticks/s", slide.getMotorVelocity()));
            telemetry.addData("Slide Encoder Ticks: ", String.format(Locale.CANADA, "%f ticks", slide.getMotorEncoderTicks()));
            telemetry.addData("Slide Target Height: ", String.format("%s mm", slide.getTargetHeight()));
            telemetry.addData("Slide Encoder Reported Height: ", String.format("%s mm", slide.getEncoderHeight()));

            // report motor powers
            telemetry.addData("Front Left Motor Power: ", String.format(Locale.CANADA, "%.2f%%", mFrontLeft.motorEx.getPower() * 100));
            telemetry.addData("Front Right Motor Power: ", String.format(Locale.CANADA, "%.2f%%", mFrontRight.motorEx.getPower()));
            telemetry.addData("Back Left Motor Power: ", String.format(Locale.CANADA, "%.2f%%", mBackLeft.motorEx.getPower()));
            telemetry.addData("Back Right Motor Power: ", String.format(Locale.CANADA, "%.2f%%", mBackRight.motorEx.getPower()));

            // report motor angular velocities
//            telemetry.addData("Front Left Motor Angular Velocity: ", String.format(Locale.CANADA, "%.2f ticks/s", mFrontLeft.getCorrectedVelocity()));
//            telemetry.addData("Front Right Motor Angular Velocity: ", String.format(Locale.CANADA, "%.2f ticks/s", mFrontRight.getCorrectedVelocity()));
//            telemetry.addData("Back Left Motor Angular Velocity: ", String.format(Locale.CANADA, "%.2f ticks/s", mBackLeft.getCorrectedVelocity()));
//            telemetry.addData("Back Right Motor Angular Velocity: ", String.format(Locale.CANADA, "%.2f ticks/s", mBackRight.getCorrectedVelocity()));

            // report motor encoder ticks
//            telemetry.addData("Front Left Motor Encoder Ticks: ", String.format(Locale.CANADA, "%d ticks", mFrontLeft.getCurrentPosition()));
//            telemetry.addData("Front Right Motor Encoder Ticks: ", String.format(Locale.CANADA, "%d ticks", mFrontRight.getCurrentPosition()));
//            telemetry.addData("Back Left Motor Encoder Ticks: ", String.format(Locale.CANADA, "%d ticks", mBackLeft.getCurrentPosition()));
//            telemetry.addData("Back Right Motor Encoder Ticks: ", String.format(Locale.CANADA, "%d ticks", mBackRight.getCurrentPosition()));

            // report servo positions
            telemetry.addData("Servo Position: ", String.format(Locale.CANADA, "%.2f degrees", sClaw.getAngle(AngleUnit.DEGREES)));

            if (driverOp.getButton(GamepadKeys.Button.A)) {
                powerCoefficient = 1;
            } else {
                powerCoefficient = 0.85;
            }

            mecanumDrive.driveRobotCentric(-driverOp.getLeftX() * powerCoefficient, -driverOp.getLeftY() * powerCoefficient, -driverOp.getRightX() * powerCoefficient, false);

            slide.setPower(-1 * (operatorOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - operatorOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

            if (operatorOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                slide.goToJunction(JunctionType.HIGH);
            } else if (operatorOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                slide.goToJunction(JunctionType.MEDIUM);
            } else if (operatorOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                slide.goToJunction(JunctionType.LOW);
            } else if (operatorOp.getButton(GamepadKeys.Button.A)) {
                slide.autoReset();
            }

            if (operatorOp.getButton(GamepadKeys.Button.Y)) {
                claw.close();
            } else if (operatorOp.getButton(GamepadKeys.Button.B)) {
                claw.open();
            }

            if (operatorOp.getButton(GamepadKeys.Button.X)) {
                claw.open();
                slide.autoReset();
            }

            telemetry.update();
        }
    }
}
