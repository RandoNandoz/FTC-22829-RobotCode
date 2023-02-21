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
        var mFrontLeft = new MotorEx(hardwareMap, "mFrontLeft", Motor.GoBILDA.RPM_312);
        var mFrontRight = new MotorEx(hardwareMap, "mFrontRight", Motor.GoBILDA.RPM_312);
        var mBackLeft = new MotorEx(hardwareMap, "mBackLeft", Motor.GoBILDA.RPM_312);
        var mBackRight = new MotorEx(hardwareMap, "mBackRight", Motor.GoBILDA.RPM_312);

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
        FourStageViperSlide slide = new FourStageViperSlide(
                63.447,
                -4400,
                new MotorEx(hardwareMap, "mSlide", Motor.GoBILDA.RPM_312),
                telemetry
        );


        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Slide Target Height: ", slide.getTargetHeight());
            telemetry.addData("Slide Encoder Reported Height: ", slide.getEncoderHeight());

            mecanumDrive.driveRobotCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX());

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
