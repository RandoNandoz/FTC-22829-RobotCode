package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
@Disabled
@TeleOp(name = "CalBotTwo")
public class KiwiDrive extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Motor driveMotorI = new Motor(hardwareMap, "driveMotorI");
        Motor driveMotorII = new Motor(hardwareMap, "driveMotorII");
        Motor driveMotorIII = new Motor(hardwareMap, "driveMotorIII");

        driveMotorI.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        driveMotorII.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        driveMotorIII.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
            double THIRTY_DEG_RAD = 0.523599;
            double ONE_HUNDRED_FIFTY_DEG_RAD = 2.61799;
            double TWO_HUNDRED_SEVENTY_DEG_RAD = 4.71239;
            HDrive kiwiDrive = new HDrive(driveMotorII, driveMotorIII, driveMotorI, THIRTY_DEG_RAD, ONE_HUNDRED_FIFTY_DEG_RAD, TWO_HUNDRED_SEVENTY_DEG_RAD);

            GamepadEx driverOp = new GamepadEx(gamepad1);
            kiwiDrive.driveRobotCentric(-driverOp.getLeftY(), -driverOp.getLeftX(), -driverOp.getRightX());
            if (driverOp.getButton(GamepadKeys.Button.A)) {
                driveMotorI.motor.setPower(1);
            }
        }
    }
}
