package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.randyzhu.claw.LoonyClaw;
import org.randyzhu.slidemgr.FourStageViperSlide;
import org.randyzhu.slidemgr.JunctionType;

import java.util.ArrayList;

@Autonomous(name = "CallieMagic")
public class CallieMagic extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    OpenCvCamera logitechC270;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagSize = 0.166;

    int left = 17;
    int middle = 18;
    int right = 19;

    AprilTagDetection tagOfInterest;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // initialize motors
        var sClaw = new SimpleServo(hardwareMap, "sClaw", 0, 300, AngleUnit.DEGREES);

        // initialize imu
        var imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        Orientation robotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        AngularVelocity robotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // initialize drivetrain
        var mecanumDrive = new SampleMecanumDrive(hardwareMap);

        // initialize gamepads
        var driverOp = new GamepadEx(gamepad1);
        var operatorOp = new GamepadEx(gamepad2);

        // set motors to brake when power is 0, this is to prevent the robot from moving when the driver lets go of the sticks
        // gets rid of the "drift" effect, and safer for operation

        // initialize slide, according to fusion 360 our claw is 63.447mm above the ground
        var slide = new FourStageViperSlide(63.447, -4000, new MotorEx(hardwareMap, "mSlide", Motor.GoBILDA.RPM_312), 2.0 / 3.0, telemetry);

        // claw
        var claw = new LoonyClaw(sClaw, 0, 82);

        // reset the position of our slide so it thinks its the right pos.
        slide.manualReset();

        // initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        logitechC270 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "w1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        logitechC270.setPipeline(aprilTagDetectionPipeline);

        logitechC270.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                logitechC270.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code " + errorCode);
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (var detectedTag : currentDetections) {
                    if (detectedTag.id == left || detectedTag.id == middle || detectedTag.id == right) {
                        tagOfInterest = detectedTag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag found");
                    tagToTelemetry(tagOfInterest);

                    if (tagOfInterest.id == left) {
                        telemetry.addLine("Left");
                        Trajectory trajectory_l = mecanumDrive.trajectoryBuilder(new Pose2d()).strafeLeft(44).build(); // inches
                        Trajectory trajectory_f = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(36).build(); // inches
                        mecanumDrive.followTrajectory(trajectory_l);
                        mecanumDrive.followTrajectory(trajectory_f);
                    } else if (tagOfInterest.id == middle) {
//                        telemetry.addLine("Middle");
//                        claw.close();
//                        slide.setHeight(80, false);
//                        Trajectory t = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(60).build();
//                        mecanumDrive.followTrajectory(t);
//                        mecanumDrive.turn(Math.toRadians(-45));
//                        slide.goToJunction(JunctionType.HIGH);
                        Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(36).build(); // inches
                        mecanumDrive.followTrajectory(trajectory);
                    } else if (tagOfInterest.id == right) {
                        telemetry.addLine("Right");
                        Trajectory trajectory_r = mecanumDrive.trajectoryBuilder(new Pose2d()).strafeRight(44).build(); // inches
                        Trajectory trajectory_f = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(36).build(); // inches
                        mecanumDrive.followTrajectory(trajectory_r);
                        mecanumDrive.followTrajectory(trajectory_f);
                    }
                } else {
                    telemetry.addLine("Tag not found");

                    if (tagOfInterest == null) {
                        telemetry.addLine("THe tag has never been seen");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }


            }
        }

//        while (opModeIsActive()) {
//            claw.open();
//            Trajectory t1 = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(60).build();
//            mecanumDrive.followTrajectory(t1);
//            mecanumDrive.turn(Math.toRadians(90));
//            Trajectory t2 = mecanumDrive.trajectoryBuilder(new Pose2d()).forward(36).build();
//            mecanumDrive.followTrajectory(t2);
//            claw.close();
//            mecanumDrive.turn(Math.toRadians(-180));
//            telemetry.update();
//            sleep(20);
//        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
