//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AutoTemplate extends LinearOpMode {

    //Include all of your motors and sensors
    Actuators actuator = new Actuators();
    Sensors sensor = new Sensors();

    private AprilTagProcessor aprilTag; // store our instance of the AprilTag processor.
    private VisionPortal visionPortal; // store our instance of the vision portal.

    AprilTagDetection tagOfInterest = null;

    int GPP = 21;
    int PGP = 22;
    int PPG = 23;


    double startingPos_X = -53;
    double startingPos_Y = -49;
    double startingPos_Heading = Math.toRadians(55);

    double scanAprilTagPos_X = -26;
    double scanAprilTagPos_Y = -24;
    double scanAprilTagPos_Heading = Math.toRadians(145);

    double pickedUpArtifactsPos_X = 6;
    double pickedUpArtifactsPos_Y = -16;
    double pickedUpArtifactsPos_Heading = Math.toRadians(270);

    @Override
    public void runOpMode() throws InterruptedException {
        //Include all of your motors and sensors hardware
        actuator.init(hardwareMap);
        sensor.init(hardwareMap);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        //TODO: Set up Robot starting position here!!!
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(startingPos_X, startingPos_Y, startingPos_Heading ) );

        Action scanAprilTag = drive.actionBuilder(new Pose2d(startingPos_X, startingPos_Y, startingPos_Heading ))
                .strafeToLinearHeading(new Vector2d(scanAprilTagPos_X, scanAprilTagPos_Y), scanAprilTagPos_Heading)
                .build();

        Action splineTest = drive.actionBuilder(new Pose2d(startingPos_X, startingPos_Y, startingPos_Heading ))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0),
                        new TranslationalVelConstraint(20),
                        new ProfileAccelConstraint(-20.0, 20.0)) //Do this path faster
                .build();

        Action aprilTagGPP21 = drive.actionBuilder(new Pose2d(scanAprilTagPos_X, scanAprilTagPos_Y, scanAprilTagPos_Heading ))
                .splineTo(new Vector2d(19, -20), Math.toRadians(0))
                .splineTo(new Vector2d(36, -30), Math.toRadians(270))
                .build();

        Action aprilTagPGP22 = drive.actionBuilder(new Pose2d(scanAprilTagPos_X, scanAprilTagPos_Y, scanAprilTagPos_Heading ))
                .splineTo(new Vector2d(-5, -20), Math.toRadians(0))
                .splineTo(new Vector2d(12, -30), Math.toRadians(270))
                .build();

        Action aprilTagPPG23 = drive.actionBuilder(new Pose2d(scanAprilTagPos_X, scanAprilTagPos_Y, scanAprilTagPos_Heading ))
                .splineToSplineHeading(new Pose2d(-12, -30, Math.toRadians(270)), Math.toRadians(270))
                .build();

        Action scoreArtifacts = drive.actionBuilder(new Pose2d(pickedUpArtifactsPos_X, pickedUpArtifactsPos_Y, pickedUpArtifactsPos_Heading))
                .splineToSplineHeading(new Pose2d(startingPos_X, startingPos_Y, startingPos_Heading), Math.toRadians(270))
                .build();

        //Create more trajectories as needed

        while (!isStarted() && !isStopRequested()) {
            //include all of your hardware configurations
            actuator.init(hardwareMap);
            sensor.init(hardwareMap);
            //telemetryAprilTag();
/*
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == GPP || tag.id == PGP || tag.id == PPG) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }

            cameraEnableAndDisable();
            telemetry.update();
            // Share the CPU.
            sleep(20);

 */
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            Actions.runBlocking(new SequentialAction(splineTest));

            /*
            // Move to scan AprilTag
            Actions.runBlocking(new SequentialAction(scanAprilTag));

            //Scan AprilTag and Close the VP
            aprilTagStart();
            visionPortal.close();

            if(tagOfInterest == null) {
                telemetry.addLine("Where is the AprilTag?");
                telemetry.update();
            }
            else {
                switch(tagOfInterest.id) {
                    case 21:
                        pickedUpArtifactsPos_X = 36; pickedUpArtifactsPos_Y = -30; pickedUpArtifactsPos_Heading = Math.toRadians(270);
                        telemetry.addLine("AprilTag ID 21 GPP");
                        Actions.runBlocking(new SequentialAction(aprilTagGPP21));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();
                        break;
                    case 22:
                        pickedUpArtifactsPos_X = 12; pickedUpArtifactsPos_Y = -30; pickedUpArtifactsPos_Heading = Math.toRadians(270);
                        telemetry.addLine("AprilTag ID 22 PGP");
                        Actions.runBlocking(new SequentialAction(aprilTagPGP22));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();
                        break;
                    case 23:
                        pickedUpArtifactsPos_X = -12; pickedUpArtifactsPos_Y = -30; pickedUpArtifactsPos_Heading = Math.toRadians(270);
                        telemetry.addLine("AprilTag ID 23 PPG");
                        Actions.runBlocking(new SequentialAction(aprilTagPPG23));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();
                        break;
                }
            }

             */
        }
    }

    public void launchMotorOn() {
        //turn on your launch motor
    }

    public void moveServo(int time) {
        //move your servo
        sleep(time);
    }

    private void aprilTagStart() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == GPP || tag.id == PGP || tag.id == PPG) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
        telemetry.update();
        // Share the CPU.
        sleep(20);
    }

    private void cameraEnableAndDisable() {
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
            telemetry.addData("Streaming is", "Paused");
            telemetry.update();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
            telemetry.addData("Streaming is", "Resumed");
            telemetry.update();
        }

    }

    // telemetry about AprilTag detections
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}