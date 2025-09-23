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
    autonomousPathConstants APC = new autonomousPathConstants();

    private AprilTagProcessor aprilTag; // store our instance of the AprilTag processor.
    private VisionPortal visionPortal; // store our instance of the vision portal.

    AprilTagDetection tagOfInterest = null;

    int GPP = 21;
    int PGP = 22;
    int PPG = 23;

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
                new Pose2d(APC.startingPos_X, APC.startingPos_Y, APC.startingPos_Heading ) );

        Action scanAprilTag = drive.actionBuilder(new Pose2d(APC.startingPos_X, APC.startingPos_Y, APC.startingPos_Heading ))
                .strafeToLinearHeading(new Vector2d(APC.scanAprilTagPos_X, APC.scanAprilTagPos_Y), APC.scanAprilTagPos_Heading)
                .build();

        Action splineTest = drive.actionBuilder(new Pose2d(APC.startingPos_X, APC.startingPos_Y, APC.startingPos_Heading ))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0),
                        new TranslationalVelConstraint(20),
                        new ProfileAccelConstraint(-20.0, 20.0))
                .build();

        Action aprilTagPPG = drive.actionBuilder(new Pose2d(APC.scanAprilTagPos_X, APC.scanAprilTagPos_Y, APC.scanAprilTagPos_Heading ))
                .splineToSplineHeading(new Pose2d(APC.firstAprilTagPPG23Pos_X, APC.firstAprilTagPPG23Pos_Y, APC.firstAprilTagPPG23Pos_Heading), Math.toRadians(270))
                .build();

        Action aprilTagPGP22 = drive.actionBuilder(new Pose2d(APC.scanAprilTagPos_X, APC.scanAprilTagPos_Y, APC.scanAprilTagPos_Heading ))
                .splineTo(new Vector2d(APC.firstAprilTagPGP22Pos_X, APC.firstAprilTagPGP22Pos_Y), APC.firstAprilTagPGP22Pos_Heading)
                .splineTo(new Vector2d(APC.secondAprilTagPGP22Pos_X, APC.secondAprilTagPGP22Pos_Y), APC.secondAprilTagPGP22Pos_Heading)
                .build();

        Action aprilTagGPP = drive.actionBuilder(new Pose2d(APC.scanAprilTagPos_X, APC.scanAprilTagPos_Y, APC.scanAprilTagPos_Heading ))
                .splineTo(new Vector2d(APC.firstAprilTagGPP21Pos_X, APC.firstAprilTagGPP21Pos_Y), APC.firstAprilTagGPP21Pos_Heading)
                .splineTo(new Vector2d(APC.secondAprilTagGPP21Pos_X, APC.secondAprilTagGPP21Pos_Y), APC.secondAprilTagGPP21Pos_Heading)
                .build();

        Action scoreArtifacts = drive.actionBuilder(new Pose2d(APC.pickedUpArtifactsPos_X, APC.pickedUpArtifactsPos_Y, APC.pickedUpArtifactsPos_Heading))
                .splineToSplineHeading(new Pose2d(APC.startingPos_X, APC.startingPos_Y, APC.startingPos_Heading), Math.toRadians(270))
                .build();

        //Create more trajectories as needed

        while (!isStarted() && !isStopRequested()) {
            //include all of your hardware configurations
            actuator.init(hardwareMap);
            sensor.init(hardwareMap);

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if(!currentDetections.isEmpty()) {
                boolean tagFound = false;
                /*
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == GPP || tag.id == PGP || tag.id == PPG) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                */
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == GPP) {
                        telemetry.addLine("AprilTag ID DETECTED");
                        telemetry.addLine("///// 21 GPP /////");
                        telemetry.addLine("///// 21 GPP /////");
                        telemetry.addLine("///// 21 GPP /////");
                        telemetry.addLine("///// 21 GPP /////");
                        telemetry.addLine("///// 21 GPP /////");
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if (tag.id == PGP) {
                        telemetry.addLine("AprilTag ID DETECTED");
                        telemetry.addLine("///// 22 PGP /////");
                        telemetry.addLine("///// 22 PGP /////");
                        telemetry.addLine("///// 22 PGP /////");
                        telemetry.addLine("///// 22 PGP /////");
                        telemetry.addLine("///// 22 PGP /////");
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if (tag.id == PPG) {
                        telemetry.addLine("AprilTag ID DETECTED");
                        telemetry.addLine("///// 23 PPG /////");
                        telemetry.addLine("///// 23 PPG /////");
                        telemetry.addLine("///// 23 PPG /////");
                        telemetry.addLine("///// 23 PPG /////");
                        telemetry.addLine("///// 23 PPG /////");
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

        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            telemetry.clear();
            telemetry.update();
            // Move to scan AprilTag
            Actions.runBlocking(new SequentialAction(scanAprilTag));

            //Scan AprilTag and Close the VP
            aprilTagStart();
            visionPortal.close();

            if(tagOfInterest == null) {
                telemetry.addLine("APRILTAG NOT FOUND");
                telemetry.addLine("APRILTAG NOT FOUND");
                telemetry.addLine("APRILTAG NOT FOUND");
                telemetry.addLine("APRILTAG NOT FOUND");
                telemetry.update();
            }
            else {
                switch(tagOfInterest.id) {
                    case 21:
                        APC.pickedUpArtifactsPos_X = APC.firstAprilTagPPG23Pos_X;
                        APC.pickedUpArtifactsPos_Y = APC.firstAprilTagPPG23Pos_Y;
                        APC.pickedUpArtifactsPos_Heading = APC.firstAprilTagPPG23Pos_Heading;
                        telemetry.addLine("AprilTag ID 21");
                        telemetry.addLine("Going to PPG");
                        Actions.runBlocking(new SequentialAction(aprilTagPPG));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();
                        break;

                    case 22:
                        APC.pickedUpArtifactsPos_X = APC.firstAprilTagPGP22Pos_X;
                        APC.pickedUpArtifactsPos_Y = APC.firstAprilTagPGP22Pos_Y;
                        APC.pickedUpArtifactsPos_Heading = APC.firstAprilTagPGP22Pos_Heading;
                        telemetry.addLine("AprilTag ID 22 PGP");
                        telemetry.addLine("Going to PGP");
                        Actions.runBlocking(new SequentialAction(aprilTagPGP22));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();

                        break;

                    case 23:
                        APC.pickedUpArtifactsPos_X = APC.firstAprilTagGPP21Pos_X;
                        APC.pickedUpArtifactsPos_Y = APC.firstAprilTagGPP21Pos_Y;
                        APC.pickedUpArtifactsPos_Heading = APC.firstAprilTagGPP21Pos_Heading;

                        telemetry.addLine("AprilTag ID 23");
                        telemetry.addLine("Going to GPP");
                        Actions.runBlocking(new SequentialAction(aprilTagGPP));
                        Actions.runBlocking(new SequentialAction(scoreArtifacts));
                        telemetry.update();
                        break;
                }
            }
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