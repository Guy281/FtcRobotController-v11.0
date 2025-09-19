//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag; // store our instance of the AprilTag processor.
    private VisionPortal visionPortal; // store our instance of the vision portal.

    int GPP = 21;
    int PGP = 22;
    int PPG = 23;

    AprilTagDetection tagOfInterest = null;

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
                new Pose2d(0, 0, Math.toRadians(180) ) );

        //TODO: Put your Odometry trajectory paths here!!!
        //Separate your paths by action (linearSlide movements, intake movements, servo movements, etc.)
        Action aprilTagGPP = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(36, 27), Math.toRadians(90))
                .build();

	    //TODO: Your trajectory 2 starting pose must match where the end of your trajectory 1 should be
        Action aprilTagPGP = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(-12.5, 27), Math.toRadians(90))
                .build();

	    //TODO: Your trajectory 3 starting pose must match where the end of your trajectory 2 should be
        Action aprilTagPPG = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(-11, 27), Math.toRadians(90))
                .build();

        //Create more trajectories as needed

        while (!isStarted() && !isStopRequested()) {
            //include all of your hardware configurations
            actuator.init(hardwareMap);
            sensor.init(hardwareMap);
            //telemetryAprilTag();

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
/*
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

 */

            if(tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                telemetry.update();
            }
            else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            cameraEnableAndDisable();
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            if(tagOfInterest == null) {
                telemetry.addLine("Where is the AprilTag?");
                telemetry.update();
            }
            else {
                switch(tagOfInterest.id) {
                    case 21:
                        telemetry.addLine("Case 1, IT WORKED!!!!");
                        Actions.runBlocking(new SequentialAction(aprilTagGPP));
                        telemetry.update();
                        break;
                    case 22:
                        telemetry.addLine("Case 2, IT WORKED!!!!");
                        Actions.runBlocking(new SequentialAction(aprilTagPGP));
                        telemetry.update();
                        break;
                    case 23:
                        telemetry.addLine("Case 3, IT WORKED!!!!");
                        Actions.runBlocking(new SequentialAction(aprilTagPPG));
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