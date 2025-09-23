package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp
public class VisionColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() {
        /*     Set the list of "acceptable" color swatches (matches).
         *     Only colors that you assign here will be returned.
         *     If you know the sensor will be pointing to one of a few specific colors, enter them here.
         *     Or, if the sensor may be pointed randomly, provide some additional colors that may match.
         *     Possible choices are:
         *         RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, PURPLE, MAGENTA, BLACK, WHITE
         *     Note: For the 2026 season ARTIFACT_PURPLE and ARTIFACT_GREEN have been added.
         *
         *     Note that in the example shown below, only some of the available colors are included.
         *     This will force any other colored region into one of these colors.
         *     eg: Green may be reported as YELLOW, as this may be the "closest" match.
         */
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.2, 0.2, 0.2, -0.2))
                //20% W * H centered square, can be bigger, adjust as needed
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        //Add some other colors if needed, only need to include colors your robot might detect
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                //keep resolution low to increase performance
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData("Press ", "Start");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(portal, 30); //If using FTC Dashboard for live camera feed

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //The color space values are returned as three-element int[] arrays as follows:
                // RGB   Red 0-255, Green 0-255, Blue 0-255
                // HSV   Hue 0-180, Saturation 0-255, Value 0-255
                // Note: to take actions based on the detected color, simply use the colorSwatch or color space value
                // Either in a comparison (if/else) or switch. See examples below
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();

                telemetry.addData("Best Match", result.closestSwatch);
                telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                        result.RGB[0], result.RGB[1], result.RGB[2]));
                telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                        result.HSV[0], result.HSV[1], result.HSV[2]));
                telemetry.update();

                //if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {//Do something}
                //else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {//Do something}/
                //if (result.RGB[0] > 128) {//Do something}
                //...
            }
        }
    }
}
