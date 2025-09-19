// This template is a starting point on setting up your actuators
// If you are not using anything listed below, comment them out (like this line)
// Deleting them is fine too but commenting is recommended in case they are added later

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Actuators {

    // TODO: If using motors, add/edit them here (up to 8)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    // TODO: If using servos or continuous servos, add them here (up to 12)

    public void init(HardwareMap hwMap) {
        // TODO: Put motor hardware here, don't forget to rename deviceNames to match yours
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        //TODO: Setup motor mode when power is Zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        //TODO: If need to change motor direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
/*
        //TODO: Put servo hardware here, don't forget to rename deviceNames to match yours
        servoRegular = hwMap.get(Servo.class, "servoRegular");
        servoContinuous = hwMap.get(CRServo.class, "servoContinuous");
        //TODO: If need to change servo direction
        servoRegular.setDirection(Servo.Direction.FORWARD); //or REVERSE
        servoContinuous.setDirection(CRServo.Direction.FORWARD); //or REVERSE
        //TODO: If need to set the logical min and max of a servo.
        servoRegular.scaleRange(0, 1);  //Change the 0 and 1 if needed

 */
    }

    //TODO: Sample classed for turning on motors using power only (Useful for TeleOp)
    public void leftFront(double power) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setPower(power);
    }
    public void rightFront(double power) {
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setPower(power);
    }
    public void leftBack(double power) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setPower(power);
    }
    public void rightBack(double power) {
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setPower(power);
    }
    /*
    public void intakeMotor(double power) {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(power);
    }
    public void liftMotor(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(power);
    }

    //TODO: Sample classes for powering a servo
    public void setServoPosition(double position) {
        servoRegular.setPosition(position);
    }

    //TODO: Sample classes for powering a continuous servo
    public void setCRServoPower(double position) {
        servoContinuous.setPower(position);
    }

     */



}
