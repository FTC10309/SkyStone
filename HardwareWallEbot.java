/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.revExtensions.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.revExtensions.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revExtensions.RevBulkData;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor    channel:  Left front drive motor:                       "lFront"
 * Motor    channel:  Right front drive motor:                      "rFront"
 * Motor    channel:  Left back drive motor:                        "lBack"
 * Motor    channel:  Right back drive motor:                       "rBack"
 * Motor    channel:  lift motor:                                   "wallELift"
 * Motor    channel:  collection motor:                             "wallEcollect"
 * Servo    channel:  Servo to pick up stones:                      "pickAndDrop"
 * Servo    channel:  Servo to flip the dump:                       "rotateArm"
 * Servo    channel:  Servo to move foundation:                    "foundationMover"
 * I2C      channel:  2nd servo to move foundation:              "foundationMover2"
 * I2C      channel:  inertia measurement unit:                     "imu"
 * I2C      channel:  inertia measurement unit:                     "imu1"
 * Digital  channel:  Touch sensor to control left wheels:          "leftTouch"
 * Digital  channel:  Touch sensor to control right wheels:         "rightTouch"
 */

/*This class gets all the motors, servos, and sensors for WallE.
 It also provides methods to get heading, roll, and pitch*/
public class HardwareWallEbot {
    public static final double FOUNDATIONMOVERLOWLIMIT = .15;
    public static final double FOUNDATIONMOVERHIGHLIMIT = .88;
    public static final double FOUNDATIONMOVERMIDDLE = (FOUNDATIONMOVERHIGHLIMIT+FOUNDATIONMOVERLOWLIMIT)/2;
    public static final double FOUNDATIONMOVER2LOWLIMIT = .24;
    public static final double FOUNDATIONMOVER2HIGHLIMIT = .95;
    public static final double FOUNDATIONMOVER2MIDDLE = (FOUNDATIONMOVER2HIGHLIMIT+FOUNDATIONMOVER2LOWLIMIT)/2;
    public static final double FOUNDATIONMOVERDOWN = (FOUNDATIONMOVERHIGHLIMIT+FOUNDATIONMOVERLOWLIMIT)/2-0.15;
    public static final double FOUNDATIONMOVER2DOWN = (FOUNDATIONMOVER2HIGHLIMIT+FOUNDATIONMOVER2LOWLIMIT)/2+0.15;


    public static final double SERVO_PER_RADIAN = 0.2046;
    public static final int COLLECT_ENCODER_PER_INCH = 163;
    public static final int LIFT_ENCODER_PER_RADIAN = 7220;
    public static final int MAX_LIFT_ENCODER = 6300; // 50 degree, 37 inch tall when collector is 10 inch away
    public static final int MIN_LIFT_ENCODER = -1800; // 9 inch above ground, 8 inch extend of collector
    public static final int MAX_COLLECT_ENCODER = 15*COLLECT_ENCODER_PER_INCH;
    public static final double ROTATE_NEUTRAL = 0.56; //0.74;
    public static final double ROTATE_BLUE_BACK = 0.18; //0.38;
    public static final double PICK_UP = 0.2;  //0.0;
    public static final double PICK_DOWN = 0.79 ;
    public static final double CAPSTONE_IN = 0.03;
    public static final double CAPSTONE_OUT = 0.30;
    public ExpansionHubMotor    lFront = null, lBack = null, rFront = null, rBack = null;
    public ExpansionHubMotor    wallELift = null, wallECollect = null /*,left = null, right = null*/;
    public Servo                pickAndDrop = null;
    public Servo                rotateArm = null;
    public Servo                foundationMover = null;
    public Servo                foundationMover2  = null;
    public Servo                capstoneMover = null;
    public DigitalChannel       leftTouch, rightTouch = null;
    public ColorSensor          leftColor, rightColor = null;
    public ExpansionHubEx mainExpansionHub, auxExpansionHub;


    public BNO055IMU imu,imu1;

    public ElapsedTime period  = new ElapsedTime();

    /* local members. */
    HardwareMap hwMap           =  null;
    private Orientation threeAngles= null;

    /* Constructor */
    public HardwareWallEbot(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;             // Save reference to Hardware map
        initMotors();               // get Motors
        initInput();
        initSensorsAndServo();      // get Sensors and Servos
        directionofWheels();        // set Motor directions
        encoderMotors();            // set up encoders of Motors
        setMotortoZero();           // zero Motor power
        initImu();                  // initialize Imu unit
    }

    private void initInput(){
        mainExpansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        auxExpansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
    }

    public void getOrientation(){
        threeAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public double getHeading(){return threeAngles.firstAngle;}

    public double getRoll(){return threeAngles.secondAngle;}

    public double getPitch(){
        return threeAngles.thirdAngle;
    }

    private void initMotors(){
        lFront = (ExpansionHubMotor)hwMap.get(DcMotor.class, "lFront");
        lBack  = (ExpansionHubMotor)hwMap.get(DcMotor.class, "lBack");
        rFront = (ExpansionHubMotor)hwMap.get(DcMotor.class, "rFront");
        rBack  = (ExpansionHubMotor)hwMap.get(DcMotor.class, "rBack");
        wallELift = (ExpansionHubMotor)hwMap.get(DcMotor.class, "wallELift");
        wallECollect = (ExpansionHubMotor)hwMap.get(DcMotor.class, "wallEcollect");
        //left = (ExpansionHubMotor)hwMap.get(DcMotor.class, "encoderLeft");
        //right = (ExpansionHubMotor)hwMap.get(DcMotor.class, "encoderRight");
    }

    private void initImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu1 = hwMap.get(BNO055IMU.class, "imu1");
        parameters.loggingTag          = "IMU1";
        imu1.initialize(parameters);
    }

    private void setMotortoZero() {
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
        rBack.setPower(0);
        wallELift.setPower(0);
        wallECollect.setPower(0);
    }

    private void encoderMotors(){
        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wallELift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wallECollect.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }

    private void directionofWheels(){
        lFront.setDirection(DcMotor.Direction.FORWARD);
        rFront.setDirection(DcMotor.Direction.REVERSE);
        wallELift.setDirection(DcMotor.Direction.REVERSE);
        wallECollect.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.FORWARD);
        rBack.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initSensorsAndServo() {
        pickAndDrop = hwMap.get(Servo.class, "pickAndDrop");
        rotateArm   = hwMap.get(Servo.class, "rotateArm");
        foundationMover  = hwMap.get(Servo.class,"foundationMover");
        foundationMover2 = hwMap.get(Servo.class, "foundationMover2");
        capstoneMover  = hwMap.get(Servo.class,"capstone");
        leftTouch   = hwMap.get(DigitalChannel.class, "left_touch");
        leftTouch.setMode(DigitalChannel.Mode.INPUT);
        leftColor = hwMap.get(ColorSensor.class, "left_color");
        rightColor = hwMap.get(ColorSensor.class, "right_color");
        rightTouch  = hwMap.get(DigitalChannel.class, "right_touch");
        rightTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setBreakMode(){
        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wallELift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wallECollect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setFloatMode(){
        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wallELift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wallECollect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLeftHue(){
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (leftColor.red() * SCALE_FACTOR),
                (int) (leftColor.green() * SCALE_FACTOR),
                (int) (leftColor.blue() * SCALE_FACTOR),
                hsvValues);
        return (int)hsvValues[0];
    }

    public int getRightHue(){
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (rightColor.red() * SCALE_FACTOR),
                (int) (rightColor.green() * SCALE_FACTOR),
                (int) (rightColor.blue() * SCALE_FACTOR),
                hsvValues);
        return (int)hsvValues[0];
    }
}