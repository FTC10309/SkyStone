package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

// This class provides data and method for driving a meccanum bot and odometery
public class WallEMeccaBot extends HardwareWallEbot {
    //AndyMark NeveRest 40 motor, 103 ticks per round, 4 in wheels,
    private static final double             TICK_MM                 = 3.511;
    private static final double             HEADING_THRESHOLD       = 0.12;
    public double imuTare          = 0;

    public double lFrontPower      = 0.0;
    public double rFrontPower      = 0.0;
    public double lBackPower       = 0.0;
    public double rBackPower       = 0.0;

    public WallEMeccaBot(){}                //constructor

    public void tareIMU(){  // reset the coordinates for the robot so that front is 0 radian
        getOrientation();
        imuTare = getHeading();
    }

    public void stopMotor(){
        lFrontPower  = 0.0;
        rFrontPower = 0.0;
        lBackPower = 0.0;
        rBackPower = 0.0;
        runMotors();
    }

    public void runMecca(double forward, double right, double cTurn){
        //field centric driving for Meccanum drive, change forward/right based on heading
        getOrientation();
        double heading = piToMinusPi(getHeading()-imuTare);
        double temp = forward*Math.cos(heading) - right*Math.sin(heading);
        right = forward*Math.sin(heading) + right*Math.cos(heading);
        forward = temp;
        runMeccaRC(forward,right,cTurn);
    }

    public void runMeccaOdo(double forward, double right, double cTurn, double sin, double cos){
        double temp = forward*cos + right*sin;
        right = -forward*sin + right*cos;
        forward = temp;
        runMeccaRC(forward,right,cTurn);
    }

    /*robot centric driving for Meccanum drive, have option to turn either or both sides off
     by setting leftBackOn/rightBackOn to false*/
    public void runMeccaRC(double forward, double right, double cTurn){
        if (Math.abs(right)>0.1) forward = forward * 0.83; // Jess strafes only 4/5 efficient
        lFrontPower = forward + cTurn + right;
        lBackPower = forward + cTurn - right;
        rFrontPower = forward - cTurn - right;
        rBackPower = forward - cTurn + right;
        clipPower();  // normalize so that the maximum power is not beyond -1 and 1
        runMotors();
    }

    private void runMotors(){
        lFront.setPower(lFrontPower);
        rFront.setPower(rFrontPower);
        lBack.setPower(lBackPower);
        rBack.setPower(rBackPower);
    }

    //convert any angle in radian to equivalent angle between -pi and pi
    public static double piToMinusPi(double angle){
        while (angle > Math.PI){angle -= Math.PI*2;}
        while (angle < -Math.PI){angle += Math.PI*2;}
        return angle;
    }

    private void clipPower(){  //normalize powers, so that maxpower is not beyond -1 to 1.
        double largerPower = 0.99;
        largerPower = Math.max(Math.abs(lFrontPower), largerPower);
        largerPower = Math.max(Math.abs(lBackPower), largerPower);
        largerPower = Math.max(Math.abs(rFrontPower), largerPower);
        largerPower = Math.max(Math.abs(rBackPower), largerPower);
        lFrontPower = lFrontPower/largerPower;
        lBackPower = lBackPower/largerPower;
        rFrontPower = rFrontPower/largerPower;
        rBackPower = rBackPower/largerPower;
    }

    public String toString(){
        //mainBulkData = mainExpansionHub.getBulkInputData();
        String result = "";
        result += "Power lf: "+ String.format("%.2f",lFrontPower);
        result += "; rf: " + String.format("%.2f",rFrontPower);
        result += "; lb: " + String.format("%.2f",lBackPower);
        result += "; rb: " + String.format("%.2f",rBackPower);
        return result;
    }

    public void directDrive(double lf, double rf, double lb, double rb){//direct control of motors.
        lFrontPower = lf;
        rFrontPower = rf;
        lBackPower = lb;
        rBackPower = rb;
        runMotors();
    }

    public boolean onHeading(double angle, double PCoeff) {
        double error = getError(angle); // determine turn power based on +/- error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            stopMotor();
            return true;
        }else runMecca(0,0,-getSteer(error, PCoeff));
        return false;
    }

    private double getError(double targetAngle) {
        getOrientation();
        return piToMinusPi(targetAngle - (getHeading()-imuTare));
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
