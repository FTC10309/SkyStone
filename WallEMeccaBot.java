package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

// This class provides data and method for driving a meccanum bot and odometery
public class WallEMeccaBot extends HardwareWallEbot {
    //AndyMark NeveRest 40 motor, 103 ticks per round, 4 in wheels,
    private static final double             TICK_MM                 = 3.511;
    private static final double             HEADING_THRESHOLD       = 0.02;

    private double lFrontPower      = 0.0;
    private double rFrontPower      = 0.0;
    private double lBackPower       = 0.0;
    private double rBackPower       = 0.0;

    private double imuTare          = 0;
    private int lFrontOldEncoder    = 0;
    private int rFrontOldEncoder    = 0;
    private int lBackOldEncoder     = 0;
    private int rBackOldEncoder     = 0;

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
        runMeccaRC(forward,right,cTurn, true,true);
    }

    /*robot centric driving for Meccanum drive, have option to turn either or both sides off
     by setting leftBackOn/rightBackOn to false*/
    public void runMeccaRC(double forward, double right, double cTurn, boolean leftBackOn, boolean rightBackOn){
        if (Math.abs(right)>0.05) forward = forward * 0.83; // Jess strafes only 4/5 efficient
        if (forward < 0 && !leftBackOn) { //put left wheels on very low power
            lFrontPower = -0.05;
            lBackPower = -0.05;
        }else {
            lFrontPower = forward + cTurn + right;
            lBackPower = forward + cTurn - right;
        }
        if (forward < 0 && !rightBackOn){ //put right wheels on very low power
            rFrontPower = -0.05;
            rBackPower = -0.05;
        }else{
            rFrontPower = forward - cTurn - right;
            rBackPower = forward - cTurn + right;
        }
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
    private double piToMinusPi(double angle){
        while (angle > Math.PI){angle -= Math.PI*2;}
        while (angle < -Math.PI){angle += Math.PI*2;}
        return angle;
    }

    private void clipPower(){  //normalize powers, so that maxpower is not beyond -1 to 1.
        double largerPower = 1;
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
        String result = "";
        result += "Power lf: "+ String.format("%.2f",lFrontPower);
        result += "; rf: " + String.format("%.2f",rFrontPower);
        result += "; lb: " + String.format("%.2f",lBackPower);
        result += "; rb: " + String.format("%.2f",rBackPower);
        result += "; Encoder :" + String.format("%4d",(lFront.getCurrentPosition()-lFrontOldEncoder));
        result += "; " + String.format("%4d",(rFront.getCurrentPosition() - rFrontOldEncoder));
        result += "; " + String.format("%4d",(lBack.getCurrentPosition() - lBackOldEncoder));
        result += "; " + String.format("%4d",(rBack.getCurrentPosition() - rBackOldEncoder));
        result += "; x : " + String.format("%6.1f", linearXMM());
        result += "; y : " + String.format("%6.1f", linearYMM());
        return result;
    }

    public void resetEncoder(){ //update the values for last time
        lFrontOldEncoder = lFront.getCurrentPosition();
        rFrontOldEncoder = rFront.getCurrentPosition();
        lBackOldEncoder = lBack.getCurrentPosition();
        rBackOldEncoder = rBack.getCurrentPosition();
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
        return piToMinusPi(targetAngle - getHeading());
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double linearYMM(){ //Odometery in forward/backward
        return (lFront.getCurrentPosition()- lFrontOldEncoder+
                lBack.getCurrentPosition() - lBackOldEncoder +
                rFront.getCurrentPosition() - rFrontOldEncoder+
                rBack.getCurrentPosition() - rBackOldEncoder) / 4. / TICK_MM;
    }

    public double linearXMM(){
        //Odometery in strafing direction, strafing slips so only 4/5 is counted
        return(lFront.getCurrentPosition() - lFrontOldEncoder +
            lBackOldEncoder - lBack.getCurrentPosition() +
            rFrontOldEncoder - rFront.getCurrentPosition() +
            rBack.getCurrentPosition() - rBackOldEncoder) *0.207 / TICK_MM;
    }
}
