package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.odometry.OdoGPS;

// This class provides data and method for driving a meccanum bot and odometery
public class WallEGPSMeccaBot extends WallEMeccaBot {
    public OdoGPS gps;
    private double totalDistance = 0., targetOPerEncoder = 0.;
    private double targetX = 0, targetY = 0, targetO = 0;
    public WallEGPSMeccaBot(){}                //constructor

    public void startGPS(int sleepTime){
        gps = new OdoGPS(rFront, lFront, rBack, sleepTime, mainExpansionHub);
        Thread positionThread = new Thread(gps);
        positionThread.start();
    }

    public void resetGPS(double inputX, double inputY, double inputO){
        if(gps != null){
            gps.resetY(inputY);
            gps.resetX(inputX);
            gps.resetOrientation(inputO);
        }
    }
    public double prepareMove(double inputX, double inputY, double inputO){
        targetX = inputX;
        targetY = inputY;
        targetO = inputO;
        double currentX = gps.odoData[OdoGPS.X_INDEX];
        double currentY = gps.odoData[OdoGPS.Y_INDEX];
        double currentO = gps.odoData[OdoGPS.O_INDEX];
        double deltaX = targetX-currentX;
        double deltaY = targetY-currentY;
        totalDistance = Math.sqrt(deltaX*deltaX+deltaY*deltaY);
        if(totalDistance>1.)targetOPerEncoder = piToMinusPi(targetO-currentO)/totalDistance;
        else targetOPerEncoder = 0.;
        return totalDistance;
    }

    public double moveGPS(double power){
        double x = gps.odoData[OdoGPS.X_INDEX];
        double y = gps.odoData[OdoGPS.Y_INDEX];
        double o = gps.odoData[OdoGPS.O_INDEX];
        x = targetX-x;
        y = targetY-y;
        if(power < 0.3 && gps.stopped ){
            power = 0.3;
        }
        double distanceTo = Math.sqrt(x*x+y*y);
        double bigger = Math.max(Math.abs(x), Math.abs(y));
        x = x/bigger*power; y = y/bigger*power;
        double cTurn =0;
        double currentOTo = piToMinusPi(targetO - o);
        double targetOTo = targetOPerEncoder*distanceTo;
        if(Math.abs(currentOTo) > Math.abs(targetOTo)) {
            double oDif = piToMinusPi(targetOTo - currentOTo);
            if (oDif < -0.03) cTurn = Math.min(0.8, -oDif * 5);
            else if (oDif > 0.03) cTurn = Math.max(-0.8, -oDif * 5);
        }else{
            if(currentOTo > 0.03) cTurn = Math.min(0.2,currentOTo);
            else if (currentOTo < -0.03) cTurn = Math.max(-0.2, currentOTo);
        }
        runMeccaOdo(y, x , cTurn, gps.odoData[OdoGPS.SIN_INDEX],gps.odoData[OdoGPS.COS_INDEX]);
        return distanceTo;
    }

    public void stopGPS(){
        gps.stop();
    }
}
