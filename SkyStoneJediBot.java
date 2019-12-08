package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.ImageHelper;
import org.firstinspires.ftc.teamcode.vision.VuHelper;
import java.io.File;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

public class SkyStoneJediBot extends WallEGPSMeccaBot {
    public LinearOpMode op;
    private VuHelper vu;
    public ImageHelper im = new ImageHelper();
    public boolean isRed = true;
    public double xMultiplier = -1;
    public static final double RED_START_X = -38.5*ENCODER_PER_INCH;

    // X coordinate for grabbing each of the six stones depending on the skystone place.
    // It is implemented as a 3 dimensional array in the format of [side][skyOrder][stoneNum]
    public static final double[][][] STONES_X = {
            {{-58*ENCODER_PER_INCH,-37*ENCODER_PER_INCH,-21*ENCODER_PER_INCH,-29*ENCODER_PER_INCH,-45*ENCODER_PER_INCH,-53*ENCODER_PER_INCH
            },
            {-57*ENCODER_PER_INCH,-29*ENCODER_PER_INCH,-21*ENCODER_PER_INCH,-37*ENCODER_PER_INCH,-45*ENCODER_PER_INCH,-54*ENCODER_PER_INCH
            },
            {-50*ENCODER_PER_INCH,-21*ENCODER_PER_INCH,-28*ENCODER_PER_INCH,-36*ENCODER_PER_INCH,-53*ENCODER_PER_INCH,-54*ENCODER_PER_INCH
            }},
            {{58*ENCODER_PER_INCH,39*ENCODER_PER_INCH,23*ENCODER_PER_INCH,31*ENCODER_PER_INCH,47*ENCODER_PER_INCH,55*ENCODER_PER_INCH
            },{57*ENCODER_PER_INCH,31*ENCODER_PER_INCH,23*ENCODER_PER_INCH,39*ENCODER_PER_INCH,47*ENCODER_PER_INCH,56*ENCODER_PER_INCH
            },{50*ENCODER_PER_INCH,23*ENCODER_PER_INCH,31*ENCODER_PER_INCH,39*ENCODER_PER_INCH,55*ENCODER_PER_INCH,56*ENCODER_PER_INCH}
    }};

    //Y coordinate for grabbing each of the six stones.  Skystone order does not matter.
    // It is a 2 dimensional array in the format of [side][stoneNum]
    public static final double[][] STONES_Y = {
            {30*ENCODER_PER_INCH,28.5*ENCODER_PER_INCH,28.5*ENCODER_PER_INCH,28.5*ENCODER_PER_INCH,32*ENCODER_PER_INCH,32*ENCODER_PER_INCH},
            {30*ENCODER_PER_INCH,31*ENCODER_PER_INCH,31*ENCODER_PER_INCH,31*ENCODER_PER_INCH,33*ENCODER_PER_INCH,33*ENCODER_PER_INCH}
    };

    //Orientation for grabbing each of the six stones in the format of [side][skyOrder][stoneNum]
    public static final double[][][] STONES_O = {
            {{-0.4,0,0,0,0,0},{0,0,0,0,0,-0.5},{0,0,0,0,0,-0.5}},
            {{0.4,0,0,0,0,0}, {0,0,0,0,0,0.5}, {0,0,0,0,0,0.5}}
    };

    //Collect setting for grabbing each of the six stones, side does not matter.
    //It is a 2 dimensional array in the format of [skyOrder][stoneNum]
    public static final double[][] STONES_C = {
            {13.5*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH},
            {13*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,5*COLLECT_ENCODER_PER_INCH},
            {13*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,7.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,4.5*COLLECT_ENCODER_PER_INCH,5*COLLECT_ENCODER_PER_INCH}
    };

    public static final int RED_SIDE = 0;
    public static final int BLUE_SIDE = 1;
    public int side = RED_SIDE;
    public int stoneNum = 0;
    public static final double START_Y = 17.75/2*ENCODER_PER_INCH;
    public double startX = RED_START_X;
    public int skyOrder = ImageHelper.SKY_INSIDE;

    public void setOP(LinearOpMode callingOP, VuHelper masterVu, boolean isBlue) {
        vu = masterVu;
        op = callingOP;
        if (isBlue) {
            isRed = false;
            xMultiplier = 1;
            startX = -RED_START_X;
            side = BLUE_SIDE;
        }
    }

    private void getStarted(){
        wallELift.setTargetPosition(0);
        wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        startGPS(30);
        setBreakMode();

        // Wait for the game to start (driver presses PLAY)
        while (!op.isStarted()&& !op.isStopRequested()) {
            op.telemetry.addData("Have been waiting ", period.seconds());
            op.telemetry.update();
        }
        period.reset();
    }

    public void moveToFoundation() throws InterruptedException{
        getStarted();
        tareIMU();
        double targetX = -47*ENCODER_PER_INCH*xMultiplier;
        resetGPS(targetX,9*ENCODER_PER_INCH,0);
        double targetY = 30*ENCODER_PER_INCH;
        prepareMove(targetX,targetY,0);
        double power = AUTO_MIN_POWER;
        while(op.opModeIsActive() && gps.odoData[Y_INDEX]<23*ENCODER_PER_INCH){
            if(power < MAX_P) power += P_INCREASE;
            moveGPS(power);
        }
        foundationMover.setPosition(FOUNDATIONMOVERDOWN);
        foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
        while(op.opModeIsActive() && power > STOP_P){
            power -= P_INCREASE;
            moveGPS(power);
        }
        targetX = -70*ENCODER_PER_INCH*xMultiplier;
        targetY = gps.odoData[Y_INDEX];
        prepareMove(targetX,targetY,0);
        power = AUTO_MIN_POWER;
        while(op.opModeIsActive() && Math.abs(gps.odoData[X_INDEX]) < 50*ENCODER_PER_INCH){
            if(power < MAX_P) power += P_INCREASE;
            moveGPS(power);
        }
        while(op.opModeIsActive() && power > STOP_P){
            power -= P_INCREASE;
            moveGPS(power);
        }
        grabFoundation();
    }

    public void moveOutOfFoundation() throws InterruptedException{
        double targetX = 0;
        double targetY = 12*ENCODER_PER_INCH;
        prepareMove(targetX,targetY,0);
        boolean seeFoundation = true;
        double power = AUTO_MIN_POWER;
        while(op.opModeIsActive() && seeFoundation){
            moveGPS(power);
            if(power < MAX_P)power += P_INCREASE;
            im.setImage(vu.getImageFromFrame());
            if(isRed)seeFoundation = im.seeRedFoundation();
            else seeFoundation = im.seeBlueFoundation();
        }
        while(op.opModeIsActive() && power > STOP_P){
            power -= P_INCREASE;
            moveGPS(power);
        }
        turnIMUAbs(1,Math.PI*xMultiplier);
    }

    private int backStone(boolean grabSky) throws InterruptedException{
        double targetX = 70.5 * ENCODER_PER_INCH * xMultiplier;
        double targetY = 12 *ENCODER_PER_INCH;
        double targetO = -Math.PI/2 * xMultiplier;
        double power = AUTO_MIN_POWER+0.1;
        prepareMove(targetX,targetY,targetO);
        while(op.opModeIsActive() && gps.stopped)moveGPS(power);
        boolean notOnTape = true;
        while(op.opModeIsActive() && notOnTape && !gps.stopped){
            int hue;
            if(isRed){
                hue = getRightHue();
                notOnTape = !(hue> 180 && hue < 240);
            }else{
                hue = getLeftHue();
                notOnTape = !((hue > 0 && hue < 50)||(hue > 310 && hue < 360));
            }
            if(power < MAX_P) power += P_INCREASE;
            moveGPS(power);
        }
        while(op.opModeIsActive() && !gps.stopped)moveGPS(STOP_P);
        gps.resetX((70.5-17.75/2)*ENCODER_PER_INCH*xMultiplier);
        targetX = (70.5-17.75/2-5)*ENCODER_PER_INCH*xMultiplier;
        prepareMove(targetX,targetY,targetO);
        while(op.opModeIsActive() && Math.abs(gps.odoData[X_INDEX]) > 59*ENCODER_PER_INCH)moveGPS(AUTO_MIN_POWER+0.1);
        turnIMUAbs(1,0);
        targetX = (70.5-23.5)*ENCODER_PER_INCH*xMultiplier;
        targetO = 0;
        prepareMove(targetX, targetY, targetO);
        notOnTape = true;
        while(op.opModeIsActive() && notOnTape){
            int hue;
            if(isRed){
                hue = getRightHue();
                notOnTape = !(hue> 180 && hue < 240);
            }else{
                hue = getLeftHue();
                notOnTape = !((hue > 0 && hue < 50)||(hue > 310 && hue < 360));
            }
            moveGPS(AUTO_MIN_POWER+0.1);
        }
        targetX = gps.odoData[X_INDEX];
        targetY = 0;
        power = AUTO_MIN_POWER;
        prepareMove(targetX,targetY,targetO);
        while(op.opModeIsActive() && gps.stopped)moveGPS(power);
        while(op.opModeIsActive() && !gps.stopped){
            if(power < MAX_P) power += P_INCREASE;
            moveGPS(power);
        }
        stopMotor();
        im.setImage(vu.getImageFromFrame());
        im.takePicture();
        if(isRed) skyOrder = im.findStonesBlue();
        else skyOrder = im.findStonesRed();
        if(grabSky) {
            if (skyOrder == ImageHelper.SKY_OUTSIDE) skyOrder = ImageHelper.SKY_INSIDE;
            else if (skyOrder == ImageHelper.SKY_INSIDE) skyOrder = ImageHelper.SKY_OUTSIDE;
        }else {
            if (skyOrder == ImageHelper.SKY_CENTER) skyOrder = ImageHelper.SKY_OUTSIDE;
            else skyOrder = ImageHelper.SKY_CENTER;
        }
        wallECollect.setTargetPosition((int)STONES_C[skyOrder][stoneNum]);
        wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wallECollect.setPower(.5);
        op.sleep(200);
        gps.resetY(9*ENCODER_PER_INCH);
        pickAndDrop.setPosition(PICK_UP);
        double rotateArmPos = ROTATE_BLUE_BACK + STONES_O[side][skyOrder][stoneNum]*SERVO_PER_RADIAN;
        rotateArm.setPosition(rotateArmPos);
        pickFirstStone();
        targetX = (70.5-30)*ENCODER_PER_INCH*xMultiplier;
        targetY = 0;
        prepareMove(targetX,targetY,targetO);
        power = AUTO_MIN_POWER;
        while(op.opModeIsActive() && gps.stopped)moveGPS(power);
        while(op.opModeIsActive() && !gps.stopped){
            if(power < MAX_P) power += P_INCREASE;
            moveGPS(power);
        }
        gps.resetY(17.75/2*ENCODER_PER_INCH);
        targetY = 13 * ENCODER_PER_INCH;
        prepareMove(targetX,targetY,targetO);
        while(op.opModeIsActive() && gps.odoData[Y_INDEX] < 11*ENCODER_PER_INCH)moveGPS(AUTO_MIN_POWER+0.1);
        turnIMUAbs(1,-Math.PI/2*xMultiplier);
        stopMotor();
        return skyOrder;
    }

    public int firstStoneEnigma() throws InterruptedException{
        getStarted();
        double targetX = 15 * ENCODER_PER_INCH * xMultiplier;
        imuTare = -Math.PI/2 * xMultiplier;
        resetGPS(targetX, 8.5*ENCODER_PER_INCH, imuTare);
        double targetY = 14 * ENCODER_PER_INCH;
        double targetO = imuTare;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        while(op.opModeIsActive() && gps.odoData[Y_INDEX] < 11*ENCODER_PER_INCH)moveGPS(AUTO_MIN_POWER+0.1);
        stopMotor();
        foundationMover.setPosition(FOUNDATIONMOVERDOWN);
        foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
        while(period.seconds() < 2){}
        return backStone(true);
    }

    private void pickFirstStone() {
        double targetY = STONES_Y[side][0];
        double targetX = STONES_X[side][skyOrder][0];
        double targetO = STONES_O[side][skyOrder][0];
        wallELift.setPower(.85);
        double power = AUTO_MIN_POWER + P_INCREASE;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        while (op.opModeIsActive() && distanceTo > 1.5*ENCODER_PER_INCH){
            distanceTo = moveGPS(power);
            double inchAway = distanceTo /ENCODER_PER_INCH;
            if(inchAway < 8) {
                if (power > STOP_P) power -= P_INCREASE;
                wallELift.setTargetPosition((int)(-200+inchAway/10*100));
                pickAndDrop.setPosition(PICK_DOWN+(PICK_UP-PICK_DOWN)*inchAway/10);
            } else if(power < MAX_P) power += P_INCREASE;
        }
        stopMotor();
        wallELift.setTargetPosition(-400);
        op.sleep(190);
        pickAndDrop.setPosition(PICK_DOWN);
        getOrientation();
        gps.resetOrientation(-(getHeading()-imuTare));
        op.sleep(190);
        wallELift.setTargetPosition(1800);
        foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        op.sleep(700);
    }

    public int getFirstStoneGP() throws InterruptedException{
        getStarted();
        im.setImage(vu.getImageFromFrame());
        tareIMU();
        resetGPS(startX,START_Y,0);
        double rotateArmPos;
        int index =0;
        double targetX = 11* ENCODER_PER_INCH * xMultiplier + startX;
        double targetY = 2*ENCODER_PER_INCH + START_Y;
        double targetO = 0;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        double power = AUTO_MIN_POWER;
        while(op.opModeIsActive() && distanceTo > ENCODER_PER_INCH && index < 20){
            distanceTo = moveGPS(power);
            power += P_INCREASE;
            index++;
            if(index == 2){
                foundationMover.setPosition(FOUNDATIONMOVERDOWN);
                foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
            }else if (index ==4) im.takePicture();
            else if (index == 6){
                if(isRed) skyOrder = im.findStonesRed();
                else skyOrder = im.findStonesBlue();
            } else if (index ==12) {
                wallECollect.setTargetPosition((int)STONES_C[skyOrder][stoneNum]);
                wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wallECollect.setPower(.5);
            } else if (index ==16){
                rotateArmPos = ROTATE_BLUE_BACK + STONES_O[side][skyOrder][stoneNum]*SERVO_PER_RADIAN;
                rotateArm.setPosition(rotateArmPos);
                pickAndDrop.setPosition(PICK_UP);
            }
        }
        pickFirstStone();
        return skyOrder;
    }

    public void pick2ndStone() throws InterruptedException{
        stoneNum = 1;
        double targetX = STONES_X[side][skyOrder][stoneNum];
        double targetY = STONES_Y[side][stoneNum];
        double targetO = Math.PI/2*(-xMultiplier);
        double power = MAX_P;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        while (op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH){
            distanceTo = moveGPS(power);
            if(distanceTo < 7.* ENCODER_PER_INCH){
                power -= P_INCREASE;
                if(power < STOP_P) power = STOP_P;
            }
        }
        rotateArm.setPosition(ROTATE_BLUE_BACK);
        wallECollect.setTargetPosition((int)STONES_C[skyOrder][stoneNum]);
        turnIMUAbs(1,0.08*(-xMultiplier));
        wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH+(int)STONES_C[skyOrder][stoneNum]);
        wallELift.setTargetPosition(-200);
        pickAndDrop.setPosition((PICK_DOWN+PICK_UP)/2);
        double stopWatch = period.milliseconds() + 200;
        im.setImage(vu.getImageFromFrame());
        double diff =  (double)im.findSkyCenter() - 580.0;
        getOrientation();
        gps.resetOrientation(-(getHeading()-imuTare));
        while(op.opModeIsActive() && period.milliseconds() < stopWatch){}
        if(diff > 20){
            targetO = gps.odoData[O_INDEX]+diff/1000.;
            runMecca(0.,0.,0.2);
            while(op.opModeIsActive() && gps.odoData[O_INDEX] < targetO){}
            stopMotor();
        }else if (diff < -20){
            targetO = gps.odoData[O_INDEX]-diff/1500.;
            runMecca(0., 0., -0.2);
            while(op.opModeIsActive() && gps.odoData[O_INDEX] > targetO){}
            stopMotor();
        }
        wallELift.setTargetPosition(-500);
        op.sleep(200);//stopWatch+= 300;
        //while(op.opModeIsActive() && period.milliseconds() < stopWatch){}
        pickAndDrop.setPosition(PICK_DOWN);
        op.sleep(200);//stopWatch += 600;
        //while(op.opModeIsActive() && period.milliseconds()<stopWatch){}
        wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        wallELift.setTargetPosition(1500);
        op.sleep(600);//stopWatch += 600;
        //while(op.opModeIsActive() && period.milliseconds() < stopWatch){}
    }

    public double pickOutsideStone(int stone, int collectOffset){
        stoneNum = stone;
        double targetX = STONES_X[side][skyOrder][stoneNum];
        double targetY = STONES_Y[side][stoneNum];
        double targetO = Math.PI/2*(-xMultiplier);
        double power = MAX_P;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        while (op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH ){
            distanceTo = moveGPS(power);
            if(distanceTo < 6.* ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
        }
        int targetC = (int)((43.5-8.5-gps.odoData[Y_INDEX]/ENCODER_PER_INCH)*COLLECT_ENCODER_PER_INCH);
        rotateArm.setPosition(ROTATE_BLUE_BACK);
        wallECollect.setTargetPosition(targetC);
        turnIMUAbs(1.5,-0.08*(xMultiplier));
        wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH+targetC);
        wallELift.setTargetPosition(-200);
        pickAndDrop.setPosition((PICK_DOWN+PICK_UP)/2);
        op.sleep(200);
        wallELift.setTargetPosition(-500);
        op.sleep(199);
        return gps.odoData[X_INDEX]/ENCODER_PER_INCH;
    }

    public void pickOutSideStoneSecondPart(){
        pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
        getOrientation();
        gps.resetOrientation(-(getHeading()-imuTare));
        op.sleep(150);
        wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        wallELift.setTargetPosition(-100);
    }

    public void hookFoundation() throws InterruptedException{
        double targetX = gps.odoData[X_INDEX];
        double targetY = gps.odoData[Y_INDEX]+10.* ENCODER_PER_INCH;
        double targetO = 0;
        double power = AUTO_MIN_POWER+0.1;
        double distanceTo = prepareMove(targetX,targetY,targetO);
        int timesSeeFoundation = 0;
        while(op.opModeIsActive() && distanceTo > ENCODER_PER_INCH  && timesSeeFoundation <3){
            distanceTo = moveGPS(power);
            im.setImage(vu.getImageFromFrame());
            if(im.isAtFoundation())timesSeeFoundation++;
            else timesSeeFoundation = 0;
        }
        foundationMover.setPosition(FOUNDATIONMOVERLOWLIMIT);
        foundationMover2.setPosition(FOUNDATIONMOVER2HIGHLIMIT);
        op.sleep(100);
        stopMotor();
        op.sleep(150);
        getOrientation();
        gps.resetOrientation(-(getHeading()-imuTare));
    }

    public void grabFoundation() throws InterruptedException{
        hookFoundation();
        double targetX = gps.odoData[X_INDEX];
        double targetY = 0* ENCODER_PER_INCH;
        double targetO = 0;
        prepareMove(targetX,targetY,targetO);
        double power = 0.4;
        pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
        while(op.opModeIsActive()&& gps.stopped)moveGPS(power);
        while(op.opModeIsActive() && !gps.stopped)moveGPS(power);
        foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        stopMotor();
        gps.resetY(9*ENCODER_PER_INCH);
    }

    public void avoidCrack(){
        double tempTargetX = gps.odoData[X_INDEX];
        double tempTargetY = gps.odoData[Y_INDEX] + 2* ENCODER_PER_INCH;
        double tempTargetO = gps.odoData[O_INDEX];
        double distanceTo = prepareMove(tempTargetX,tempTargetY,tempTargetO);
        while(op.opModeIsActive() && distanceTo > ENCODER_PER_INCH){
            distanceTo = moveGPS(0.5);
        }
    }

    public void writeRobot(){
        File robotXFile = AppUtil.getInstance().getSettingsFile("xEncoder.txt");
        File robotYFile = AppUtil.getInstance().getSettingsFile("yEncoder.txt");
        File robotOFile = AppUtil.getInstance().getSettingsFile("orientation.txt");
        File robotStoneFile = AppUtil.getInstance().getSettingsFile("numStone.txt");
        File robotSkyFile = AppUtil.getInstance().getSettingsFile("skyOrder.txt");
        File robotCollectFile = AppUtil.getInstance().getSettingsFile("collect.txt");
        File robotLiftFile = AppUtil.getInstance().getSettingsFile("lift.txt");
        ReadWriteFile.writeFile(robotXFile, String.valueOf(gps.odoData[X_INDEX]));
        ReadWriteFile.writeFile(robotYFile, String.valueOf(gps.odoData[Y_INDEX]));
        ReadWriteFile.writeFile(robotOFile, String.valueOf(gps.odoData[O_INDEX]));
        ReadWriteFile.writeFile(robotStoneFile, String.valueOf(stoneNum));
        ReadWriteFile.writeFile(robotSkyFile, String.valueOf(skyOrder));
        ReadWriteFile.writeFile(robotCollectFile, String.valueOf(wallECollect.getCurrentPosition()));
        ReadWriteFile.writeFile(robotLiftFile, String.valueOf(wallELift.getCurrentPosition()));
    }

    public void turnIMUAbs(double speed, double targetAngle){
        double stopWatch = period.seconds()+3;
        while (op.opModeIsActive() && (!onHeading(targetAngle,speed)) && (period.seconds() < stopWatch)) {}
    }
}
