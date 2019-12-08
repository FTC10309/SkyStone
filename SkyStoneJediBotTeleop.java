package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.revExtensions.RevBulkData;

import java.io.File;

import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

public class SkyStoneJediBotTeleop extends SkyStoneJediBot {
    public double foundationMoverPosition = FOUNDATIONMOVERHIGHLIMIT;
    public double foundationMover2Position = FOUNDATIONMOVER2LOWLIMIT;
    public int nextStone;
    public int liftPos = 0, collectPos = 0;
    public double liftEncoder = 0, collectEncoder = 0;

    private double targetRotateArm = ROTATE_NEUTRAL;
    private double targetIMU = Math.PI/2;
    private int targetCollect = 0;
    private double targetX = -12*ENCODER_PER_INCH;
    private double targetY = 12*ENCODER_PER_INCH;
    private double targetO = Math.PI/2;
    private double targetA = 0.;
    private double rotateArmPosition = ROTATE_NEUTRAL;
    private double pickUpStonePosition = PICK_UP;
    private boolean capstonePicked = false;
    private boolean stateIsSet = true;
    private boolean isAngleFree = true;
    private int rotateArmJiggle = 2;
    private int level = 0;
    private int[] levelLiftPos = {1441,2640,3913,5317,6084,6800,7200};
    private int[] levelCollectPos = {1352,1372,1702,1900,2048,2242,2500};
    private static final double rotateArmHighLimit = 1.;
    private static final double rotateArmLowLimit = 0.;
    private static final double pickUpStoneLowLimit = .15;
    private static final double pickUpStoneHighLimit = .85;
    private static final double INCREMENT = .02;
    private static final int ENCODERINCREMENT = 40;

    private void outSideStoneTeleop(){
        double targetX = STONES_X[side][skyOrder][nextStone]-18*ENCODER_PER_INCH*xMultiplier;
        double targetY = 46*ENCODER_PER_INCH;
        double targetO = Math.PI/2*xMultiplier;
        double power = MAX_P;
        collectEncoder = 0;
        wallECollect.setTargetPosition(0);
        turnIMUAbs(1,-targetO);
        double distanceTo = prepareMove(targetX,targetY,targetO);
        moveGPS(power);
        liftEncoder = -200;
        wallELift.setTargetPosition(-200);
        pickUpStonePosition = PICK_UP;
        pickAndDrop.setPosition(PICK_UP);
        rotateArmPosition = ROTATE_NEUTRAL;
        rotateArm.setPosition(ROTATE_NEUTRAL);
        op.sleep(300);
        while (op.opModeIsActive() && distanceTo > 2*ENCODER_PER_INCH && !gps.stopped && !op.gamepad1.x){
            distanceTo = moveGPS(power);
            if(distanceTo < 6.* ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
        }
        stopMotor();
        nextStone++;
    }

    public void initOpmode(){
        setBreakMode();
        targetX *= xMultiplier;
        targetO *= xMultiplier;
        targetIMU *= xMultiplier;
        wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallECollect.setTargetPosition(0);
        wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wallECollect.setPower(0.45);
        wallELift.setTargetPosition(0);
        wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wallELift.setPower(0.8);
        capstoneMover.setPosition(CAPSTONE_IN);
        foundationMover2.setPosition(foundationMover2Position);
        foundationMover.setPosition(foundationMoverPosition);
        startGPS(50);
        File robotXFile = AppUtil.getInstance().getSettingsFile("xEncoder.txt");
        File robotYFile = AppUtil.getInstance().getSettingsFile("yEncoder.txt");
        File robotOFile = AppUtil.getInstance().getSettingsFile("orientation.txt");
        File robotStoneFile = AppUtil.getInstance().getSettingsFile("numStone.txt");
        File robotSkyFile = AppUtil.getInstance().getSettingsFile("skyOrder.txt");
        File robotLiftFile = AppUtil.getInstance().getSettingsFile("lift.txt");
        File robotCollectFile = AppUtil.getInstance().getSettingsFile("collect.txt");
        double xValue = Double.parseDouble(ReadWriteFile.readFile(robotXFile).trim());
        double yValue = Double.parseDouble(ReadWriteFile.readFile(robotYFile).trim());
        double oValue = Double.parseDouble(ReadWriteFile.readFile(robotOFile).trim());
        resetGPS(xValue,yValue,oValue);
        imuTare = oValue;
        nextStone = Integer.parseInt(ReadWriteFile.readFile((robotStoneFile)))+1;
        skyOrder = Integer.parseInt(ReadWriteFile.readFile((robotSkyFile)));
/*        wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH - Integer.parseInt(ReadWriteFile.readFile((robotCollectFile))));
        wallELift.setTargetPosition(-Integer.parseInt(ReadWriteFile.readFile((robotLiftFile))));
        op.sleep(1000);
        wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wallECollect.setTargetPosition(0);
        wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wallECollect.setPower(0.45);
        wallELift.setTargetPosition(0);
        wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wallELift.setPower(0.8);*/
    }

    public void setBuild(){
        targetX = -(70.5-20)*ENCODER_PER_INCH;
        targetY = 45*ENCODER_PER_INCH;
        targetO = -Math.PI/2;
        targetIMU = Math.PI/2;
    }

    public void moveWallEServo(){
        double oldOne = foundationMoverPosition;
        double oldTwo = foundationMover2Position;
        if(op.gamepad1.left_bumper) {
            foundationMoverPosition -= INCREMENT;
            foundationMover2Position += INCREMENT;
        }else if(op.gamepad1.right_bumper){
            foundationMoverPosition += INCREMENT;
            foundationMover2Position -= INCREMENT;
        }
        foundationMoverPosition = Range.clip(foundationMoverPosition, FOUNDATIONMOVERLOWLIMIT, FOUNDATIONMOVERHIGHLIMIT);
        foundationMover2Position = Range.clip(foundationMover2Position,FOUNDATIONMOVER2LOWLIMIT, FOUNDATIONMOVER2HIGHLIMIT);
        if(Math.abs(foundationMoverPosition-oldOne)> 0.001 ||Math.abs(foundationMover2Position-oldTwo) > 0.001){
            foundationMover.setPosition(foundationMoverPosition);
            foundationMover2.setPosition(foundationMover2Position);
        }
    }

    public void getSensors(){
        RevBulkData auxData = auxExpansionHub.getBulkInputData();
        liftPos = auxData.getMotorCurrentPosition(wallELift);
        collectPos = auxData.getMotorCurrentPosition(wallECollect);
    }

    public void moveSlide(){
        double rightStickY = -op.gamepad2.right_stick_y;
        double maxCollect = Math.min(28.*COLLECT_ENCODER_PER_INCH, (double)MAX_COLLECT_ENCODER /
                Math.cos(((double)(liftPos))/LIFT_ENCODER_PER_RADIAN));
        double oldLift = liftEncoder;
        double oldCollect = collectEncoder;
        if(rightStickY > .9) {
            if (collectPos < maxCollect && collectPos > collectEncoder-100) collectEncoder+=ENCODERINCREMENT;
        }else if(rightStickY > .3) {
            if(collectPos < maxCollect && collectPos > collectEncoder-100) collectEncoder+=ENCODERINCREMENT/4;
        }else if(rightStickY < -.3 && collectEncoder > 0 && collectPos < collectEncoder+100) {
            if (rightStickY > -.9) collectEncoder -= ENCODERINCREMENT/4;
            else collectEncoder -= ENCODERINCREMENT;
        }
        if(collectEncoder > maxCollect)collectEncoder = (int)maxCollect;
        if(Math.abs(collectEncoder-oldCollect)>1)wallECollect.setTargetPosition((int)collectEncoder);

        if (op.gamepad2.dpad_up && liftPos < MAX_LIFT_ENCODER
                && liftPos > liftEncoder-400) liftEncoder += ENCODERINCREMENT;
        else if(op.gamepad2.dpad_down && liftPos > MIN_LIFT_ENCODER
                && liftPos < liftEncoder+400) liftEncoder -= ENCODERINCREMENT;
        if(op.gamepad1.a && op.gamepad1.y){
            wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftEncoder = 0;
            collectEncoder = 0;
            wallECollect.setTargetPosition(0);
            wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wallECollect.setPower(0.4);
            wallELift.setTargetPosition(0);
            wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wallELift.setPower(0.8);
            resetGPS(0,9*ENCODER_PER_INCH,0);
            isAngleFree = true;
            tareIMU();
        }
        if(op.gamepad1.y && !op.gamepad1.a && !stateIsSet) liftEncoder = 0;
        if(Math.abs(liftEncoder - oldLift)>1.)wallELift.setTargetPosition((int)liftEncoder);
    }

    public void moveCapstone(){
        double joyStick = -op.gamepad2.left_stick_y;
        if (joyStick > 0.9 && !capstonePicked){
            foundationMover2.setPosition(FOUNDATIONMOVER2MIDDLE);
            foundationMover.setPosition(FOUNDATIONMOVERMIDDLE);
            liftEncoder = 0.45*LIFT_ENCODER_PER_RADIAN;
            wallELift.setTargetPosition((int)liftEncoder);
            rotateArmPosition = 0.5;
            rotateArm.setPosition(rotateArmPosition);
            pickUpStonePosition = PICK_UP;
            pickAndDrop.setPosition(pickUpStonePosition);
            collectEncoder = COLLECT_ENCODER_PER_INCH;
            wallECollect.setTargetPosition((int)collectEncoder);
            op.sleep(700);
            capstoneMover.setPosition(CAPSTONE_OUT);
            capstonePicked = true;
        } else if (joyStick < -0.9 && capstonePicked){
            foundationMover.setPosition(FOUNDATIONMOVERMIDDLE);
            foundationMover2.setPosition(FOUNDATIONMOVER2MIDDLE);
            op.sleep(450);
            capstoneMover.setPosition(CAPSTONE_IN);
            op.sleep(500);
            foundationMover2Position = FOUNDATIONMOVER2DOWN;
            foundationMoverPosition = FOUNDATIONMOVERDOWN;
            foundationMover.setPosition(FOUNDATIONMOVERDOWN);
            foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
            capstonePicked = false;
        }
    }

    public void checkDriveTrain(){
        //dpad for fine driving adjustment
        double forward = 0;
        double right = 0;
        double cTurn = 0;
        double forwardJoyStick = -op.gamepad1.left_stick_y;
        double rightJoyStick = op.gamepad1.right_stick_y;
        double leftTrigger = op.gamepad1.left_trigger;
        double rightTrigger = op.gamepad1.right_trigger;
        if(op.gamepad1.dpad_left){
            if(isAngleFree){
                targetA = gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-gps.odoData[O_INDEX])*2;
            right=-0.3;
        } else if(op.gamepad1.dpad_right){
            if(isAngleFree){
                targetA = gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-gps.odoData[O_INDEX])*2;
            right=0.3;
        } else if(op.gamepad1.dpad_up){
            if(isAngleFree){
                targetA = gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-gps.odoData[O_INDEX])*2;
            forward=0.3;
        } else if(op.gamepad1.dpad_down){
            if(isAngleFree){
                targetA = gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-gps.odoData[O_INDEX])*2;
            forward=-0.3;
        } else if (op.gamepad1.x){
            isAngleFree = true;
            cTurn = -0.2;
        } else if (op.gamepad1.b){
            isAngleFree = true;
            cTurn = 0.2;
        } else if (Math.abs(forwardJoyStick)>0.2 || Math.abs(rightJoyStick) > 0.2){
            if(isAngleFree){
                targetA = gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-gps.odoData[O_INDEX])*2;
            forward = forwardJoyStick;
            right =rightJoyStick;
        } else if (Math.abs(leftTrigger)> 0.2 || Math.abs(rightTrigger)>0.2){
            cTurn = rightTrigger-leftTrigger;
            isAngleFree = true;
        }
        if(cTurn > 1.)cTurn = 1.;
        else if (cTurn <-1.)cTurn = -1.;
        runMeccaRC(forward,right,cTurn);
    }

    public void pickUpStone(){
        if(op.gamepad1.a){
            if(op.gamepad1.b){
                if(nextStone < 6) outSideStoneTeleop();
            }else if(op.gamepad1.x){
                double targetX = 57 * ENCODER_PER_INCH * xMultiplier;
                double targetY = (36 +70.5)* ENCODER_PER_INCH;
                double targetOrientation = 0;
                double distanceTo = prepareMove(targetX, targetY, targetOrientation);
                wallECollect.setTargetPosition(0);
                wallELift.setTargetPosition(-300);
                rotateArm.setPosition(ROTATE_BLUE_BACK);
                pickAndDrop.setPosition(PICK_UP);
                while(op.opModeIsActive() && gps.stopped )moveGPS(MAX_P);
                while (op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH && !gps.stopped && !op.gamepad1.b){
                    distanceTo = moveGPS(MAX_P);
                }
                targetX = 70.5 * ENCODER_PER_INCH * xMultiplier;
                distanceTo = prepareMove(targetX, targetY, targetOrientation);
                while (op.opModeIsActive() && distanceTo > ENCODER_PER_INCH && !gps.stopped && !op.gamepad1.b){
                    distanceTo = moveGPS(MAX_P);
                }
                stopMotor();
                if(!op.gamepad1.b) {
                    getOrientation();
                    gps.resetOrientation(-(getHeading() - imuTare));
                    gps.resetX(62 * ENCODER_PER_INCH * xMultiplier);
                    if (isRed) runMeccaRC(0, 0.5, 0);
                    else runMeccaRC(0, -0.5, 0);
                }
                while(op.opModeIsActive() && !op.gamepad1.b && Math.abs(gps.odoData[X_INDEX])> 61.5*ENCODER_PER_INCH){}
                targetY = 131 * ENCODER_PER_INCH;
                targetX = gps.odoData[X_INDEX];
                prepareMove(targetX, targetY, 0);
                boolean notOnTape = true;
                while (op.opModeIsActive() && notOnTape &&!op.gamepad1.b && !gps.stopped){
                    moveGPS(.5);
                    int hue = getLeftHue();
                    if(isRed) notOnTape = !(hue > 0 && hue < 50) && !(hue > 310 && hue < 360);
                    else notOnTape = !(hue> 180 && hue < 240);
                }
                stopMotor();
                if(!op.gamepad1.b)gps.resetY((141-23.5)*ENCODER_PER_INCH);
            }
        }

        double oldRotate = rotateArmPosition;
        double oldPick = pickUpStonePosition;
        double factor = 1.-op.gamepad2.right_trigger;
        if(factor < 0.9) factor = 0.5;
        else if (factor < 0.3) factor = 0.25;
        else if (factor < 0.1) factor = 0.1;
        if(op.gamepad2.left_bumper) rotateArmPosition -= INCREMENT*0.5*factor;
        else if(op.gamepad2.right_bumper)rotateArmPosition += INCREMENT*0.5*factor;
        if(op.gamepad2.x)pickUpStonePosition -= INCREMENT*factor;
        else if(op.gamepad2.y)pickUpStonePosition += INCREMENT*factor;
        boolean jiggle = op.gamepad2.dpad_left && !op.gamepad2.dpad_up && !op.gamepad2.dpad_down;
        if(jiggle){
            pickUpStonePosition = 0.67;
            rotateArmPosition += INCREMENT*rotateArmJiggle;
            rotateArmJiggle = -rotateArmJiggle;
        }
        rotateArmPosition = Range.clip(rotateArmPosition,rotateArmLowLimit,rotateArmHighLimit);
        pickUpStonePosition = Range.clip(pickUpStonePosition, pickUpStoneLowLimit,pickUpStoneHighLimit );
        if(Math.abs(rotateArmPosition-oldRotate)>0.0001) rotateArm.setPosition(rotateArmPosition);
        if(Math.abs(pickUpStonePosition-oldPick)>0.0001)pickAndDrop.setPosition(pickUpStonePosition);
        if(jiggle)rotateArmPosition+= INCREMENT*rotateArmJiggle;
    }

    public void quarrySetState(){
        if(op.gamepad2.a){
            targetX = gps.odoData[X_INDEX];
            targetY = gps.odoData[Y_INDEX];
            targetO = gps.odoData[O_INDEX];
            targetRotateArm = rotateArmPosition;
            targetCollect = collectPos;
            stateIsSet = true;
            isAngleFree = true;
        }
    }

    public void flipStone(){
        if(op.gamepad2.left_trigger>0.5){
            for (int i = 10; i<1000 && op.opModeIsActive(); i=i+10){
                double stopWatch = period.milliseconds()+20;
                liftEncoder -= 10;
                collectEncoder -= 10;
                wallELift.setTargetPosition((int)liftEncoder);
                wallECollect.setTargetPosition((int)collectEncoder);
                while(op.opModeIsActive() && period.milliseconds()<stopWatch){}
            }
        }
    }

    public void quarryReturnState(){
        if(op.gamepad2.b && stateIsSet){
            collectEncoder = 0;
            liftEncoder = -100;
            wallECollect.setTargetPosition(0);
            wallELift.setTargetPosition(-100);
            rotateArm.setPosition(targetRotateArm);
            if((isRed && gps.odoData[X_INDEX] < -9*ENCODER_PER_INCH) ||
                    (!isRed && gps.odoData[X_INDEX] > 9*ENCODER_PER_INCH)){
                double distanceTo = prepareMove(12*ENCODER_PER_INCH*xMultiplier,12*ENCODER_PER_INCH,-Math.PI/2*xMultiplier);
                while(op.opModeIsActive() && gps.stopped)moveGPS(AUTO_MIN_POWER);
                while(op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH && !gps.stopped && !op.gamepad1.a){
                    distanceTo = moveGPS(MAX_P);
                }
            }
            double distanceTo = prepareMove(targetX,targetY,targetO);
            while(op.opModeIsActive() && gps.stopped)moveGPS(AUTO_MIN_POWER);
            double power = AUTO_MIN_POWER+0.1;
            while(op.opModeIsActive() && !op.gamepad1.a &&distanceTo > 2*ENCODER_PER_INCH && !gps.stopped){
                distanceTo = moveGPS(power);
                if(distanceTo < 6*ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
                else if(power < MAX_P) power += P_INCREASE;
            }
            if(!op.gamepad1.a && !gps.stopped) {
                collectEncoder = targetCollect;
                wallECollect.setTargetPosition((int)collectEncoder);
                wallELift.setTargetPosition(0);
                liftEncoder = 0;
            }
            stopMotor();
            isAngleFree = true;
        }
    }

    public void buildSetState(){
        if(op.gamepad2.a && !stateIsSet){
            if(level < 6)level++;
            targetX = gps.odoData[X_INDEX] + 3*ENCODER_PER_INCH*xMultiplier;
            targetY = gps.odoData[Y_INDEX];
            targetO = gps.odoData[O_INDEX];
            getOrientation();
            targetIMU = getHeading()-imuTare;
            stateIsSet = true;
            isAngleFree = true;
        }
    }

    public void buildReturnState(){
        if(op.gamepad2.b && stateIsSet){
            collectEncoder = 0;
            wallECollect.setTargetPosition(0);
            liftEncoder = -100;
            wallELift.setTargetPosition(-100);

            if((isRed && gps.odoData[X_INDEX] < -9*ENCODER_PER_INCH) ||
                    (!isRed && gps.odoData[X_INDEX] > 9*ENCODER_PER_INCH)){
                double distanceTo = prepareMove(12*ENCODER_PER_INCH*xMultiplier,12*ENCODER_PER_INCH,-Math.PI/2*xMultiplier);
                while(op.opModeIsActive() && gps.stopped)moveGPS(AUTO_MIN_POWER);
                while(op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH && !gps.stopped && !op.gamepad1.a){
                    distanceTo = moveGPS(MAX_P);
                }
                distanceTo = prepareMove(-8*ENCODER_PER_INCH*xMultiplier,24*ENCODER_PER_INCH,-Math.PI*0.75*xMultiplier);
                while(op.opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH && !gps.stopped)distanceTo = moveGPS(MAX_P);
            }
            liftEncoder = levelLiftPos[level];
            wallELift.setTargetPosition((int)liftEncoder);
            double distanceTo = prepareMove(targetX,targetY,targetO);
            if(distanceTo < 5*ENCODER_PER_INCH){
                turnIMUAbs(1,targetIMU);
            }
            double power = AUTO_MIN_POWER;
            while(op.opModeIsActive() && !op.gamepad1.a && gps.stopped)moveGPS(power);
            while(op.opModeIsActive() && !op.gamepad1.a &&distanceTo > ENCODER_PER_INCH && !gps.stopped){
                distanceTo = moveGPS(power);
                if(distanceTo < 6*ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
                else if(power < MAX_P) power += P_INCREASE;
            }
            turnIMUAbs(1,targetIMU);
            stopMotor();
            if(!op.gamepad1.a ) {
                rotateArmPosition = ROTATE_NEUTRAL;
                rotateArm.setPosition(rotateArmPosition);
                collectEncoder = levelCollectPos[level];
                wallECollect.setTargetPosition((int)collectEncoder);
                stateIsSet = false;
            }
            isAngleFree = true;
        }
    }
}
