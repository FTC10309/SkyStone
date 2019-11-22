 package org.firstinspires.ftc.teamcode;

/**
 * Created by qiu29 on 9/27/2018.
 */

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;

 import org.firstinspires.ftc.teamcode.revExtensions.RevBulkData;

 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.CAPSTONE_IN;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.CAPSTONE_OUT;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.COLLECT_ENCODER_PER_INCH;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2DOWN;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2HIGHLIMIT;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2MIDDLE;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERLOWLIMIT;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERMIDDLE;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.LIFT_ENCODER_PER_RADIAN;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MAX_COLLECT_ENCODER;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MAX_LIFT_ENCODER;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MIN_LIFT_ENCODER;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_BLUE_BACK;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_NEUTRAL;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

 /**
 * Opmode for TeleOp operation
 * The following are the full list of controls available to driver (lift is front of the robot)
 *
 *  gamepad1 left joy stick: drive robot forward/backward
 *  gamepad1 right joy stick: strafing
 *  gamepad1 left/right trigger spin robot clockwise (right) /counterclockwise (left)
 *  gamepad1 dpad-left: fine strafing to the left
 *  gamepad1 dpad-right: fine strafing to the right
 *  gamepad1 dpad-up: fine turning clockwise
 *  gamepad1 dpad-down: fine turning counterclockwise

 *  gamepad2 left joy stick:   lower/extend collect system, pull back collect system.
 *  gamepad2 right joy stick:   lower collect system
 *  gamepad2 dpad-up    : lift up
 *  gamepad2 dpad-down  : lift down
 *  gamepad2 dpad-right : slow lift down (overwrite)
 *  gamepad2 dpad-left  : slow lift up (overwrite)
 *  gamepad2 X : auto dump
 *  */

@TeleOp(name="TeleOp Quarry", group="Linear Opmode")
//@Disabled
public class TeleOpQuarry extends LinearOpMode {

    // Declare OpMode members.
    private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
    private static final double INCREMENT = .02;
    private static final int ENCODERINCREMENT = 40;
    private double foundationMoverPosition = FOUNDATIONMOVERHIGHLIMIT;
    private double foundationMover2Position = FOUNDATIONMOVER2LOWLIMIT;
    private static final double rotateArmHighLimit = 1.;
    private static final double rotateArmLowLimit = -1.;
    private static final double pickUpStoneLowLimit = .15;
    private static final double pickUpStoneHighLimit = .85;
    private static final double capstoneHighLimit = CAPSTONE_OUT+0.1;
    private double capstonePosition = CAPSTONE_IN;
    private double rotateArmPosition = HardwareWallEbot.ROTATE_NEUTRAL;
    private double pickUpStonePosition = HardwareWallEbot.PICK_UP;
    private double liftEncoder = 0, collectEncoder = 0;
    private double targetRotateArm = 0.;
    private int targetLift = 0, targetCollect = 0;
    private int liftPos = 0, collectPos = 0;
    private boolean stateIsSet = false;
    private boolean isAngleFree = true;
    private double targetA = 0.;
    private boolean capstonePicked = false;
    @Override
    public void runOpMode() {
        r.period.reset();
        initOpmode();
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.update();
        }
        r.period.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getSensors();
            moveWallEServo();
            moveSlide();
            checkDriveTrain();
            pickUpStone();
            setState();
            returnState();
            flipStone();
            moveCapstone();
            telemetry.addData("X Position", r.gps.odoData[X_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Y Position", r.gps.odoData[Y_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Orientation (Radian)", r.gps.odoData[O_INDEX]);
            telemetry.addData("Lift encoder", liftPos);
            telemetry.addData("Lift target", liftEncoder);
            telemetry.addData("Collect encoder", collectPos);
            telemetry.addData("Target Lift ",targetLift);
            telemetry.addData("Target Collect ",targetCollect);
            telemetry.update();
        }
    }

    private void moveCapstone(){
        double joyStick = -gamepad2.left_stick_y;
        if (joyStick > 0.9  && !capstonePicked){
            r.foundationMover.setPosition(FOUNDATIONMOVERMIDDLE);
            r.foundationMover2.setPosition(FOUNDATIONMOVER2MIDDLE);
            liftEncoder = 0.447*LIFT_ENCODER_PER_RADIAN;
            r.wallELift.setTargetPosition((int)liftEncoder);
            rotateArmPosition = 0.5;
            r.rotateArm.setPosition(rotateArmPosition);
            pickUpStonePosition = PICK_UP;
            r.pickAndDrop.setPosition(pickUpStonePosition);
            collectEncoder = COLLECT_ENCODER_PER_INCH;
            r.wallECollect.setTargetPosition((int)collectEncoder);
            double stopWatch = r.period.milliseconds()+600;
            while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
            r.capstoneMover.setPosition(CAPSTONE_OUT);
            //stopWatch =r.period.milliseconds()+500;
            //while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
            //liftEncoder = 0.22*LIFT_ENCODER_PER_RADIAN;
            //r.wallELift.setTargetPosition((int)liftEncoder);
            //stopWatch += 500;
            //while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
            //pickUpStonePosition = 0.82;
            //r.pickAndDrop.setPosition(pickUpStonePosition);
            capstonePicked = true;
        } else if (joyStick < -0.9){
            double stopWatch = r.period.milliseconds()+500;
            r.foundationMover.setPosition(FOUNDATIONMOVERMIDDLE);
            r.foundationMover2.setPosition(FOUNDATIONMOVER2MIDDLE);
            while(opModeIsActive() && r.period.milliseconds()<stopWatch){ }
            r.capstoneMover.setPosition(CAPSTONE_IN);
            stopWatch = r.period.milliseconds()+500;
            while(opModeIsActive() && r.period.milliseconds()<stopWatch){ }
            r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
            r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
            capstonePicked = false;
        }
        joyStick = gamepad2.left_stick_x;
        if(joyStick > 0.5){
            capstonePosition += INCREMENT/40;
            if(capstonePosition > capstoneHighLimit) capstonePosition = capstoneHighLimit;
            r.capstoneMover.setPosition(capstonePosition);
        }else if (joyStick < -0.5){
            capstonePosition -= INCREMENT/40;
            if(capstonePosition < CAPSTONE_IN) capstonePosition = CAPSTONE_IN;
            r.capstoneMover.setPosition(capstonePosition);
        }
    }

    private void setState(){
        if(gamepad2.a){
            r.resetGPS(0,0,0);
            targetCollect = collectPos;
            targetLift = liftPos;
            targetRotateArm = rotateArmPosition;
            stateIsSet = true;
            isAngleFree = true;
        }
    }

    private void flipStone(){
        if(gamepad2.left_trigger>0.5){
            for (int i = 10; i<1000 && opModeIsActive(); i=i+10){
                double stopWatch = r.period.milliseconds()+20;
                liftEncoder -= 10;
                collectEncoder -= 10;
                r.wallELift.setTargetPosition((int)liftEncoder);
                r.wallECollect.setTargetPosition((int)collectEncoder);
                while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
            }
        }
    }

    private void returnState(){
        if(gamepad2.b && stateIsSet){
            collectEncoder = 600;
            r.wallECollect.setTargetPosition((int)collectEncoder);
            liftEncoder = 0;
            r.wallELift.setTargetPosition((int)liftEncoder);
            double distanceTo = r.prepareMove(0,0,0);
            double power = 0.25;
            while(opModeIsActive() && !gamepad1.a &&distanceTo > ENCODER_PER_INCH/2){
                distanceTo = r.moveGPS(power);
                if(distanceTo < 6*ENCODER_PER_INCH)power -= 0.01;
                else power += 0.01;
                if(power > 0.95)power = 0.95;
                else if(power < 0.2) power = 0.2;
                telemetry.addData("Distance to target", distanceTo/ENCODER_PER_INCH);
                telemetry.update();
            }
            while(opModeIsActive() && !gamepad1.a && Math.abs(r.gps.odoData[O_INDEX])>0.05){
                r.runMeccaRC(0,0,-r.gps.odoData[O_INDEX]*3);
            }
            r.stopMotor();
            if(!gamepad1.a) {
                rotateArmPosition = targetRotateArm;
                r.rotateArm.setPosition(rotateArmPosition);
                collectEncoder = targetCollect;
                r.wallECollect.setTargetPosition((int)collectEncoder);
                liftEncoder = targetLift;
                r.wallELift.setTargetPosition((int)liftEncoder);
                double stopWatch = r.period.milliseconds()+600;
                while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
                pickUpStonePosition = PICK_UP;
                r.pickAndDrop.setPosition(pickUpStonePosition);
                r.wallELift.setTargetPosition(0);
            }
            isAngleFree = true;
        }
    }

    private void moveSlide(){
        double rightStickY = -gamepad2.right_stick_y;
        double maxCollect = Math.min(27.*COLLECT_ENCODER_PER_INCH, (double)MAX_COLLECT_ENCODER /
                Math.cos(((double)(liftPos))/LIFT_ENCODER_PER_RADIAN));
        double oldLift = liftEncoder;
        double oldCollect = collectEncoder;
        if(rightStickY > .9) {
            if (collectPos < maxCollect &&
                    collectPos > collectEncoder-200) collectEncoder+=ENCODERINCREMENT;
        }else if(rightStickY > .3){
            if(collectPos < maxCollect &&
                    collectPos > collectEncoder-200) collectEncoder+=ENCODERINCREMENT/4;
        }else if(rightStickY < -.3 && collectEncoder > 0) {
            if (rightStickY > -.9) collectEncoder -= ENCODERINCREMENT/4;
            else collectEncoder -= ENCODERINCREMENT;
        }
        if(collectPos > maxCollect)collectPos = (int)maxCollect;
        if(Math.abs(collectEncoder- oldCollect) > 1.)r.wallECollect.setTargetPosition((int)collectEncoder);
        if (gamepad2.dpad_up && liftPos < MAX_LIFT_ENCODER
                && liftPos > liftEncoder-400) liftEncoder += ENCODERINCREMENT;
        else if(gamepad2.dpad_down && liftPos > MIN_LIFT_ENCODER
                && liftPos < liftEncoder +400) liftEncoder -= ENCODERINCREMENT;
        if(gamepad1.a && gamepad1.y){
            r.wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftEncoder = 0;
            collectEncoder = 0;
            r.wallECollect.setTargetPosition((int)collectEncoder);
            r.wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.wallECollect.setPower(0.4);
            r.wallELift.setTargetPosition((int)liftEncoder);
            r.wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.wallELift.setPower(0.8);
        //neutralLift = liftPos;
        }
        if(gamepad1.y ) liftEncoder = 0;
        if(Math.abs(liftEncoder - oldLift)>1.)r.wallELift.setTargetPosition((int)liftEncoder);
    }

    private void initOpmode(){
        r.init(hardwareMap);
        r.lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.wallECollect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.wallELift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.wallECollect.setTargetPosition(0);
        r.wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.wallECollect.setPower(0.4);
        r.wallELift.setTargetPosition(0);
        r.wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.wallELift.setPower(0.8);
        r.capstoneMover.setPosition(CAPSTONE_IN);
        r.startGPS(50);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void getSensors(){
        RevBulkData auxData = r.auxExpansionHub.getBulkInputData();
        liftPos = auxData.getMotorCurrentPosition(r.wallELift);
        collectPos = auxData.getMotorCurrentPosition(r.wallECollect);
    }

    private void checkDriveTrain(){
        //dpad for fine driving adjustment
        double forward = 0;
        double right = 0;
        double cTurn = 0;
        double forwardJoyStick = -gamepad1.left_stick_y;
        double rightJoyStick = -gamepad1.right_stick_y;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        if(gamepad1.dpad_left){
            if(isAngleFree){
                targetA = r.gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-r.gps.odoData[O_INDEX])*2;
            right=-0.3;
        } else if(gamepad1.dpad_right){
            if(isAngleFree){
                targetA = r.gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-r.gps.odoData[O_INDEX])*2;
            right=0.3;
        } else if(gamepad1.dpad_up){
            if(isAngleFree){
                targetA = r.gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-r.gps.odoData[O_INDEX])*2;
            forward=0.3;
        } else if(gamepad1.dpad_down){
            if(isAngleFree){
                targetA = r.gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-r.gps.odoData[O_INDEX])*2;
            forward=-0.3;
        } else if (gamepad1.x){
            isAngleFree = true;
            cTurn = -0.2;
        } else if (gamepad1.b){
            isAngleFree = true;
            cTurn = 0.2;
        } else if (Math.abs(forwardJoyStick)>0.2 || Math.abs(rightJoyStick) > 0.2){
            if(isAngleFree){
                targetA = r.gps.odoData[O_INDEX];
                isAngleFree = false;
            }
            cTurn = WallEMeccaBot.piToMinusPi(targetA-r.gps.odoData[O_INDEX])*2;
            forward = forwardJoyStick;
            right =rightJoyStick;
        } else if (Math.abs(leftTrigger)> 0.2 || Math.abs(rightTrigger)>0.2){
            cTurn = rightTrigger-leftTrigger;
            isAngleFree = true;
        }
        if(cTurn > 1.)cTurn = 1.;
        else if (cTurn <-1.)cTurn = -1.;
        r.runMeccaRC(forward,right,cTurn);
    }

    private void moveWallEServo(){
        double oldOne = foundationMoverPosition;
        double oldTwo = foundationMover2Position;
        if(gamepad1.left_bumper) {
            foundationMoverPosition = foundationMoverPosition - INCREMENT;
            foundationMover2Position = foundationMover2Position + INCREMENT;
        }
        if(gamepad1.right_bumper){
            foundationMoverPosition = foundationMoverPosition + INCREMENT;
            foundationMover2Position = foundationMover2Position - INCREMENT;
        }
        if(foundationMoverPosition > FOUNDATIONMOVERHIGHLIMIT) foundationMoverPosition = FOUNDATIONMOVERHIGHLIMIT;
        if(foundationMoverPosition < FOUNDATIONMOVERLOWLIMIT) foundationMoverPosition = FOUNDATIONMOVERLOWLIMIT;
        if(foundationMover2Position > FOUNDATIONMOVER2HIGHLIMIT) foundationMover2Position = FOUNDATIONMOVER2HIGHLIMIT;
        if(foundationMover2Position < FOUNDATIONMOVER2LOWLIMIT) foundationMover2Position = FOUNDATIONMOVER2LOWLIMIT;
        if(Math.abs(foundationMoverPosition-oldOne)> 0.001 ||Math.abs(foundationMover2Position-oldTwo) > 0.001){
            r.foundationMover.setPosition(foundationMoverPosition);
            r.foundationMover2.setPosition(foundationMover2Position);
        }
    }

    private void pickUpStone(){
        double oldRotate = rotateArmPosition;
        double oldPick = pickUpStonePosition;
        if(gamepad2.left_bumper) {
            rotateArmPosition = rotateArmPosition - INCREMENT*0.2;
        }
        if(gamepad2.right_bumper){
            rotateArmPosition = rotateArmPosition + INCREMENT*0.2;
        }
        if(gamepad2.x){
            pickUpStonePosition = pickUpStonePosition - INCREMENT;
        }
        if(gamepad2.y){
            pickUpStonePosition = pickUpStonePosition + INCREMENT;
        }
        if(rotateArmPosition > rotateArmHighLimit) rotateArmPosition = rotateArmHighLimit;
        if(rotateArmPosition < rotateArmLowLimit) rotateArmPosition = rotateArmLowLimit;
        if(pickUpStonePosition > pickUpStoneHighLimit) pickUpStonePosition = pickUpStoneHighLimit;
        if(pickUpStonePosition < pickUpStoneLowLimit) pickUpStonePosition = pickUpStoneLowLimit;
        if(Math.abs(rotateArmPosition-oldRotate)>0.001) r.rotateArm.setPosition(rotateArmPosition);
        if(Math.abs(pickUpStonePosition-oldPick)>0.001)r.pickAndDrop.setPosition(pickUpStonePosition);
    }
}