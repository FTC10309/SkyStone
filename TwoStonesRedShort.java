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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.OdoGPS;
import org.firstinspires.ftc.teamcode.vision.ImageHelper;
import org.firstinspires.ftc.teamcode.vision.VuHelper;

import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2HIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERLOWLIMIT;

//Start on blue side

@Autonomous(name="Two stones red short", group="Red Linear Opmode")
//@Disabled
public class TwoStonesRedShort extends LinearOpMode {

    // Declare OpMode members.
    private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
    private static final double maxP =0.93;
    private static final double pIncrease = 0.01;
    private static final double startMinPower = 0.25;
    private static final double stopMinPower = 0.2;
    double power = startMinPower;
    double targetY = OdoGPS.ENCODER_PER_INCH;
    double targetX = -11*OdoGPS.ENCODER_PER_INCH;
    double targetO = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        VuHelper vu = new VuHelper(hardwareMap);
        ImageHelper im = new ImageHelper();
        vu.vuActivate();
        r.init(hardwareMap);
        r.wallECollect.setTargetPosition(8*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        r.wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        r.wallELift.setTargetPosition(-600);
        r.wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        r.startGPS(50);
        r.setBreakMode();
        double rotateArm = HardwareWallEbot.ROTATE_BLUE_BACK;
        int skyOrder = ImageHelper.SKY_LEFT;

        // Wait for the game to start (driver presses PLAY)
        double stopWatch = r.period.milliseconds()+60000;
        while (!isStarted()&& !isStopRequested()) {
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.update();
            if(r.period.milliseconds() > stopWatch){
                im.setImage(vu.getImageFromFrame());
                im.takePicture();
                stopWatch = r.period.milliseconds()+60000;
            }
        }
        r.period.reset();
        im.setImage(vu.getImageFromFrame());
        r.tareIMU();
        r.resetGPS(0,0,0);
        int index =0;
        double distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = startMinPower;
        while(opModeIsActive() && distanceTo > OdoGPS.ENCODER_PER_INCH && index < 20){
            distanceTo = r.moveGPS(power);
            power += pIncrease;
            index++;
            if(index == 2){
                r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
                r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
            }else if (index ==4) im.takePicture();
             else if (index == 6)skyOrder = im.findStonesRed();
             else if (index ==8) r.wallECollect.setPower(.5);
             else if (index ==10){
                if(skyOrder == ImageHelper.SKY_LEFT)rotateArm += -0.64*HardwareWallEbot.SERVO_PER_RADIAN;
                r.rotateArm.setPosition(rotateArm);
                r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
            }
        }
        if(skyOrder == ImageHelper.SKY_RIGHT){
            targetX = -11 * OdoGPS.ENCODER_PER_INCH;
            targetY = 23 * OdoGPS.ENCODER_PER_INCH;
        }else if (skyOrder == ImageHelper.SKY_CENTER) {
            targetX = -19 * OdoGPS.ENCODER_PER_INCH;
            targetY = 23 * OdoGPS.ENCODER_PER_INCH;
        }else{
            targetX = -20 * OdoGPS.ENCODER_PER_INCH;
            targetY = 25 * OdoGPS.ENCODER_PER_INCH;
            targetO = -0.55;
            r.wallECollect.setTargetPosition(9*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        }
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 3*OdoGPS.ENCODER_PER_INCH) power -= pIncrease;
            else power += pIncrease;
            if(power > maxP) power = maxP;
            else if(power < stopMinPower)power = stopMinPower;
        }
        r.stopMotor();
        while (opModeIsActive()&& r.wallECollect.getCurrentPosition()<5*HardwareWallEbot.COLLECT_ENCODER_PER_INCH){}
        r.wallELift.setPower(.85);
        r.pickAndDrop.setPosition((HardwareWallEbot.PICK_UP + HardwareWallEbot.PICK_DOWN) /2);
        while (opModeIsActive() && r.wallECollect.getCurrentPosition() < 7 * HardwareWallEbot.COLLECT_ENCODER_PER_INCH){}
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
        stopWatch = r.period.milliseconds()+500;
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        if(skyOrder == ImageHelper.SKY_LEFT) stopWatch += 800;
        else stopWatch +=300;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.wallELift.setTargetPosition(1500);
        r.runMeccaRC(-0.5,0,0);
        while(opModeIsActive() && r.gps.odoData[OdoGPS.Y_INDEX] > 23*OdoGPS.ENCODER_PER_INCH){}
        r.wallELift.setTargetPosition(0);
        r.runMeccaRC(0,0,maxP);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.2);
        targetY = 23. * OdoGPS.ENCODER_PER_INCH;
        targetO = Math.PI/2;
        targetX = 52. * OdoGPS.ENCODER_PER_INCH;
        power = maxP;
        distanceTo = r.prepareMove(targetX, targetY,targetO);
        int leftHue = 0;
        while (opModeIsActive() && !(leftHue > 0 && leftHue < 50)
                && !(leftHue > 310 && leftHue < 360)){
            distanceTo = r.moveGPS(power);
            leftHue = r.getLeftHue();
        }
        r.wallELift.setTargetPosition(1500);
        r.wallECollect.setTargetPosition(9*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        r.rotateArm.setPosition(HardwareWallEbot.ROTATE_BLUE_BACK-0.1);
        targetX=66. * OdoGPS.ENCODER_PER_INCH;
        targetY = 28. * OdoGPS.ENCODER_PER_INCH;
        targetO = 0;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 10*OdoGPS.ENCODER_PER_INCH)power -= pIncrease;
            if(power < stopMinPower) power = stopMinPower;
        }
        turnIMUAbs(1,0);
        r.stopMotor();
        r.runMeccaRC(0,0,0.7);
        stopWatch = r.period.milliseconds()+300;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch);
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
        while(opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] <  Math.PI/2-0.25);
        r.wallELift.setTargetPosition(0);
        r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        targetY = 23.* OdoGPS.ENCODER_PER_INCH;
        targetX = 48. * OdoGPS.ENCODER_PER_INCH;
        targetO = Math.PI/2;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = startMinPower;
        while (opModeIsActive() && distanceTo > 2.*OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            power += pIncrease;
            if(power > maxP)power = maxP;
        }
        if(skyOrder == ImageHelper.SKY_RIGHT)targetX = 16 * OdoGPS.ENCODER_PER_INCH;
        else if (skyOrder == ImageHelper.SKY_CENTER) targetX = 8 * OdoGPS.ENCODER_PER_INCH;
        else targetX = 0;
        targetO = Math.PI/2;
        power = maxP;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 2.*OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 5.*OdoGPS.ENCODER_PER_INCH){
                power -= pIncrease;
                if(power < stopMinPower) power = stopMinPower;
            }
        }
        r.rotateArm.setPosition(HardwareWallEbot.ROTATE_BLUE_BACK);
        r.wallECollect.setTargetPosition(8*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        turnIMUAbs(2,0.06);
        r.stopMotor();
        r.runMeccaRC(0.5,0,0);
        r.wallELift.setTargetPosition(-500);
        while(opModeIsActive() && r.gps.odoData[OdoGPS.Y_INDEX] < 25 * OdoGPS.ENCODER_PER_INCH){}
        r.stopMotor();
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
        stopWatch = r.period.milliseconds()+1000;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        r.runMeccaRC(-0.5,0,0);
        while(opModeIsActive() && r.gps.odoData[OdoGPS.Y_INDEX] > 24 * OdoGPS.ENCODER_PER_INCH);
        r.runMeccaRC(0,0,maxP);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.2){ }
        power = maxP;
        targetY = 25. * OdoGPS.ENCODER_PER_INCH;
        targetX = 70. * OdoGPS.ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        leftHue = 140;
        while (opModeIsActive() && distanceTo > 3.*OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            leftHue = r.getLeftHue();
            if((leftHue > 0 && leftHue < 50)
                    || (leftHue > 310 && leftHue < 360))r.wallELift.setTargetPosition(1500);
        }
        targetX = 88 * OdoGPS.ENCODER_PER_INCH;
        targetO = 0;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 2.*OdoGPS.ENCODER_PER_INCH) {
            distanceTo = r.moveGPS(power);
            if(distanceTo < 4.*OdoGPS.ENCODER_PER_INCH)power-= pIncrease;
            if(power < stopMinPower)power = stopMinPower;
        }
        turnIMUAbs(2,0);
        r.stopMotor();
        r.getOrientation();
        r.gps.resetOrientation(-r.getHeading());
        targetX = r.gps.odoData[OdoGPS.X_INDEX];
        targetY = 36.*OdoGPS.ENCODER_PER_INCH;
        targetO = 0;
        power = startMinPower;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while(opModeIsActive() && distanceTo< OdoGPS.ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
        }
        r.foundationMover.setPosition(FOUNDATIONMOVERLOWLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2HIGHLIMIT);
        r.stopMotor();
        stopWatch = r.period.milliseconds()+500;
        while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
        targetY = -2*OdoGPS.ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = 0.4;
        stopWatch = r.period.milliseconds() + 1000;
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
        while(opModeIsActive()&& r.period.milliseconds()<stopWatch){
            distanceTo = r.moveGPS(power);
        }
        double oldDistanceTo = distanceTo - OdoGPS.ENCODER_PER_INCH;
        while(opModeIsActive() && Math.abs(distanceTo - oldDistanceTo) > 10){
            distanceTo = r.moveGPS(power);
            oldDistanceTo = distanceTo;
        }
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        r.stopMotor();
        r.gps.resetY(0);
        targetY = OdoGPS.ENCODER_PER_INCH;
        targetX = 28.*OdoGPS.ENCODER_PER_INCH;
        targetO = 0;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = maxP;
        leftHue = 140;
        while(opModeIsActive() && !(leftHue > 0 && leftHue < 50)
                && !(leftHue > 310 && leftHue < 360)){
            distanceTo = r.moveGPS(power);
            power+=pIncrease;
            if(power > maxP)power = maxP;
            leftHue = r.getLeftHue();
        }
        r.stopMotor();
        r.gps.stop();
        vu.vuDeActivate();
    }

    public void turnIMUAbs(double speed, double targetAngle){
        double stopWatch = r.period.seconds()+3;
        while (opModeIsActive() && (!r.onHeading(targetAngle,speed)) && (r.period.seconds() < stopWatch)) {}
    }
}
