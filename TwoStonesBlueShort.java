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

import static org.firstinspires.ftc.teamcode.HardwareWallEbot.COLLECT_ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2HIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERLOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_BLUE_BACK;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.SERVO_PER_RADIAN;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

//Start on blue side

@Autonomous(name="Two stones blue", group="Blue Linear Opmode")
//@Disabled
public class TwoStonesBlueShort extends LinearOpMode {

    // Declare OpMode members.
    private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
    private static final double maxP =0.93;
    private static final double pIncrease = 0.015;
    private static final double startMinPower = 0.25;
    private static final double stopMinPower = 0.1;
    double power = startMinPower;
    double targetY = ENCODER_PER_INCH;
    double targetX = 11* ENCODER_PER_INCH;
    double targetO = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        VuHelper vu = new VuHelper(hardwareMap);
        ImageHelper im = new ImageHelper();
        vu.vuActivate();
        r.init(hardwareMap);
        int targetCollect = 13*COLLECT_ENCODER_PER_INCH;
        r.wallECollect.setTargetPosition(targetCollect);
        r.wallECollect.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        r.wallELift.setTargetPosition(0);
        r.wallELift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        r.startGPS(30);
        r.setBreakMode();
        double rotateArm = HardwareWallEbot.ROTATE_BLUE_BACK;
        int skyOrder = ImageHelper.SKY_RIGHT;

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()&& !isStopRequested()) {
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.update();
        }
        r.period.reset();
        im.setImage(vu.getImageFromFrame());
        r.tareIMU();
        r.resetGPS(0,0,0);
        int index =0;
        double distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = 0.5;
        while(opModeIsActive() && distanceTo > ENCODER_PER_INCH && index < 20){
            distanceTo = r.moveGPS(power);
            power += pIncrease;
            index++;
            if(index == 2){
                r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
                r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
            }else if (index ==4) im.takePicture();
             else if (index == 6)skyOrder = im.findStonesBlue();
             else if (index ==8) r.wallECollect.setPower(.5);
             else if (index ==10){
                if(skyOrder == ImageHelper.SKY_RIGHT)rotateArm = ROTATE_BLUE_BACK+0.40*SERVO_PER_RADIAN;
                r.rotateArm.setPosition(rotateArm);
                r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
            }
        }
        targetY = 21 * ENCODER_PER_INCH;
        if(skyOrder == ImageHelper.SKY_LEFT){
            targetX = 11.5 * ENCODER_PER_INCH;
        }else if (skyOrder == ImageHelper.SKY_CENTER) {
            targetX = 18.5 * ENCODER_PER_INCH;
        }else{
            targetX = 19.5 * ENCODER_PER_INCH;
            targetO = 0.40;
            targetCollect =(int) (13.5*COLLECT_ENCODER_PER_INCH);
            r.wallECollect.setTargetPosition(targetCollect);
        }
        r.wallELift.setPower(.85);
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 1.5*ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            double inchAway = distanceTo /ENCODER_PER_INCH;
            if(inchAway < 8) {
                power -= pIncrease;
                r.wallELift.setTargetPosition((int)(-200+inchAway/10*100));
                r.pickAndDrop.setPosition(PICK_DOWN+(PICK_UP-PICK_DOWN)*inchAway/10);
            } else power += pIncrease;
            if(power > maxP) power = maxP;
            else if(power < stopMinPower)power = stopMinPower;
        }
        r.stopMotor();
        double stopWatch = r.period.milliseconds()+200;
        r.wallELift.setTargetPosition(-400);
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
        stopWatch += 300;
        r.getOrientation();
        r.gps.resetOrientation(-(r.getHeading()-r.imuTare));
        while (opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.wallELift.setTargetPosition(1800);
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        stopWatch += 700;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.runMeccaRC(0,0,-0.7);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.4);
        r.wallELift.setTargetPosition(0);
        targetY = 24. * ENCODER_PER_INCH;
        targetO = -Math.PI/2;
        targetX = -52. * ENCODER_PER_INCH;
        power = maxP;
        distanceTo = r.prepareMove(targetX, targetY,targetO);
        int hue = 0;
        while (opModeIsActive() && !(hue> 180 && hue < 240)){
            distanceTo = r.moveGPS(power);
            hue = r.getRightHue();
        }
        r.wallELift.setTargetPosition(1700);
        r.wallECollect.setTargetPosition(16*COLLECT_ENCODER_PER_INCH);
        r.rotateArm.setPosition(ROTATE_BLUE_BACK+0.2);
        targetX=-62. * ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 2*ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 8* ENCODER_PER_INCH)power -= pIncrease;
            if(power < stopMinPower) power = stopMinPower;
        }
        turnIMUAbs(1,-0.08);
        r.runMeccaRC(0,0,-0.7);
        stopWatch = r.period.milliseconds()+50;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch);
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
        while(opModeIsActive() && r.gps.odoData[O_INDEX]> -Math.PI/2+0.4){}
        r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        targetY = 21.* ENCODER_PER_INCH;
        targetX = -48. * ENCODER_PER_INCH;
        targetO = -Math.PI/2;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = startMinPower;
        while (opModeIsActive() && distanceTo > 2.* ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            power += pIncrease;
            if(power > maxP)power = maxP;
        }
        if(skyOrder == ImageHelper.SKY_LEFT)targetX = -14* ENCODER_PER_INCH;
        else if (skyOrder == ImageHelper.SKY_CENTER) targetX = -6 * ENCODER_PER_INCH;
        else targetX = 2*ENCODER_PER_INCH;
        power = maxP;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 2*ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 6.* ENCODER_PER_INCH){
                power -= pIncrease;
                if(power < stopMinPower) power = stopMinPower;
            }
        }
        r.rotateArm.setPosition(HardwareWallEbot.ROTATE_BLUE_BACK);
        r.wallECollect.setTargetPosition(8*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        turnIMUAbs(1,-0.06);
        r.wallECollect.setTargetPosition(13*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(-200);
        r.pickAndDrop.setPosition((PICK_DOWN+PICK_UP)/2);
        stopWatch = r.period.milliseconds() + 300;
        while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
        stopWatch = r.period.milliseconds()+600;
        while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
        r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(1500);
        stopWatch+= 600;
        while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
        r.runMeccaRC(0,0,-0.7);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.4){ }
        r.wallELift.setTargetPosition(0);
        power = maxP;
        targetY = 24. * ENCODER_PER_INCH;
        targetX = -70. * ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        hue = 140;
        while (opModeIsActive() && distanceTo > 3.* ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            hue = r.getRightHue();
            if(hue> 180 && hue < 240)r.wallELift.setTargetPosition(1500);
        }
        targetX = -86 * ENCODER_PER_INCH;
        targetY = 25 *ENCODER_PER_INCH;
        targetO = 0;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        r.wallECollect.setTargetPosition(8*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(2500);
        r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
        while (opModeIsActive() && distanceTo > 2.* ENCODER_PER_INCH) {
            distanceTo = r.moveGPS(power);
            if(distanceTo < 8.* ENCODER_PER_INCH)power-= pIncrease;
            if(power < stopMinPower)power = stopMinPower;
        }
        turnIMUAbs(2,-0.08);
        r.stopMotor();
        r.getOrientation();
        r.gps.resetOrientation(-(r.getHeading()-r.imuTare));
        targetX = r.gps.odoData[OdoGPS.X_INDEX];
        targetY = r.gps.odoData[Y_INDEX]+10.* ENCODER_PER_INCH;
        targetO = 0;
        power = startMinPower;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        int timesSeeFoundation = 0;
        while(opModeIsActive() && distanceTo > ENCODER_PER_INCH  && timesSeeFoundation <3){
            distanceTo = r.moveGPS(power);
            im.setImage(vu.getImageFromFrame());
            if(im.isAtFoundation())timesSeeFoundation++;
            else timesSeeFoundation = 0;
        }
        r.foundationMover.setPosition(FOUNDATIONMOVERLOWLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2HIGHLIMIT);
        r.stopMotor();
        stopWatch = r.period.milliseconds()+500;
        while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
        targetY = -15* ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = 0.4;
        stopWatch = r.period.milliseconds() + 1000;
        r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
        while(opModeIsActive()&& r.period.milliseconds()<stopWatch){
            distanceTo = r.moveGPS(power);
        }
        while(opModeIsActive() && !r.gps.stopped){
            distanceTo = r.moveGPS(power);
        }
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        r.stopMotor();
        r.gps.resetY(0);
        targetY = 2*ENCODER_PER_INCH;
        targetX = -28.* ENCODER_PER_INCH;
        targetO = 0;
        r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = maxP;
        stopWatch = r.period.milliseconds()+1000;
        while(opModeIsActive() && r.period.milliseconds() < stopWatch){
            distanceTo = r.moveGPS(power);
        }
        targetO = -Math.PI/4;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        hue = 140;
        while(opModeIsActive() && !(hue> 180 && hue < 240) && !r.gps.stopped){
            distanceTo = r.moveGPS(power);
            hue = r.getRightHue();
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
