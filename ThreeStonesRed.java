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
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_BLUE_BACK;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.SERVO_PER_RADIAN;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

//Start on blue side

 @Autonomous(name="Red 3 Stones", group="Red Linear Opmode")
 //@Disabled
 public class ThreeStonesRed extends LinearOpMode {

     // Declare OpMode members.
     private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
     private static final double maxP =0.93;
     private static final double pIncrease = 0.015;
     private static final double startMinPower = 0.25;
     private static final double stopMinPower = 0.1;
     double power = startMinPower;
     double targetY = ENCODER_PER_INCH;
     double targetX = -11* ENCODER_PER_INCH;
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
             else if (index == 6)skyOrder = im.findStonesRed();
             else if (index ==8) r.wallECollect.setPower(.5);
             else if (index ==10){
                 if(skyOrder == ImageHelper.SKY_LEFT)rotateArm = ROTATE_BLUE_BACK-0.4*SERVO_PER_RADIAN;
                 r.rotateArm.setPosition(rotateArm);
                 r.pickAndDrop.setPosition(HardwareWallEbot.PICK_UP);
             }
         }
         targetY = 21 * ENCODER_PER_INCH;
         if(skyOrder == ImageHelper.SKY_RIGHT){
             targetX = -11.5 * ENCODER_PER_INCH;
         }else if (skyOrder == ImageHelper.SKY_CENTER) {
             targetX = -18.5 * ENCODER_PER_INCH;
         }else{
             targetX = -19.5 * ENCODER_PER_INCH;
             targetO = -0.4;
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
         r.runMeccaRC(0,0,0.7);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.4);
         r.wallELift.setTargetPosition(0);
         targetY = 23. * ENCODER_PER_INCH;
         targetO = Math.PI/2;
         targetX = 52. * ENCODER_PER_INCH;
         power = maxP;
         distanceTo = r.prepareMove(targetX, targetY,targetO);
         int hue = 100;
         while (opModeIsActive() && !((hue > 0 && hue < 50)||(hue > 310 && hue < 360))){
             distanceTo = r.moveGPS(power);
             hue = r.getRightHue();
         }
         r.wallELift.setTargetPosition(1700);
         r.wallECollect.setTargetPosition(18*COLLECT_ENCODER_PER_INCH);
         targetX = r.gps.odoData[X_INDEX]+20 * ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         while(opModeIsActive() && distanceTo > 1.5*ENCODER_PER_INCH){
              distanceTo = r.moveGPS(power);
              if(distanceTo < 8*ENCODER_PER_INCH)power -= pIncrease;
              if(power < stopMinPower) power = stopMinPower;
         }
         targetY = r.gps.odoData[Y_INDEX]-20*ENCODER_PER_INCH;
         targetX = r.gps.odoData[X_INDEX];
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         power = 0.5;
         im.setImage(vu.getImageFromFrame());
         while(opModeIsActive() && !im.centerFoundationRed()){
             r.moveGPS(power);
             im.setImage(vu.getImageFromFrame());
         }
        // targetX = r.gps.odoData[X_INDEX];
        // targetY = r.gps.odoData[Y_INDEX] - 4*ENCODER_PER_INCH;
        // distanceTo = r.prepareMove(targetX,targetY,targetO);
        // while(opModeIsActive() && distanceTo > 2*ENCODER_PER_INCH){
        //     distanceTo = r.moveGPS(power);
        // }
         targetY = r.gps.odoData[Y_INDEX];
         targetX = 72*ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         int timesSeeFoundation = 0;
         while(opModeIsActive() && distanceTo > ENCODER_PER_INCH  && timesSeeFoundation <3){
             distanceTo = r.moveGPS(power);
             im.setImage(vu.getImageFromFrame());
             if(im.isAtFoundation())timesSeeFoundation++;
             else timesSeeFoundation = 0;
         }
         r.pickAndDrop.setPosition(PICK_UP);
         r.stopMotor();
         double firstTime = r.period.seconds();
         double foundationX = r.gps.odoData[X_INDEX];
         double foundationY = r.gps.odoData[Y_INDEX];
         r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         targetY = 20.* ENCODER_PER_INCH;
         targetX = 44. * ENCODER_PER_INCH;
         targetO = Math.PI/2;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         power = startMinPower;
         while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH){
             distanceTo = r.moveGPS(power);
             power += pIncrease;
             if(power > maxP)power = maxP;
         }
         if(skyOrder == ImageHelper.SKY_RIGHT)targetX = 17* ENCODER_PER_INCH;
         else if (skyOrder == ImageHelper.SKY_CENTER) targetX = 9 * ENCODER_PER_INCH;
         else targetX = 0*ENCODER_PER_INCH;
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
         turnIMUAbs(1,0.06);
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
         r.runMeccaRC(0,0,0.7);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.4){ }
         r.wallELift.setTargetPosition(0);
         targetX = foundationX+ENCODER_PER_INCH;
         targetY = foundationY;
         targetO = Math.PI/2;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         power = 0.5;
         while(opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH){
             distanceTo = r.moveGPS(power);
             if(distanceTo < 8*ENCODER_PER_INCH)power -= pIncrease;
             else power += pIncrease;
             if(power > maxP) power = maxP;
             else if (power < stopMinPower) power = stopMinPower;
             hue = r.getLeftHue();
             if((hue > 0 && hue < 50)||(hue > 310 && hue < 360)){
                 r.wallECollect.setTargetPosition(16*COLLECT_ENCODER_PER_INCH);
                 r.wallELift.setTargetPosition(1700);
             }
         }
         r.pickAndDrop.setPosition(PICK_UP);
         r.stopMotor();
         double secondTime = r.period.seconds();
         r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         targetY = 20.* ENCODER_PER_INCH;
         targetX = 44. * ENCODER_PER_INCH;
         targetO = Math.PI/2;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         power = startMinPower;
         while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH){
             distanceTo = r.moveGPS(power);
             power += pIncrease;
             if(power > maxP)power = maxP;
         }
         if(skyOrder == ImageHelper.SKY_RIGHT)targetX = 9* ENCODER_PER_INCH;
         else targetX = 17 * ENCODER_PER_INCH;
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
         turnIMUAbs(1,0.06);
         r.wallECollect.setTargetPosition(13*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(-200);
         r.pickAndDrop.setPosition((PICK_DOWN+PICK_UP)/2);
         stopWatch = r.period.milliseconds() + 300;
         while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
         r.pickAndDrop.setPosition(HardwareWallEbot.PICK_DOWN);
         stopWatch = r.period.milliseconds()+600;
         while(opModeIsActive() && r.period.milliseconds()<stopWatch){}
         r.wallECollect.setTargetPosition(6*HardwareWallEbot.COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(1000);
         stopWatch+= 100;
         while(opModeIsActive() && r.period.milliseconds() < stopWatch){}
         r.runMeccaRC(0,0,0.7);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.4){ }
         r.wallELift.setTargetPosition(0);
         targetX = foundationX+ENCODER_PER_INCH;
         targetY = foundationY;
         targetO = Math.PI/2;
         distanceTo = r.prepareMove(targetX,targetY,targetO);
         power = 0.5;
         while(opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH){
             distanceTo = r.moveGPS(power);
             if(distanceTo < 8*ENCODER_PER_INCH)power -= pIncrease;
             else power += pIncrease;
             if(power > maxP) power = maxP;
             else if (power < stopMinPower) power = stopMinPower;
             hue = r.getLeftHue();
             if((hue > 0 && hue < 50)||(hue > 310 && hue < 360)){
                 r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
                 r.wallELift.setTargetPosition(1700);
             }
         }
         r.pickAndDrop.setPosition(PICK_UP);
         r.stopMotor();
         double thirdTime = r.period.seconds();
         targetY=r.gps.odoData[Y_INDEX]+ENCODER_PER_INCH;
         targetX = 22*ENCODER_PER_INCH;
         r.prepareMove(targetX,targetY,targetO);
         power = maxP;
         hue =0;
         r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         while(opModeIsActive() && ! (hue > 0 && hue < 50)||(hue > 310 && hue < 360)){
             r.moveGPS(power);
             hue = r.getLeftHue();
         }
         r.stopMotor();
         stopWatch = r.period.seconds();
         while(opModeIsActive() && !gamepad1.a){
             telemetry.addData("first in ", firstTime);
             telemetry.addData("second in ", secondTime);
             telemetry.addData("third in ", thirdTime);
             telemetry.addData("finished in ", stopWatch);
             telemetry.update();
         }
         r.gps.stop();
         vu.vuDeActivate();
     }

     public void turnIMUAbs(double speed, double targetAngle){
         double stopWatch = r.period.seconds()+3;
         while (opModeIsActive() && (!r.onHeading(targetAngle,speed)) && (r.period.seconds() < stopWatch)) {}
     }
 }
