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

import org.firstinspires.ftc.teamcode.odometry.OdoGPS;
import org.firstinspires.ftc.teamcode.vision.ImageHelper;
import org.firstinspires.ftc.teamcode.vision.VuHelper;

import static org.firstinspires.ftc.teamcode.HardwareWallEbot.COLLECT_ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MAX_P;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.P_INCREASE;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.STOP_P;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

//Start on blue side

 @Autonomous(name="Blue 4 Stones", group="Blue Linear Opmode")
 //@Disabled
 public class ThreeStonesBlue extends LinearOpMode {

     // Declare OpMode members.
     private SkyStoneJediBot r = new SkyStoneJediBot();
     double power = MAX_P;
     double targetY = 33*ENCODER_PER_INCH;
     double targetX = -(52-38.5)* ENCODER_PER_INCH;

     @Override
     public void runOpMode() throws InterruptedException {
         VuHelper vu = new VuHelper(hardwareMap);
         vu.vuActivate();
         r.init(hardwareMap);
         r.setOP(this,vu,true);
         int skyOrder = r.getFirstStoneGP();
         r.runMeccaRC(0,0,-0.7);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.4);
         r.wallELift.setTargetPosition(-100);
         r.prepareMove(targetX, targetY,-Math.PI/2);
         int hue = 0;
         while (opModeIsActive() && !(hue> 180 && hue < 240)){
             r.moveGPS(MAX_P);
             hue = r.getRightHue();
         }
         r.wallELift.setTargetPosition(1900);
         r.wallECollect.setTargetPosition(18*COLLECT_ENCODER_PER_INCH);
         targetX = r.gps.odoData[X_INDEX]-20 * ENCODER_PER_INCH;
         double distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         while(opModeIsActive() && distanceTo > ENCODER_PER_INCH){
              distanceTo = r.moveGPS(power);
              if(distanceTo < 7*ENCODER_PER_INCH)power -= P_INCREASE;
              if(power < STOP_P) power = STOP_P;
         }
         targetY = r.gps.odoData[Y_INDEX]-20*ENCODER_PER_INCH;
         targetX = r.gps.odoData[X_INDEX];
         r.prepareMove(targetX,targetY,-Math.PI/2);
         power = 0.5;
         r.im.setImage(vu.getImageFromFrame());
         while(opModeIsActive() && !r.im.centerFoundation()&& !r.gps.stopped){
             r.moveGPS(power);
             r.im.setImage(vu.getImageFromFrame());
         }
         targetX = r.gps.odoData[X_INDEX];
         targetY = r.gps.odoData[Y_INDEX] - 5*ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         while(opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH)distanceTo = r.moveGPS(power);
         targetY = r.gps.odoData[Y_INDEX];
         targetX = -(72-38.5)*ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         int timesSeeFoundation = 0;
         while(opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH  && timesSeeFoundation <3 && !r.gps.stopped){
             distanceTo = r.moveGPS(MAX_P);
             r.im.setImage(vu.getImageFromFrame());
             if(r.im.isAtFoundation())timesSeeFoundation++;
             else timesSeeFoundation = 0;
         }
         r.pickAndDrop.setPosition(PICK_UP);
         double foundationX = r.gps.odoData[X_INDEX];
         double foundationY = r.gps.odoData[Y_INDEX];
         targetY = 30.* ENCODER_PER_INCH;
         targetX = -(44.-38.5) * ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         r.moveGPS(MAX_P);
         r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH)distanceTo = r.moveGPS(MAX_P);
         if(skyOrder == ImageHelper.SKY_OUTSIDE){
             r.pickOutsideStone(1,0);
             r.pickOutSideStoneSecondPart();
         }else r.pick2ndStone();
         r.runMeccaRC(0,0,-0.8);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.42){ }
         r.wallELift.setTargetPosition(-100);
         targetX = foundationX-ENCODER_PER_INCH;
         targetY = foundationY+ENCODER_PER_INCH;
         r.prepareMove(targetX,targetY,-Math.PI/2);
         power = MAX_P;
         while(opModeIsActive() && r.gps.odoData[X_INDEX] > targetX+2*ENCODER_PER_INCH && !r.gps.stopped){
             distanceTo = r.moveGPS(power);
             if(distanceTo < 3*ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
             hue = r.getLeftHue();
             if(hue> 180 && hue < 240){
                 r.wallECollect.setTargetPosition(18*COLLECT_ENCODER_PER_INCH);
                 r.wallELift.setTargetPosition(1700);
             }
         }
         r.pickAndDrop.setPosition(PICK_UP);
         targetY = 30.* ENCODER_PER_INCH;
         targetX = -(44-38.5) * ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         r.moveGPS(MAX_P);
         r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH)distanceTo = r.moveGPS(MAX_P);
         r.pickOutsideStone(2,0);
         r.pickOutSideStoneSecondPart();
         r.runMeccaRC(0,0,-0.8);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.42){ }
         targetX = foundationX-ENCODER_PER_INCH;
         targetY = foundationY+2*ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         power = MAX_P;
         while(opModeIsActive() && r.gps.odoData[X_INDEX] > targetX+3*ENCODER_PER_INCH && !r.gps.stopped){
             distanceTo = r.moveGPS(power);
             if(distanceTo < 3*ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
             hue = r.getLeftHue();
             if(hue> 180 && hue < 240){
                 r.wallECollect.setTargetPosition(12*COLLECT_ENCODER_PER_INCH);
                 r.wallELift.setTargetPosition(2000);
             }
         }
         r.pickAndDrop.setPosition(PICK_UP);
         double firstDecision = r.period.seconds();
         targetY = 30.* ENCODER_PER_INCH;
         targetX = -(44-38.5) * ENCODER_PER_INCH;
         distanceTo = r.prepareMove(targetX,targetY,-Math.PI/2);
         r.moveGPS(MAX_P);
         r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH)distanceTo = r.moveGPS(MAX_P);
         r.pickOutsideStone(3,0);
         r.pickOutSideStoneSecondPart();
         double decisionTime = r.period.seconds();
         boolean goForIt = decisionTime < 25.75;
         r.runMeccaRC(0,0,-0.8);
         while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] > -Math.PI/2+0.42){ }
         targetX = foundationX-ENCODER_PER_INCH;
         targetY = foundationY+2*ENCODER_PER_INCH;
         r.prepareMove(targetX,targetY,-Math.PI/2);
         r.moveGPS(MAX_P);
         r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
         hue = 100;
         while (opModeIsActive() && !(hue> 180 && hue < 240)){
             r.moveGPS(MAX_P);
             hue = r.getRightHue();
         }
         if(goForIt) {
             r.wallECollect.setTargetPosition(10 * COLLECT_ENCODER_PER_INCH);
             r.wallELift.setTargetPosition(2000);
             r.prepareMove(targetX, targetY, -Math.PI/2);
             power = MAX_P;
             while (opModeIsActive() && r.gps.odoData[X_INDEX] > targetX+5*ENCODER_PER_INCH && !r.gps.stopped) {
                 distanceTo = r.moveGPS(power);
                 if (distanceTo < 6 * ENCODER_PER_INCH && power > STOP_P) power -= P_INCREASE;
             }
             r.pickAndDrop.setPosition(PICK_UP);
         }else sleep(200);
         targetY=r.gps.odoData[Y_INDEX]+ENCODER_PER_INCH;
         targetX = -(22-38.5)*ENCODER_PER_INCH;
         r.prepareMove(targetX,targetY,-Math.PI/2);
         power = MAX_P;
         r.moveGPS(power);
         hue =0;
         r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
         r.wallELift.setTargetPosition(0);
         while(opModeIsActive() && !(hue> 180 && hue < 240)){
             r.moveGPS(power);
             hue = r.getLeftHue();
         }
         r.stopMotor();
         while(opModeIsActive() && r.period.seconds() < 29.4){}
         r.writeRobot();
         double stopWatch = r.period.seconds();
         while(opModeIsActive() && !gamepad1.a){
             telemetry.addData("first decision ", firstDecision);
             telemetry.addData("decision time ", decisionTime);
             telemetry.addData("finished in ", stopWatch);
             telemetry.update();
         }
         r.gps.stop();
         vu.vuDeActivate();
     }
 }

