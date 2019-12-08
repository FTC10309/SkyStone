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

import static org.firstinspires.ftc.teamcode.HardwareWallEbot.AUTO_MIN_POWER;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.COLLECT_ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MAX_P;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.P_INCREASE;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.STOP_P;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

//Start on red side
@Autonomous(name="Three stones self red", group="Red Linear Opmode")
//@Disabled
public class ThreeStonesSelfRed extends LinearOpMode {

    // Declare OpMode members.
    private SkyStoneJediBot r = new SkyStoneJediBot();
    double power = AUTO_MIN_POWER;
    double targetY = 31*ENCODER_PER_INCH;
    double targetX = 32.5* ENCODER_PER_INCH;
    double targetO = Math.PI/2;

    @Override
    public void runOpMode() throws InterruptedException {
        VuHelper vu = new VuHelper(hardwareMap);
        vu.vuActivate();
        r.init(hardwareMap);
        r.setOP(this,vu,false);
        int skyOrder = r.getFirstStoneGP();
        r.runMeccaRC(0,0,0.8);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.44){ }
        r.wallELift.setTargetPosition(-100);
        double distanceTo = r.prepareMove(targetX,targetY,targetO);
        double hue = 140;
        while (opModeIsActive() && distanceTo > 3.* ENCODER_PER_INCH){
            distanceTo = r.moveGPS(MAX_P);
            hue = r.getLeftHue();
            if((hue > 0 && hue < 50)
                    || (hue > 310 && hue < 360))r.wallELift.setTargetPosition(2500);
        }
        targetX = 47.5 * ENCODER_PER_INCH;
        targetY = 30 *ENCODER_PER_INCH;
        targetO = 0;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        r.wallECollect.setTargetPosition(8*COLLECT_ENCODER_PER_INCH);
        r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
        power = MAX_P;
        while (opModeIsActive() && distanceTo > 2.* ENCODER_PER_INCH) {
            distanceTo = r.moveGPS(power);
            if(distanceTo < 5.* ENCODER_PER_INCH && power > STOP_P)power-= P_INCREASE;
        }
        r.turnIMUAbs(2,0);
        r.stopMotor();
        r.hookFoundation();
        targetY = 24*ENCODER_PER_INCH;
        targetX = 40.* ENCODER_PER_INCH;
        targetO = Math.PI/2;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = 0.5;
        r.moveGPS(power);
        r.pickAndDrop.setPosition(PICK_UP);
        while(opModeIsActive() && distanceTo > 2*ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(power < MAX_P)power += P_INCREASE;
        }
        r.runMeccaRC(0,0,0.8);
        while(opModeIsActive() && r.gps.odoData[O_INDEX]<Math.PI/2-0.2){}
        r.stopMotor();
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        double foundationX = r.gps.odoData[X_INDEX];
        double foundationY = r.gps.odoData[Y_INDEX];
        r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        targetY = 29* ENCODER_PER_INCH;
        targetX = (44.-38.5) * ENCODER_PER_INCH;
        targetO = Math.PI/2;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH){
            distanceTo = r.moveGPS(MAX_P);
        }
        if(skyOrder == ImageHelper.SKY_OUTSIDE){
            r.pickOutsideStone(1,0);
            r.pickOutSideStoneSecondPart();
        }else{
            r.pick2ndStone();
        }
        r.runMeccaRC(0,0,0.8);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.42){ }
        r.wallELift.setTargetPosition(-100);
        targetX = foundationX+2*ENCODER_PER_INCH;
        targetY = foundationY - 2*ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        hue = 100;
        while (opModeIsActive() && !((hue > 0 && hue < 50)||(hue > 310 && hue < 360))){
            r.moveGPS(MAX_P);
            hue = r.getRightHue();
        }
        r.wallECollect.setTargetPosition(12*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(1700);
        distanceTo = r.prepareMove(targetX,targetY,targetO + 0.1);
        power = MAX_P;
        while(opModeIsActive() && distanceTo > 3*ENCODER_PER_INCH && !r.gps.stopped){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 6*ENCODER_PER_INCH && power > STOP_P)power -= P_INCREASE;
        }
        r.pickAndDrop.setPosition(PICK_UP);
        r.stopMotor();
        r.wallECollect.setTargetPosition(5*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        targetY = 29* ENCODER_PER_INCH;
        targetX = (44.-38.5) * ENCODER_PER_INCH;
        targetO = Math.PI/2;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while (opModeIsActive() && distanceTo > 6.* ENCODER_PER_INCH) distanceTo = r.moveGPS(MAX_P);
        r.pickOutsideStone(2,0);
        r.pickOutSideStoneSecondPart();
        double thirdTime = r.period.seconds();
        boolean goForIt = thirdTime < 25.75;
        r.runMeccaRC(0,0,0.8);
        while (opModeIsActive() && r.gps.odoData[OdoGPS.O_INDEX] < Math.PI/2-0.42){ }
        targetX = foundationX + 2 * ENCODER_PER_INCH;
        targetY = foundationY - 2 * ENCODER_PER_INCH;
        r.prepareMove(targetX,targetY,Math.PI/2);
        r.moveGPS(MAX_P);
        r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
        hue = 100;
        while (opModeIsActive() && !((hue > 0 && hue < 50)||(hue > 310 && hue < 360))){
            r.moveGPS(MAX_P);
            hue = r.getRightHue();
        }
        if(goForIt) {
            r.wallECollect.setTargetPosition(12 * COLLECT_ENCODER_PER_INCH);
            r.wallELift.setTargetPosition(1700);
            distanceTo = r.prepareMove(targetX, targetY, Math.PI / 2 + 0.18);
            power = MAX_P;
            while (opModeIsActive() && distanceTo > 3 * ENCODER_PER_INCH && !r.gps.stopped) {
                distanceTo = r.moveGPS(power);
                if (distanceTo < 6 * ENCODER_PER_INCH && power > STOP_P) power -= P_INCREASE;
            }
            r.pickAndDrop.setPosition(PICK_UP);
        }else sleep(200);
        targetY=r.gps.odoData[Y_INDEX]+ENCODER_PER_INCH;
        targetX = (22-38.5)*ENCODER_PER_INCH;
        r.prepareMove(targetX,targetY,Math.PI/2);
        r.moveGPS(MAX_P);
        hue =0;
        r.wallECollect.setTargetPosition(6*COLLECT_ENCODER_PER_INCH);
        r.wallELift.setTargetPosition(0);
        while(opModeIsActive() && !((hue > 0 && hue < 50)||(hue > 310 && hue < 360))){
            r.moveGPS(MAX_P);
            hue = r.getLeftHue();
        }
        r.stopMotor();
        while(opModeIsActive() && r.period.seconds() < 29.4){}
        r.writeRobot();
        double stopWatch = r.period.seconds();
        while(opModeIsActive() && !gamepad1.a){
            telemetry.addData("third in ", thirdTime);
            telemetry.addData("finished in ", stopWatch);
            telemetry.update();
        }
        r.gps.stop();
        vu.vuDeActivate();
    }
}
