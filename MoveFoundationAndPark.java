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

import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2DOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2HIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVER2LOWLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERDOWN;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERHIGHLIMIT;
import static org.firstinspires.ftc.teamcode.HardwareWallEbot.FOUNDATIONMOVERLOWLIMIT;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;


//Start on blue side

@Autonomous(name="Move foundation and park", group="Neutral Linear Opmode")
//@Disabled
public class MoveFoundationAndPark extends LinearOpMode {

    // Declare OpMode members.
    private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
    private static final double maxP =0.7;
    private static final double pIncrease = 0.01;
    private static final double minPower = 0.2;
    double power = minPower;
    double targetY = 0.;
    double targetX = 0.;
    double targetO = 0.;
    double distanceTo;

    @Override
    public void runOpMode() throws InterruptedException {
        double increment = 0.0002;
        double yTarget = 0;
        double delay = 0;
        boolean isRed = true;
        VuHelper vu = new VuHelper(hardwareMap);
        ImageHelper im = new ImageHelper();
        vu.vuActivate();
        r.init(hardwareMap);
        r.startGPS(50);
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()&& !isStopRequested()) {
            if (gamepad1.dpad_up && delay <= 15) delay += increment;
            else if (gamepad1.dpad_down && delay >= 0) delay -= increment;
            if (gamepad1.y) yTarget = 23;
            else if(gamepad1.a) yTarget = 0;
            if (gamepad1.x) isRed = false;
            else if (gamepad1.b) isRed = true;
            telemetry.addLine("Use dpad up and down to change delay");
            telemetry.addLine("Use y or a button to change final move");
            telemetry.addLine("Use x and b to change red or blue side");
            if(isRed)telemetry.addLine("robot is on red side");
            else telemetry.addLine("robot is on blue side");
            String tempStr = "delay for " + delay + " seconds.";
            telemetry.addLine(tempStr);
            tempStr = "Final move up "+ yTarget+ " inches";
            telemetry.addLine(tempStr);
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.update();
        }

        r.tareIMU();
        r.setBreakMode();
        r.period.reset();
        r.resetGPS(0,0,0);

        while (opModeIsActive() && r.period.seconds() < delay) { }
        if(isRed)targetX = 5.*ENCODER_PER_INCH;
        else targetX = -5.*ENCODER_PER_INCH;
        targetY = 0.1 * ENCODER_PER_INCH;
        targetO = 0.;
        distanceTo = r.prepareMove(targetX, targetY,targetO);
        power = 0.3;
        while(opModeIsActive() && distanceTo > ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
        }
        r.stopMotor();
        r.foundationMover.setPosition(FOUNDATIONMOVERDOWN);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2DOWN);
        targetY = 32.*ENCODER_PER_INCH;
        targetX = r.gps.odoData[X_INDEX];
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = minPower;
        int timesSeeFoundation = 0;
        while(opModeIsActive() && distanceTo > ENCODER_PER_INCH  && timesSeeFoundation <3){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 10)power-=pIncrease;
            else power+=pIncrease;
            if(power > maxP) power = maxP;
            else if(power < minPower) power = minPower;
            im.setImage(vu.getImageFromFrame());
            if(im.isAtFoundation())timesSeeFoundation++;
            else timesSeeFoundation = 0;
        }
        r.foundationMover.setPosition(FOUNDATIONMOVERLOWLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2HIGHLIMIT);
        r.stopMotor();
        double stopTime = r.period.seconds() + 0.5;
        while (opModeIsActive() && r.period.seconds() < stopTime){ }
        r.runMeccaRC(-0.35, 0, 0);
        stopTime = r.period.seconds() + 1;
        while (opModeIsActive() && r.period.seconds() < stopTime){ }
        targetY = -2*ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        while(opModeIsActive() && !r.gps.stopped){}
        //double oldDistanceTo = distanceTo - OdoGPS.ENCODER_PER_INCH;
        //while(opModeIsActive() && Math.abs(distanceTo - oldDistanceTo) > 5){
        //    oldDistanceTo = distanceTo;
        //    sleep(60);
        //    distanceTo = r.moveGPS(power);
        //}
        r.stopMotor();
        r.foundationMover.setPosition(FOUNDATIONMOVERHIGHLIMIT);
        r.foundationMover2.setPosition(FOUNDATIONMOVER2LOWLIMIT);
        stopTime = r.period.milliseconds()+1000;
        while(opModeIsActive() && r.period.milliseconds() < stopTime){}
        r.gps.resetY(0);
        targetY = 0.5*ENCODER_PER_INCH;
        r.runMecca(0.2,0,0);
        while(opModeIsActive() && r.gps.odoData[Y_INDEX] < targetY){}
        r.stopMotor();
        r.runMecca(-0.2,0,0);
        stopTime = r.period.milliseconds()+1000;
        while(opModeIsActive() && r.period.milliseconds() < stopTime){}
        r.stopMotor();
        targetY = ENCODER_PER_INCH;
        if(isRed)targetX = -28*ENCODER_PER_INCH;
        else targetX = 28*ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        boolean notOnTape = true;
        power = minPower;
        while(opModeIsActive() && notOnTape ){
            distanceTo = r.moveGPS(power);
            if(isRed) {
                int hue = r.getLeftHue();
                notOnTape = !(hue > 0 && hue < 50) && !(hue > 310 && hue < 360);
            } else {
                int hue = r.getRightHue();
                notOnTape = !(hue > 180 && hue < 240);
            }
            power += pIncrease;
            if(power > maxP) power = maxP;
        }
        r.stopMotor();

        targetY = yTarget*ENCODER_PER_INCH;
        targetX = r.gps.odoData[X_INDEX];
        distanceTo = r.prepareMove(targetX,targetY,0);
        power = minPower;
        while(opModeIsActive() && distanceTo < ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            power += pIncrease;
            if(power > maxP) power = maxP;
        }
        r.stopMotor();
        r.stopGPS();
        vu.vuDeActivate();
    }
}
