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

import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;

//This OpMode finishes autonomous mode actions when latched on lander facing crater.

@Autonomous(name="Move to line", group="Neutral Linear Opmode")
//@Disabled
public class MoveToLine extends LinearOpMode {

    // Declare OpMode members.
    private WallEGPSMeccaBot r = new WallEGPSMeccaBot();
    private static final double maxP =0.7;
    private static final double pIncrease = 0.01;
    private static final double minPower = 0.25;
    double power = minPower;
    double targetY, targetX, targetO, distanceTo;
    @Override

    public void runOpMode() throws InterruptedException {
        boolean toRight = true;
        boolean isRed = true;
        double increment = 0.0002;
        double yTarget = 0;
        double delay = 0;
        boolean backParking = true;
        r.init(hardwareMap);
        r.startGPS(50);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Use dpad right or left to select direction");
            telemetry.addLine("Use dpad up and down to change delay");
            telemetry.addLine("Use y and a to change distance to move first");
            telemetry.addLine("Use x and b to change red or blue side");
            telemetry.addLine("Use left bumper for parking back, right bumper for forward");
            if (gamepad1.dpad_right) toRight = true;
            else if (gamepad1.dpad_left) toRight = false;
            if (gamepad1.x) isRed = false;
            else if (gamepad1.b) isRed = true;
            if(isRed)telemetry.addLine("robot is on red side");
            else telemetry.addLine("robot is on blue side");
            if (toRight) telemetry.addLine("moving to right");
            else telemetry.addLine("moving to left.");
            if (gamepad1.dpad_up && delay <= 22) delay += increment;
            else if (gamepad1.dpad_down && delay >= 0) delay -= increment;
            if (gamepad1.y && yTarget < 54) yTarget += increment;
            else if(gamepad1.a && yTarget > 0) yTarget -= increment;
            if(gamepad1.left_bumper)backParking = true;
            else if(gamepad1.right_bumper) backParking = false;
            telemetry.addData("Parking in the back ",backParking);
            String tempStr = "delay for " + delay + " seconds.";
            telemetry.addLine(tempStr);
            tempStr = "First move up "+ yTarget+ " inches";
            telemetry.addLine(tempStr);
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.update();
        }
        r.period.reset();
        r.setBreakMode();
        r.resetGPS(0,0,0);

        targetY = yTarget * ENCODER_PER_INCH;
        targetX = 0.;
        targetO = 0.;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = minPower;
        while (opModeIsActive() && distanceTo > ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 4. * ENCODER_PER_INCH)power -= pIncrease;
            else power += pIncrease;
            if(power > maxP) power = maxP;
            if(power < minPower) power = minPower;
        }
        r.stopMotor();
        while(opModeIsActive() && r.period.seconds() < delay){}
        if(backParking)targetY = ENCODER_PER_INCH;
        else targetY = 24.* ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = minPower;
        while (opModeIsActive() && distanceTo > ENCODER_PER_INCH){
            distanceTo = r.moveGPS(power);
            if(distanceTo < 4.* ENCODER_PER_INCH)power -= pIncrease;
            else power += pIncrease;
            if(power > maxP) power = maxP;
            if(power < minPower) power = minPower;
        }
        int hue = 100;
        if(toRight)targetX = 70.* ENCODER_PER_INCH;
        else targetX = -70.* ENCODER_PER_INCH;
        targetY = r.gps.odoData[OdoGPS.Y_INDEX] + ENCODER_PER_INCH;
        distanceTo = r.prepareMove(targetX,targetY,targetO);
        power = minPower;
        boolean notOnTape = true;
        while(opModeIsActive() && notOnTape ){
            distanceTo = r.moveGPS(power);
            if(toRight) hue = r.getRightHue();
            else hue = r.getLeftHue();
            if(isRed) notOnTape = !(hue > 0 && hue < 50) && !(hue > 310 && hue < 360);
            else notOnTape = !(hue> 180 && hue < 240);
            power += pIncrease;
            if(power > maxP) power = maxP;
        }
        r.stopMotor();
        while(opModeIsActive()) {
            telemetry.addData("left hue", r.getLeftHue());
            telemetry.addData("right hue", r.getRightHue());
            telemetry.update();
        }
        r.stopGPS();
    }
}

