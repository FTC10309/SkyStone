 package org.firstinspires.ftc.teamcode;

/**
 * Created by qiu29 on 9/27/2018.
 */

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

 /**
 * Opmode for TeleOp operation
 * The following are the full list of controls available to driver (lift is front of the robot)
 *  */

@TeleOp(name="TeleOp Quarry Red", group="Red Linear Opmode")
//@Disabled
public class TeleopWallERedQuarry extends LinearOpMode {

    // Declare OpMode members.
    private SkyStoneJediBotTeleop r = new SkyStoneJediBotTeleop();
    @Override
    public void runOpMode() {
        r.period.reset();
        r.init(hardwareMap);
        r.setOP(this,null,false);
        r.initOpmode();
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Have been waiting ", r.period.seconds());
            telemetry.addData("X Position", r.gps.odoData[X_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Y Position", r.gps.odoData[Y_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Orientation (Radian)", r.gps.odoData[O_INDEX]);
            telemetry.addData( "Number stones already pickes",r.nextStone);
            telemetry.addData("skyOrder is ", r.skyOrder);
            telemetry.update();
        }
        r.period.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r.getSensors();
            r.moveWallEServo();
            r.moveSlide();
            r.checkDriveTrain();
            r.pickUpStone();
            r.quarrySetState();
            r.quarryReturnState();
            r.flipStone();
            r.moveCapstone();
            telemetry.addData("X Position", r.gps.odoData[X_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Y Position", r.gps.odoData[Y_INDEX] / ENCODER_PER_INCH);
            telemetry.addData("Orientation (Radian)", r.gps.odoData[O_INDEX]);
            telemetry.addData("Lift encoder", r.liftPos);
            telemetry.addData("Collect encoder", r.collectPos);
            telemetry.update();
        }
    }
}