 package org.firstinspires.ftc.teamcode;

/**
 * Created by qiu29 on 9/27/2018.
 */

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ReadWriteFile;

 import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
 import org.firstinspires.ftc.teamcode.revExtensions.RevBulkData;

 import java.io.File;

 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.AUTO_MIN_POWER;
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
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MAX_P;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.MIN_LIFT_ENCODER;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.PICK_UP;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.P_INCREASE;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_BLUE_BACK;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.ROTATE_NEUTRAL;
 import static org.firstinspires.ftc.teamcode.HardwareWallEbot.STOP_P;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.ENCODER_PER_INCH;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.O_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.X_INDEX;
 import static org.firstinspires.ftc.teamcode.odometry.OdoGPS.Y_INDEX;

 /**
 * Opmode for TeleOp operation
 * The following are the full list of controls available to driver (lift is front of the robot)
 *
 *  */

@TeleOp(name="TeleOp Build Blue", group="Blue Linear Opmode")
//@Disabled
public class TeleopWallEBlueBuild extends LinearOpMode {

    // Declare OpMode members.
    private SkyStoneJediBotTeleop r = new SkyStoneJediBotTeleop();
    @Override
    public void runOpMode() {
        r.period.reset();
        r.init(hardwareMap);
        r.setOP(this,null,true);
        r.setBuild();
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
            r.buildSetState();
            r.buildReturnState();
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