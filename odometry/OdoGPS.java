package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.revExtensions.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.revExtensions.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revExtensions.RevBulkData;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */

public class OdoGPS implements Runnable{
    //Odometry wheels
    private ExpansionHubMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Thread run condition
    private boolean isRunning = true;
    public double[] odoData = new double[5];
    public static final int X_INDEX = 0, Y_INDEX = 1, O_INDEX = 2, SIN_INDEX = 3, COS_INDEX  = 4;
    public static final double ENCODER_PER_INCH = 306.66;
    public boolean stopped = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    private ExpansionHubEx myHub;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerRadianOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdoGPS(ExpansionHubMotor verticalEncoderLeft, ExpansionHubMotor verticalEncoderRight, ExpansionHubMotor horizontalEncoder, int threadSleepDelay, ExpansionHubEx hub){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        myHub = hub;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * ENCODER_PER_INCH;
        this.horizontalEncoderTickPerRadianOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        RevBulkData dataPack = myHub.getBulkInputData();
        verticalLeftEncoderWheelPosition = dataPack.getMotorCurrentPosition(verticalEncoderLeft);
        verticalRightEncoderWheelPosition = dataPack.getMotorCurrentPosition(verticalEncoderRight);
        normalEncoderWheelPosition = dataPack.getMotorCurrentPosition(horizontalEncoder);
        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        odoData[X_INDEX] = 0.;
        odoData[Y_INDEX] = 0.;
        odoData[O_INDEX] = 0.;
        odoData[SIN_INDEX] = 0.;
        odoData[COS_INDEX] = 0.;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        RevBulkData dataPack = myHub.getBulkInputData();
        verticalLeftEncoderWheelPosition = dataPack.getMotorCurrentPosition(verticalEncoderLeft);
        verticalRightEncoderWheelPosition = dataPack.getMotorCurrentPosition(verticalEncoderRight);
        //extraEncoder = dataPack.getMotorCurrentPosition(fourthMotor);
        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange)/(robotEncoderWheelDistance);
        odoData[O_INDEX] += changeInRobotOrientation;
        double sin = Math.sin(odoData[O_INDEX]);
        double cos = Math.cos(odoData[O_INDEX]);
        //Get the components of the motion
        normalEncoderWheelPosition = dataPack.getMotorCurrentPosition(horizontalEncoder);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerRadianOffset);
        stopped = (Math.abs(leftChange)+Math.abs(rightChange)+Math.abs(rawHorizontalChange)) < 30;
        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;
        //Calculate and update the position values
        odoData[X_INDEX] += p*sin + n*cos;
        odoData[Y_INDEX] += p*cos - n*sin;
        odoData[SIN_INDEX] = sin;
        odoData[COS_INDEX] = cos;

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */

    public void resetOrientation(double newAngle){
        odoData[O_INDEX]=newAngle;
    }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public void resetY(double newY){ odoData[Y_INDEX] = 0; }
    public void resetX(double newX) { odoData[X_INDEX] = 0; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

