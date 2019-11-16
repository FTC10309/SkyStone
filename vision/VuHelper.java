package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Bobo Qiu on 10/6/2018.
 * This is a class to take care of Vuforia stuff.  Currently we are not using targeting information
 * Only need to get a frame out of Vuforia for openCV
 */

public class VuHelper {

    private static final String VUFORIA_KEY = "ARFj043/////AAABmcqs6O58EExiiz/iuDeILhAQwUwWb+w6iE4+SIoDqF/OGwkdGX9gwb35jCHrnLpvbmFTlNF0+t3sOaKORIOU7jaLYcuFxm50fUCtvubLKL+9MrF84wDCS8dLGYp0n5+UeXqtjxgUNaj5l6GG+nwRUZa+8GcM3DJqyvwjY20cftapiWGt5hyokH6sf4cP+JRW2t4kkd77znAJzZ3cdxjYVT/PkYxXavnn8NmC1+fLwKvKMk5bLHUghBLz0BMb3nXsOLWPF1o7vLFp/d2FT5InuwYvm7K8+dFb6cU9USkoEWiUjn0nvGjWqIZFegRHLGyc/okBXKSqzRA/M2EBunQ/4t2EodxWqFqM5YR4Wq7wgV3V";
    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targetsRoverRuckus;
    public VuHelper(HardwareMap hwMap){
        //int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(1);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
    }

    public void vuActivate(){targetsRoverRuckus.activate();}

    public void vuDeActivate(){targetsRoverRuckus.deactivate();}

    public Image getImageFromFrame() throws InterruptedException{
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++){
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                return frame.getImage(i);
            }
        }
        return null;
    }
}
