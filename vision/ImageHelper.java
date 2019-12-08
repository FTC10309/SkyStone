package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.os.Environment;
import com.vuforia.Image;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * Created by Bobo Qiu on 10/6/2019.
 * A helper class to provide various vision detection method.
 * input is a frame from vuforia.
 */

public class ImageHelper {
    public final static int SKY_INSIDE = 0;
    public final static int SKY_CENTER = 1;
    public final static int SKY_OUTSIDE = 2;
    public final static int SKY_NONE = 4;
    private final static Scalar YELLOW_LOW = new Scalar(20,70,70);
    private final static Scalar YELLOW_HIGH = new Scalar(50,255,255);
    private final static Scalar RED1_LOW = new Scalar(0,50,50);
    private final static Scalar RED1_HIGH = new Scalar(20,255,255);
    private final static Scalar RED2_LOW = new Scalar(235,50,50);
    private final static Scalar RED2_HIGH = new Scalar(255,255,255);
    private final static Scalar BLUE_LOW = new Scalar(140,50,50);
    private final static Scalar BLUE_HIGH = new Scalar(180,255,255);
    private final static Scalar BLACK_LOW = new Scalar(0,0,0);
    private final static Scalar BLACK_HIGH = new Scalar(255,255,20);
    private final static int MIN_CONSECUTIVE_COLOR = 2;
    private final static double FOUNDATION_MIN = 4000000;

    private Mat crop = null, mask = null;
    private int skyStoneOrder = SKY_NONE;

    public void takePicture(){ if(crop!= null)saveColoredImage(crop);}

    /* Returns whether the robot is next to the Foundation by cropping two small rectangles
     *  on the bottom left and bottom right and compute the highest black horizontal line.
     *  Both the left and right image have to satisfy the predefined level.  It is used for the
     *  robot to make sure it touches the Foundation before moving it.
     */
    public boolean isAtFoundation(){
        int picWidth = 20;
        int picHeight = 250;
        //crop picture
        Mat cropped = new Mat(crop, new Rect(300,450,picWidth,picHeight));
        Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);
        mask = new Mat();
        Core.inRange(cropped,BLACK_LOW, BLACK_HIGH, mask);
        Mat rowSum = new Mat();
        Core.reduce(mask,rowSum,1, Core.REDUCE_SUM,4);
        int leftBlackTop = -1;
        for(int i = 0; i<picHeight; i++) {
            if(rowSum.get(i,0)[0]>2500) {
                leftBlackTop = i;
                break;
            }
        }
        cropped = new Mat(crop, new Rect(crop.width() - picWidth - 2,450,picWidth,picHeight));
        Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);
        mask = new Mat();
        Core.inRange(cropped,BLACK_LOW, BLACK_HIGH, mask);
        rowSum = new Mat();
        Core.reduce(mask,rowSum,1, Core.REDUCE_SUM,4);
        int rightBlackTop = -1;
        for(int i = 0; i<picHeight; i++) {
            if(rowSum.get(i,0)[0]>2500) {
                rightBlackTop = i;
                break;
            }
        }
        return (leftBlackTop > 150 && rightBlackTop > 165);
    }

    /*  Returns whether robot is aimed at the center of the Foundation by cropping a corner
     ^   and check whether it is black.  It is used by the robot to find aim for the Foundation.
     */
    private boolean centerFoundationXY(int x, int y){
        if(crop != null){
            int picWidth = 20;
            int picHeight = 200;
            //crop picture
            Mat cropped = new Mat(crop, new Rect(x, y, picWidth, picHeight));
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //apply black mask;
            mask = new Mat();
            Core.inRange(cropped,BLACK_LOW, BLACK_HIGH, mask);
            return Core.sumElems(mask).val[0] > 25500;
        }
        return false;
    }

    public boolean centerFoundation(){return centerFoundationXY(crop.width() - 21,crop.height() - 201);}

    public boolean centerFoundationRed() {return centerFoundationXY(1,crop.height() - 201);}

    /* Take a masked image and quickly calculates for each column whether it is white.  It allows
     *  the bottom to be black, but at least the top quarter needs to be white.  Once white is
     *  found from the bottom, the rest of the 9/10 of the points also need to be white.  It is done
     *  to compensate that camera is not always level nad perpendicular to the stones.
     */
    private boolean[] findWhiteLines(int picWidth, int picHeight){
        boolean[] yellowLines = new boolean[picWidth];
        byte [] b = new byte[picWidth*picHeight];
        mask.get(0,0,b);
        int [] lastNonZero = new int[picWidth];
        int [] totalZero = new int[picWidth];
        for(int i = 0; i < picWidth; i++) {
            lastNonZero[i] = -1;
            yellowLines[i] = false;
            totalZero[i] = 0;
        }
        int index = picWidth*picHeight-1;
        for (int i =picHeight-1; i >=0;i-- ) {
            for (int j = picWidth-1; j >=0; j--) {
                if (b[index]==0) {
                    totalZero[j]++;
                }else {
                    if(lastNonZero[j]==-1)lastNonZero[j] = i;
                }
                index--;
            }
        }
        for (int i = 0; i < picWidth; i++) {
            int expectedNonZero = lastNonZero[i];
            int actualNonZero = picHeight-totalZero[i];
            if (expectedNonZero > picHeight/4 && actualNonZero >= expectedNonZero*9/10) {
                yellowLines[i] = true;
            }
        }
        return yellowLines;
    }

    /* Find sky stone position on the blue side by first find a yellow line starting from left.
     *  then find the middle of blacklines and compare.
     */
    public int findStonesBlue(){
        if(crop != null) {
            int startX = 40, picWidth = 700, startY = 350, picHeight = 100;
            crop = new Mat(crop, new Rect(startX,startY,picWidth,picHeight));

            //apply yellow filter
            Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV_FULL);

            //apply yellow mask;
            mask = new Mat();
            Core.inRange(crop,YELLOW_LOW, YELLOW_HIGH, mask);
            boolean[] yellowLines = findWhiteLines(picWidth,picHeight);

            int numBlackLinesFound = 0;
            int numWhiteLinesFound = 0;
            int stoneEnd = 0;
            int sumColumnIndex = 0;
            for(int i = 0; i < picWidth; i++) {
                if(yellowLines[i] == true) {
                    numWhiteLinesFound++;
                    if(numWhiteLinesFound > MIN_CONSECUTIVE_COLOR) {
                        stoneEnd = i - MIN_CONSECUTIVE_COLOR;
                        break;
                    }
                }else numWhiteLinesFound = 0;
            } //Found stone end
            for(int i = stoneEnd; i <picWidth; i++) {
                if(yellowLines[i] == false) {
                    numBlackLinesFound++;
                    sumColumnIndex += i;
                }
            }
            if(numBlackLinesFound < (picWidth-stoneEnd )/ 5) skyStoneOrder = SKY_INSIDE;
            else {
                int blackColumn = sumColumnIndex / numBlackLinesFound;
                if(blackColumn-stoneEnd < (picWidth-stoneEnd ) / 2) skyStoneOrder = SKY_OUTSIDE;
                else skyStoneOrder = SKY_CENTER;
            }
        }
    	return skyStoneOrder;
    }

    /* Find sky stone position on the red side by first find a yellow line starting from right.
     *  then find the middle of blacklines and compare.
     */
    public int findStonesRed() {
        if(crop != null){
            int startX = 600, picWidth = crop.width()-600, startY = 350, picHeight = 100;
            //crop picture
            crop = new Mat(crop, new Rect(startX,startY,picWidth,picHeight));

            //apply yellow filter
            Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV_FULL);

            //apply yellow mask;
            mask = new Mat();
            Core.inRange(crop,YELLOW_LOW, YELLOW_HIGH, mask);
            boolean[] yellowLines = findWhiteLines(picWidth,picHeight);
            int numBlackLinesFound = 0;
            int numWhiteLinesFound = 0;
            int stoneEnd = 0;
            int sumColumnIndex = 0;
            for(int i = picWidth - 1; i >= 0; i--) {
                if(yellowLines[i] == true) {
                    numWhiteLinesFound++;
                    if(numWhiteLinesFound > MIN_CONSECUTIVE_COLOR) {
                        stoneEnd = i - MIN_CONSECUTIVE_COLOR;
                        break;
                    }
                }else numWhiteLinesFound = 0;
            } //Found stone end
            for(int i = stoneEnd; i >= 0; i--) {
                if(yellowLines[i] == false) {
                    numBlackLinesFound++;
                    sumColumnIndex += i;
                }
            }
            if(numBlackLinesFound < stoneEnd / 5) skyStoneOrder = SKY_INSIDE;
            else {
                int blackColumn = sumColumnIndex / numBlackLinesFound;
                if(blackColumn > stoneEnd / 2) skyStoneOrder = SKY_OUTSIDE;
                else skyStoneOrder = SKY_CENTER;
            }
        }
        return skyStoneOrder;
    }

    /* Returns the center of the sky stone when robot is 12 inches away.  The robot uses this info
     *  to decide how to grab the center of the sky stone.
     */
    public int findSkyCenter(){
        int picWidth = crop.width()-300;
        int picHeight = 50;
        //crop picture
        crop = new Mat(crop, new Rect(300,400,picWidth,picHeight));
        //apply yellow filter
        Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV_FULL);

        //apply yellow mask;
        mask = new Mat();
        Core.inRange(crop,YELLOW_LOW, YELLOW_HIGH, mask);
        boolean[] yellowLines = findWhiteLines(picWidth,picHeight);

        int numBlackLinesFound = 0;
        int sumColumnIndex = 0;
        for(int i = picWidth-1; i >= 0; i--) {
            if(yellowLines[i] == false) {
                numBlackLinesFound++;
                sumColumnIndex += i;
            }
        }
        if(numBlackLinesFound > 0) return sumColumnIndex / numBlackLinesFound;
        else return 580;
        //apply yellow filter
    }

    /* Returns whether the camera sees the red foundation by cropping out a rectangle on the right
     *  and check the total pixels in red.
     */
    public boolean seeRedFoundation(){
        if (crop != null){
            Mat cropped = new Mat(crop, new Rect(crop.width()/2,350,crop.width()/2,200));
            //convert color to HSV
            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_RGB2HSV_FULL);
            //apply blue filter
            Mat mask = new Mat();
            Core.inRange(cropped,RED1_LOW, RED1_HIGH, mask);
            Mat mask2 = new Mat();
            Core.inRange(cropped, RED2_LOW, RED2_HIGH, mask2);
            Mat destMask = new Mat();
            Core.bitwise_or(mask,mask2,destMask);
            return(Core.sumElems(mask).val[0]>FOUNDATION_MIN);

        }else return false;
    }

    /* Returns whether the camera sees the blue foundation by cropping out a rectangle on the left
     *  and check the total pixels in red.
     */
    public boolean seeBlueFoundation(){
        if (crop != null){
            Mat cropped = new Mat(crop, new Rect(0,350,crop.width()/2,200));
            //convert color to HSV
            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_RGB2HSV_FULL);
            //apply blue filter
            Mat mask = new Mat();
            Core.inRange(cropped,BLUE_LOW, BLUE_HIGH, mask);
            return(Core.sumElems(mask).val[0]>FOUNDATION_MIN);

        }else return false;
    }

    public void setImage(Image img){
        //convert vuforia image into bitmap image
        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());

        //turn bitmap into Matrix
        crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm,crop);
    }

    private boolean saveColoredImage(Mat inputMat){
        Calendar c = Calendar.getInstance();
        SimpleDateFormat dateformat = new SimpleDateFormat("hh_mm_ss");
        String datetime = dateformat.format(c.getTime());
        Mat bgrMat = new Mat();
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, "C"+datetime+".png");
        Imgproc.cvtColor(inputMat,bgrMat, Imgproc.COLOR_RGBA2BGR);
        return (Imgcodecs.imwrite(file.toString(), bgrMat));
    }
}
