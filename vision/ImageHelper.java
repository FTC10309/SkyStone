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
 * Created by Bobo Qiu on 10/6/2018.
 * A helper class to provide a static method to get gold mineral position: Left, Center, Right
 * input is a frame from vuforia
 * also has a method to saveImage to record what the camera sees.
 */

public class ImageHelper {
    public final static int SKY_LEFT = 0;
    public final static int SKY_CENTER = 1;
    public final static int SKY_RIGHT = 2;
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
    private final static double FOUNDATION_MIN = 4000000;
    private final static int MIN_CONSECUTIVE_COLOR = 2;
    private final static int GRAB_ROWSUM_MIN = 10000;

    private Mat crop = null, mask = null;
    private int grabBottom = 0, grabWidth = 0;
    private int skyStoneOrder = SKY_NONE;

    public void takePicture(){
        if(crop!= null){
            saveColoredImage(crop);
        }
    }

    public int getGrabBottom() {return grabBottom;}

    public int getGrabWidth() {return grabWidth;}

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
    public boolean centerFoundation() {
        if(crop != null){
            int picWidth = 20;
            int picHeight = 200;
            //crop picture
            Mat cropped = new Mat(crop, new Rect(crop.width() - picWidth - 1, crop.height() - picHeight - 1, picWidth, picHeight));
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //apply yellow mask;
            mask = new Mat();
            Core.inRange(cropped,BLACK_LOW, BLACK_HIGH, mask);
            return Core.sumElems(mask).val[0] > 25500;
        }
        return false;
    }

    public boolean centerFoundationRed() {
        if(crop != null){
            int picWidth = 20;
            int picHeight = 200;
            //crop picture
            Mat cropped = new Mat(crop, new Rect(1, crop.height() - picHeight - 1, picWidth, picHeight));
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //apply yellow mask;
            mask = new Mat();
            Core.inRange(cropped,BLACK_LOW, BLACK_HIGH, mask);
            return Core.sumElems(mask).val[0] > 25500;
        }
        return false;
    }

    public boolean seeGrab(int height) {
        grabBottom = 0;
        grabWidth = 500;
        crop = new Mat(crop, new Rect(500,315,500,height));
        Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV_FULL);
        mask = new Mat();
        Core.inRange(crop,BLUE_LOW, BLUE_HIGH, mask);
        Mat rowSum = new Mat();
        Core.reduce(mask,rowSum,1, Core.REDUCE_SUM,4);
        for(int i = mask.height(); i>0; i--) {
            if(rowSum.get(i-1,0)[0]>GRAB_ROWSUM_MIN) {
                grabBottom = i;
                break;
            }
        }
        if (grabBottom<10){return false;}
        else{
            grabWidth = 0;
            double colSumMin = GRAB_ROWSUM_MIN*grabBottom/height;
            Mat colSum = new Mat();
            Core.reduce(mask,colSum,0, Core.REDUCE_SUM,4);
            for(int i = mask.width(); i>0; i--) {
                if(colSum.get(0,i-1)[0]>colSumMin) {
                    grabWidth++;
                }
            }
            return true;
        }
    }

    public int getStoneBottom() {
        if(mask == null) return 0;
        int centerColumn = mask.width()/2;
        int numPoints = 0;
        for(int i = mask.height(); i > 0; i--) {
            if (mask.get(i-1,centerColumn)[0] == 255) {
                if(numPoints == 2)return i+2;
                numPoints++;
            }else {
                if(numPoints > 0) numPoints--;
            }
        }
        return 0;
    }

    public double blueTopRight() {
        if (crop != null) {
            int picWidth = crop.width() - 1000;
            crop = new Mat(crop, new Rect(1000, 0, picWidth, 200));

            //apply yellow filter
            Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV_FULL);

            mask = new Mat();
            Core.inRange(crop,BLUE_LOW, BLUE_HIGH, mask);
            return(Core.sumElems(mask).val[0]);
        }
        return -1;
    }

    public int findStonesBlue(){
        if(crop != null) {
            int startX = 40, picWidth = 700, startY = 350, picHeight = 100;
            crop = new Mat(crop, new Rect(startX,startY,picWidth,picHeight));

            //apply yellow filter
            Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV_FULL);

            //apply yellow mask;
            mask = new Mat();
            Core.inRange(crop,YELLOW_LOW, YELLOW_HIGH, mask);
            byte [] b = new byte[picWidth*100];
            boolean[] yellowLines = new boolean[picWidth];
            mask.get(0,0,b);
            int [] lastNonZero = new int[picWidth];
            int [] totalZero = new int[picWidth];
            for(int i = 0; i < picWidth; i++) {
                lastNonZero[i] = -1;
                yellowLines[i] = false;
                totalZero[i] = 0;
            }
            int index = picWidth*100-1;
            for (int i =99; i >=0;i-- ) {
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
                int actualNonZero = 100-totalZero[i];
                if (expectedNonZero > 25 && actualNonZero >= expectedNonZero*9/10) {
                    yellowLines[i] = true;
                }
            }
            //saveRangeImage();
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
                }else {
                    numWhiteLinesFound = 0;
                }
            } //Found stone end
            for(int i = stoneEnd; i <picWidth; i++) {
                if(yellowLines[i] == false) {
                    numBlackLinesFound++;
                    sumColumnIndex += i;
                }
            }
            if(numBlackLinesFound < (picWidth-stoneEnd )/ 5) {
                skyStoneOrder = SKY_RIGHT;
            }else {
                int blackColumn = sumColumnIndex / numBlackLinesFound;
                if(blackColumn-stoneEnd < (picWidth-stoneEnd ) / 2) {
                    skyStoneOrder = SKY_LEFT;
                }else {
                    skyStoneOrder = SKY_CENTER;
                }
            }
        }
    	return skyStoneOrder;
    }

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
            byte [] b = new byte[picWidth*100];
            boolean[] yellowLines = new boolean[picWidth];
            mask.get(0,0,b);
            int [] lastNonZero = new int[picWidth];
            int [] totalZero = new int[picWidth];
            for(int i = 0; i < picWidth; i++) {
                lastNonZero[i] = -1;
                yellowLines[i] = false;
                totalZero[i] = 0;
            }
            int index = picWidth*100-1;
            for (int i =99; i >=0;i-- ) {
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
                int actualNonZero = 100-totalZero[i];
                if (expectedNonZero > 25 && actualNonZero >= expectedNonZero*9/10) {
                    yellowLines[i] = true;
                }
            }
            //saveRangeImage();
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
                }else {
                    numWhiteLinesFound = 0;
                }
            } //Found stone end
            for(int i = stoneEnd; i >= 0; i--) {
                if(yellowLines[i] == false) {
                    numBlackLinesFound++;
                    sumColumnIndex += i;
                }
            }
            if(numBlackLinesFound < stoneEnd / 5) {
                skyStoneOrder = SKY_LEFT;
            }else {
                int blackColumn = sumColumnIndex / numBlackLinesFound;
                if(blackColumn > stoneEnd / 2) {
                    skyStoneOrder = SKY_RIGHT;
                }else {
                    skyStoneOrder = SKY_CENTER;
                }
            }
        }
        return skyStoneOrder;
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

    public boolean saveRangeImage(){
        Calendar c = Calendar.getInstance();
        SimpleDateFormat dateformat = new SimpleDateFormat("hh_mm_ss");
        String datetime = dateformat.format(c.getTime());
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, "R"+datetime+".png");
        return (Imgcodecs.imwrite(file.toString(), mask));
    }

    public boolean seeRedFoundation(){
        if (crop != null){
            Mat cropped = new Mat(crop, new Rect(crop.width()/2,350,crop.width()/2,200));
//            saveColoredImage(cropped);
            //convert color to HSV
            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_RGB2HSV_FULL);
            //apply blue filter
            Mat mask = new Mat();
            Core.inRange(cropped,RED1_LOW, RED1_HIGH, mask);
            Mat mask2 = new Mat();
            Core.inRange(cropped, RED2_LOW, RED2_HIGH, mask2);
            Mat destMask = new Mat();
            Core.bitwise_or(mask,mask2,destMask);
//            saveRangeImage(mask);
            return(Core.sumElems(mask).val[0]>FOUNDATION_MIN);

        }else return false;
    }
    public boolean seeBlueFoundation(){
         if (crop != null){
            Mat cropped = new Mat(crop, new Rect(0,350,crop.width()/2,200));
            //saveColoredImage(cropped);
            //convert color to HSV
            Imgproc.cvtColor(cropped,cropped,Imgproc.COLOR_RGB2HSV_FULL);
            //apply blue filter
            Mat mask = new Mat();
            Core.inRange(cropped,BLUE_LOW, BLUE_HIGH, mask);
            //saveRangeImage(mask);
            return(Core.sumElems(mask).val[0]>FOUNDATION_MIN);

        }else return false;
    }

}
