package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class StackDeterminationPipeline extends OpenCvPipeline {

    /*
         * An enum to define the skystone position
         */
        public enum StackHeight
        {
            STACK_0,
            STACK_1,
            STACK_4
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        Mat mask = new Mat();

        /*
         * The core values which define the location and size of the sample regions
         */



        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */


        /*
         * Working variables
         */
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile StackHeight position = StackHeight.STACK_0;
        private volatile int height = -1;
        private volatile double heightWidthRatio = 0;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            final Scalar lower = new Scalar(13, 50, 20);//150, 90
            final Scalar upper = new Scalar(25, 255, 250);// 230, 255

            //inputToCb(input);
            Mat nonCroppedHsv = new Mat();
            Imgproc.cvtColor(input, nonCroppedHsv, Imgproc.COLOR_RGB2HSV);

            Rect rectCrop = new Rect(nonCroppedHsv.width()*3/4, nonCroppedHsv.height()*3/8, nonCroppedHsv.width()/4, nonCroppedHsv.height()/2);
            Mat hsv = new Mat(nonCroppedHsv, rectCrop);
            Core.inRange(hsv, lower, upper, mask);
            if(mask.height()!=rectCrop.height) input.release();
//
// Set to (0,0,0) all pixels that are 0 in the mask, i.e. not in range
            //Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
            //Core.bitwise_not(hsv, mask);
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((3*2) + 1, (2*2)+1));
            Imgproc.erode(mask, mask, kernel);
            Imgproc.dilate(mask, mask, kernel);
            //Imgproc.boundingRect(mask);
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
           // Imgproc.findContours( , contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );;
            double maxArea = 0;
            MatOfPoint largestContour = new MatOfPoint();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(input, boundingRect, new Scalar(255, 0, 0));
            try {
                heightWidthRatio = (double)boundingRect.height / (double)boundingRect.width;
            } catch(Exception e){
            }

            if(Double.isNaN(heightWidthRatio)) {
                position = StackHeight.STACK_0;
                height = 0;
            }
            else if(heightWidthRatio>3.0) {
                position = StackHeight.STACK_1;
                height = 1;
            }
            else {
                position = StackHeight.STACK_4;
                height = 4;
            }


            //hsv.setTo(new Scalar(0,0,0), mask);


//Convert back to RGBA (now i use 4 channels since the destination is RGBA)
            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */


            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */

            /*
             * Find the max of the 3 averages
             */


            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
//            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
            hsv.release();
            //input.release();
            nonCroppedHsv.release();
            hierarchy.release();
            kernel.release();
            return mask;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public StackHeight getAnalysis()
        {
            return position;
        }
        public int getHeight(){
            return height;
        }
        public double getHeightWidthRatio() {return heightWidthRatio;}
}
