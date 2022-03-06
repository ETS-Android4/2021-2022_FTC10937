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

package testFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled

public class distanceMeasurement extends OpenCvPipeline {
    private int width; // width of the image
    distanceMeasurement location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public distanceMeasurement(int width) {
        this.width = width;
    }
    Telemetry telemetry;
    Mat mat = new Mat();

    static double PERCENT_COLOR_THRESHOLD = 0.05;

    public distanceMeasurement(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(15, 30, 65);
        Scalar highHSV = new Scalar(32, 255, 255);
//        Scalar lowHSV = new Scalar(0, 0, 168);
//        Scalar highHSV = new Scalar(172, 111, 255);
        Mat thresh = new Mat();

//        Mat mask = new Mat();

//        Mat scr = new Mat(bitmap.getHeight(), bitmap.getWidth(), CV_8UC4);
        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);


//        Imgproc.GaussianBlur(mat, mat, new Size(3, 3), 0, 0);
        Imgproc.medianBlur(mat, mat, 3);


//        Imgproc.bilateralFilter(input, mat, 9, 75, 75);


//        // Use Canny Edge Detection to find edges
//        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 0, 300);

        Core.inRange(mat, lowHSV, highHSV, mat);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
//        Mat contours, hierarchy = Imgproc.findContours(thresh, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(mat, contours, -1, new Scalar(0,255, 0), 3);

        Mat cnt = contours.get(4);
//        Imgproc.drawContours(mat, [cnt], 0, (0, 255, 0), 3);
//        Imgproc.drawContours(mat, contours[4], 0, new Scalar(0, 255, 0), 3);


//        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
//        Rect[] boundRect = new Rect[contours.size()];
//        for (int i = 0; i < contours.size(); i++) {
//            contoursPoly[i] = new MatOfPoint2f();
//            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
//            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
//            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
//
//        }
//
//        telemetry.update();
//
////        Imgproc.cvtColor(input, mat, Imgproc.COLOR_GRAY2BGR);
//
        Scalar colorYellow = new Scalar(0, 255, 0);

//        double left_x = 0.25 * width;
//        double right_x = 0.75 * width;
//        boolean left = false; // true if regular stone found on the left side
//        boolean right = false; // "" "" on the right side
//        for (int i = 0; i != boundRect.length; i++) {
//            if (boundRect[i].x < left_x)
//                left = true;
//            if (boundRect[i].x + boundRect[i].width > right_x)
//                right = true;
//
//            // draw red bounding rectangles on mat
//            // the mat has been converted to HSV so we need to use HSV as well
//        }



        return mat;
    }

}

