package org.firstinspires.ftc.teamcode.team10515.auto;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class FFCV_Pipeline extends OpenCvPipeline {
    boolean viewportPaused;
    Mat mat = new Mat();
    Mat firstMat = new Mat();
    boolean done = false;

    @Override
    public Mat processFrame(Mat input) {
        Size dim = new Size(320, 320);
        Imgproc.resize(input, mat, dim);
        if(!done){
            done = true;
            firstMat = mat;
            saveMatToDisk(firstMat, "firstMat");
        }
        return mat;
    }

    @Override
    public void onViewportTapped(){
        viewportPaused = !viewportPaused;

        if (viewportPaused){
//            cam.pauseViewport();
        }
        else {
//            cam.resumeViewport();
        }
    }

    Mat getMat(){
        return mat;
    }

    Mat getFirstMat(){
        return firstMat;
    }

}
