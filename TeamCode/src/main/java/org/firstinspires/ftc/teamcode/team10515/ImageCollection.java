package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;
import org.outoftheboxrobotics.tensorflowapi.ObjectDetection.TensorObjectDetector;

import java.io.IOException;
import java.util.List;

@TeleOp(name="Image Collection", group="Collection")
public class ImageCollection extends FreightFrenzyRobot {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    boolean ready = false;

    @Override
    public void init(){
        super.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error code: " + errorCode);
                telemetry.update();
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void loop(){
        super.loop();
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        if(ready){
            telemetry.addData("Zoom: ", webcam.getPtzControl().getZoom());
            telemetry.addData("Max zoom: ", webcam.getPtzControl().getMaxZoom());
            telemetry.addData("Min zoom: ", webcam.getPtzControl().getMinZoom());
        }

        if (getEnhancedGamepad1().isA()){
            telemetry.addLine("A Pressed");
//            pipeline.saveMatToDisk(pipeline.getFirstMat(), "mat");
            webcam.stopStreaming();
        }
        else if(getEnhancedGamepad1().isB()){
            telemetry.addLine("B Pressed");
            webcam.startStreaming(640, 480);
        }
        else if (getEnhancedGamepad1().isY()){
            telemetry.addLine("Y Pressed");
        }
        telemetry.update();
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * This is the mat object into which we will save the mat passed by the backend to processFrame()
         * It will be returned by getMat()
         */
        Mat mat = new Mat();
        int num = 0;

        @Override
        public Mat processFrame(Mat input) {
            num++;
            Size dim = new Size(320, 320);
            Imgproc.resize(input, mat, dim);
//            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA);
            if(num % 10 == 0 && num <= 200) {
                String name = String.format("mat(%d)", num / 10);
                saveMatToDisk(mat, name);
            }
            return mat;
        }

        @Override
        public void onViewportTapped(){
            viewportPaused = !viewportPaused;

            if (viewportPaused){
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }

        Mat getMat(){
            return mat;
        }
    }
}