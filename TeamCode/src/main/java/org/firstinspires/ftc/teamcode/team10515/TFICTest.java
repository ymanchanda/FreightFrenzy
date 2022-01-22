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

@TeleOp(name="TFIC Test", group="Test")
public class TFICTest extends FreightFrenzyRobot {

    /* Declare OpMode members. */

    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    TensorImageClassifier tfic;
    List<TensorImageClassifier.Recognition> recognitions;

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

        try {
            tfic = new TFICBuilder(hardwareMap, "converted_modelTFIC.tflite", "left", "right", "center").build();
            telemetry.addLine("Successful build of model");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addLine("Model build failed");
        }

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
            recognitions = tfic.recognize(pipeline.getFirstMat());
            telemetry.addLine("Size of list: " + recognitions.size());
//            telemetry.addLine("Highest recognition: " + recognitions.get(0).getTitle());
            while(!getEnhancedGamepad1().isB()){
                telemetry.addLine("Press B to continue");
                telemetry.update();
            }

        }
        telemetry.update();
//            sleep(100);
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * This is the mat object into which we will save the mat passed by the backend to processFrame()
         * It will be returned by getMat()
         */
        Mat mat = new Mat();
        Mat firstMat = new Mat();
        boolean done = false;

        @Override
        public Mat processFrame(Mat input) {
            Size dim = new Size(320, 320);
            Imgproc.resize(input, mat, dim);
//            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA);
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
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }

        Mat getMat(){
            return mat;
        }

        Mat getFirstMat(){
            return firstMat;
        }
    }
}