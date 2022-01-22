package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.outoftheboxrobotics.tensorflowapi.ObjectDetection.TFODBuilder;
import org.outoftheboxrobotics.tensorflowapi.ObjectDetection.TensorObjectDetector;

import java.io.IOException;
import java.util.List;

@TeleOp(name="TFOD Test IMAGE", group="Test")
public class TFODTest_IMAGE extends FreightFrenzyRobot {

    /* Declare OpMode members. */

    TensorObjectDetector tfod;
    List<TensorObjectDetector.Detection> detections;
    Mat mat;

    @Override
    public void init(){
        super.init();


        try {
            tfod = new TFODBuilder(hardwareMap, "converted_model.tflite", "element").build();
            telemetry.addLine("Successful build of model");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addLine("Model build failed");
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        mat = Imgcodecs.imread("firstMat.png");
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGB); //TODO: Try this line


        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void loop(){
        super.loop();
        if (getEnhancedGamepad1().isY()){
            telemetry.addLine("Y Pressed");
//            detections = tfod.recognize(pipeline.getMat());
            detections = tfod.recognize(mat); //TODO: Try this line
            telemetry.addLine("Size of list: " + detections.size());
//            telemetry.addLine("Left most coordinate of bounding box: " + detections.get(0).getLocation().left);
//            telemetry.addLine("Confidence: " + detections.get(0).getConfidence());
        }
        telemetry.update();
    }
}