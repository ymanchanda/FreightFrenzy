package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team10515.DbgLog;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;
import java.util.List;

public class FFCV {
    public OpenCvWebcam webcam;
    public FFCV_Pipeline pipeline;
    public TensorImageClassifier tfic;
    public List<TensorImageClassifier.Recognition> recognitions;

    public void init(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new FFCV_Pipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                DbgLog.msg("Successful camera open");
            }

            @Override
            public void onError(int errorCode) {
//                telemetry.addLine("Error code: " + errorCode);
//                telemetry.update();
                DbgLog.msg("Error code: " + errorCode);
            }
        });

        try {
            tfic = new TFICBuilder(hardwareMap, "converted_modelTFIC_FINAL2.tflite", "left", "neither", "right").build();
//            telemetry.addLine("Successful build of model");
            DbgLog.msg("Succesful build of model");
        } catch (IOException e) {
            e.printStackTrace();
            DbgLog.msg("Model build failed");
//            telemetry.addLine("Model build failed");
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        DbgLog.msg("Waiting for start");
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
    }

    public void recognize(boolean stopStreaming){
        recognitions = tfic.recognize(pipeline.getFirstMat());
        DbgLog.msg("Size of recognitions: " + recognitions.size());
        if(stopStreaming){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public String getPlacement(){
        DbgLog.msg(recognitions.get(0).getTitle());
        return recognitions.get(0).getTitle();
    }

    public float getConfidence(){
        return recognitions.get(0).getConfidence();
    }

    public int getFrameCount(){
        return webcam.getFrameCount();
    }
}
