
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "TeleopImNotARobot", group = "motor")
public class TeleopImNotARobot extends LinearOpMode {

    private DcMotor slide;
    private DcMotor left_back;
    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor right_front;
    private Servo claw;
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    Recognition recognition;

    /**
     * Describe this function...
     */
    private void runToPositionSlide() {
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void power() {
        left_back.setPower(0.35);
        left_front.setPower(0.35);
        right_back.setPower(0.35);
        right_front.setPower(0.35);
        while (!(!right_back.isBusy() && !right_front.isBusy() && !left_back.isBusy() && !left_front.isBusy())) {
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        claw = hardwareMap.get(Servo.class, "claw");
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();

        // Put initialization blocks here.
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0.15);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setTargetPosition(100);
            runToPositionSlide();
            powerSlide();
            Tolerance();
            left_back.setTargetPosition(1150);
            left_front.setTargetPosition(1150);
            right_back.setTargetPosition(1150);
            right_front.setTargetPosition(1150);
            runToPosition();
            power();
            Tolerance();
            left_back.setTargetPosition(0);
            left_front.setTargetPosition(0);
            right_back.setTargetPosition(0);
            right_front.setTargetPosition(0);
            left_back.setPower(0);
            left_front.setPower(0);
            right_back.setPower(0);
            right_front.setPower(0);
            slide.setPower(0);
            while (opModeIsActive()) {
                // Put loop blocks here.
                velocityTelementry();
                telemetry.update();
            }
        }

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Describe this function...
     */
    private void displayInfo(int i) {
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Label: " + recognition.getLabel() + ", Confidence: " + recognition.getConfidence(), "X: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))))) + ", Y: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0))))));
    }

    /**
     * Describe this function...
     */
    private void velocityTelementry() {
        telemetry.addData("velocity", left_back.getPower());
        telemetry.addData("velocity", left_front.getPower());
        telemetry.addData("velocity", right_back.getPower());
        telemetry.addData("velocity", right_front.getPower());
        telemetry.addData("position", left_back.getCurrentPosition());
        telemetry.addData("position", left_front.getCurrentPosition());
        telemetry.addData("position", right_back.getCurrentPosition());
        telemetry.addData("position", right_front.getCurrentPosition());
        telemetry.addData("position", slide.getCurrentPosition());
        telemetry.addData("is at target", !left_back.isBusy());
        telemetry.addData("is at target", !left_front.isBusy());
        telemetry.addData("is at target", !right_back.isBusy());
        telemetry.addData("is at target", !right_front.isBusy());
        telemetry.addData("is at target", !slide.isBusy());
    }

    /**
     * Describe this function...
     */
    private void Tolerance() {
        ((DcMotorEx) left_back).setTargetPositionTolerance(20);
        ((DcMotorEx) left_front).setTargetPositionTolerance(20);
        ((DcMotorEx) right_back).setTargetPositionTolerance(20);
        ((DcMotorEx) right_front).setTargetPositionTolerance(20);
        ((DcMotorEx) slide).setTargetPositionTolerance(25);
    }

    /**
     * Describe this function...
     */
    private void powerSlide() {
        slide.setPower(1);
        while (!(!slide.isBusy() && !slide.isBusy())) {
        }
    }

    /**
     * Describe this function...
     */
    private void runToPosition() {
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void Camera2023() {
        List<Recognition> recognitions;
        int index;

        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set isModelTensorFlow2 to true if you used a TensorFlow
        // 2 tool, such as ftc-ml, to create the model.
        //
        // Set isModelQuantized to true if the model is
        // quantized. Models created with ftc-ml are quantized.
        //
        // Set inputSize to the image size corresponding to the model.
        // If your model is based on SSD MobileNet v2
        // 320x320, the image size is 300 (srsly!).
        // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
        // If your model is based on SSD MobileNet V1 FPN 640x640 or
        // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
        tfod.useModelFromFile("model_20230117_194840.tflite", JavaUtil.createListWith("one", "two", "three"), true, true, 320);
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
        tfod.setClippingMargins(0, 80, 0, 0);
        tfod.setZoom(2.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfod.getRecognitions();
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                }
                telemetry.update();
            }
            tfod.deactivate();
        }
    }
}
