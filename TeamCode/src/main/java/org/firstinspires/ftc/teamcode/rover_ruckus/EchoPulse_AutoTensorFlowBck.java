package org.firstinspires.ftc.teamcode.rover_ruckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Constants.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.Constants.Direction.DOWN;
import static org.firstinspires.ftc.teamcode.Constants.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HLEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HRIGHT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.UP;

@Disabled
@Autonomous(name = "Autonomous w/ TensorFlow", group = "FTC")
public class EchoPulse_AutoTensorFlowBck extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorSF, motorDF, motorDS, motorSS, motorCarlig, armBase;
    private Servo rotatieCuva;

    private int limitEnd;
    private int limitStart;
    private int removedObject = -86;
    private int step = 0;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0, power = 0.3;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AexRVhH/////AAABmRwlvZsGx0Oor/vwJ7jQe7w0CCm9dj4XqZzZM+GKL0bAOBWbJZCukHVq80UOiV4X6fZipT53Y/ekerVZ4Y73NnXBy3fxFkz11J6LweNoe5HZNQEXbeCuTGGc4XhidpQPDhXGjwQW302VtF6gK4z9Sru7Lqyu+eYSeSfy8UhVs2VYLlCuP8vO8gJCbFG8dptNQGn/NVZP7BTugsioepH2DnoKmkj1kwMdbiQGZkAOLYrI/RqPVdR1qOyqY2dX4s2N3LPWkN39fh6VVMm7A353UAE4OYDPgj9Id4wWBlKUL0inI5TgbMFRTkPcvykUDS1N29aZ6tmBfIixe/RRWQXh4WAteCeZ34wMnL/bts8EDy4g";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private WebcamName webcamName;
    private int caz = 0;

    //INITIALIZARE VALORI END

    @Override
    public void runOpMode() {
        //INITIALIZARE START
        //INITIALIZARE VALORI START
        EchoPulse_Parts parts = new EchoPulse_Parts(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        motorCarlig = parts.getMotorCarlig();
        imu = parts.getGyro();
        armBase = parts.getDcBaza();
        rotatieCuva = parts.getRotatieCuva();
        limitStart = motorCarlig.getCurrentPosition();
        limitEnd = motorCarlig.getCurrentPosition() - 11000;
        webcamName = parts.getWebcam();
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;
        motorSF.setDirection(DcMotor.Direction.REVERSE);
        motorDF.setDirection(DcMotor.Direction.REVERSE);
        motorDS.setDirection(DcMotor.Direction.REVERSE);
        motorSS.setDirection(DcMotor.Direction.REVERSE);
        armBase.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCarlig.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.initialize(parametersIMU);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        //INITIALIZARE END
        //CALIBRARE GYRO START
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();
        runtime.reset();
        //CALIBRARE GYRO END
        sleep(500);
        //INITIALIZARE CAMERA WEB START
        initVuforia();
        initTfod();
        waitForStart();

        //INITIALIZARE CAMERA WEB END
        //Start
        if (opModeIsActive()) {
            //ACTIVARE CAMERA WB
            if (tfod != null) tfod.activate();
            //CALCULARE LIMITE
            limitStart = motorCarlig.getCurrentPosition();
            limitEnd = motorCarlig.getCurrentPosition() - 11000;
            //ALEGERE CAZUL DE ATUONOMIE
            while (opModeIsActive()) {
                switch (caz) {
                    case 0:
                        //RULARE CAZ 1
                        caseOne();
                        break;
                    case 1:
                        //RULARE CAZ 2
                        //caseTwo();
                        break;
                }
            }
        }

        //OPRIRE WEB CAM
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /*
     * Interactiuni
     **/

    private void caseOne() {
        switch (step) {
            case 0:

                useCarlig(UP);
                break;
            case 1:
                stabilizeOrientation();
                move(FORWARD);
                waitAndStop(500);
                step++;
                break;
            case 2:
                useCarlig(DOWN);
                break;
            case 3:
                move(BACKWARD);
                waitAndStop(400);
                stabilizeOrientation();
                step++;
                break;
            case 4:
                getGoldMineralPosition();
                break;
            case 5:
                boolean done = false;
                switch (removedObject) {
                    case -1:
                        done = true;
                        break;
                    case 0:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        rotate(-8);
                        setSteering(HRIGHT, 0.3);
                        waitAndStop( 710);
                        done = true;
                        break;
                    case 1:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        done = true;
                        break;
                    case 2:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        rotate(8);
                        setSteering(HLEFT, 0.3);
                        waitAndStop(710);
                        done = true;
                        break;
                }
                if (done) step++;
                break;
            case 6:
                move(FORWARD);
                waitAndStop(600);
                rotate(75);
                move(FORWARD, 2);
                waitAndStop(1700);
                rotate(50);
                move(FORWARD, 3);
                waitAndStop(1400);
                step++;
                break;
            case 7:
                move(BACKWARD, 3.3);
                waitAndStop(2600);
                step++;
                break;
            case 8:
                step++;
                break;
            case 9:
                stopAuto(true);
                stop();
                break;
        }
    }

    private void stabilizeOrientation() {
        rotate((int) getAngle());
    }

    private void caseTwo() {
        switch (step) {
            case 0:
                useCarlig(UP);
                break;
            case 1:
                move(FORWARD);
                waitAndStop(500);
                step++;
                break;
            case 2:
                setSteering(HRIGHT, 0.3);
                waitAndStop(250);
                step++;
                break;
            case 3:
                move(BACKWARD);
                waitAndStop(400);
                //stabilizeOrientation();
                step++;
                break;
            case 4:
                getGoldMineralPosition();
                break;
            case 5:
                boolean done = false;
                switch (removedObject) {
                    case -1:
                        done = true;
                        break;
                    case 0:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        rotate(-8);
                        setSteering(HRIGHT, 0.3);
                        waitAndStop( 710);
                        done = true;
                        break;
                    case 1:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        done = true;
                        break;
                    case 2:
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        rotate(8);
                        setSteering(HLEFT, 0.3);
                        waitAndStop(710);
                        done = true;
                        break;
                }
                if (done) step++;
                break;
            case 6:
                move(FORWARD, 2);
                waitAndStop(1500);
                step++;
                break;
            case 7:
                rotatieCuva.setPosition(rotatieCuva.getPosition() - 0.2);
                step++;
                break;
            case 8:
                rotate(-90);
                move(FORWARD);
                waitAndStop(800);
                rotate(-35);
                move(FORWARD, 3);
                waitAndStop(2000);
                useCarlig(DOWN);
                break;
            case 9:
                stopAuto(true);
                stop();
                break;
        }
    }

    private void getGoldMineralPosition() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        //FOLOSIM 'shouldScan' PENTRU A DETERMINA
        boolean shouldScan = true;
        if (updatedRecognitions != null && shouldScan) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            /*if (updatedRecognitions.size() == 2) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;

                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }

                if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    telemetry.addData("Gold Mineral Position", "Right");
                    shouldScan = false;
                    rotate(90);
                    setSteering(HRIGHT, 0.3);
                    waitAndStop(710);
                    rotate(-8);
                    move(FORWARD, 2);
                    waitAndStop(1000);
                    step++;
                    removedObject = 2;
                } else if (goldMineralX != -1 && silverMineral1X != -1) {
                    if (goldMineralX > silverMineral1X) {
                        telemetry.addData("Gold Mineral Position", "Center");
                        shouldScan = false;
                        rotate(90);
                        move(FORWARD, 2);
                        waitAndStop(1000);
                        step++;
                        removedObject = 1;
                    } else {
                        telemetry.addData("Gold Mineral Position", "Left");
                        shouldScan = false;
                        rotate(90);
                        setSteering(HLEFT, 0.3);
                        waitAndStop(710);
                        rotate(8);
                        move(FORWARD, 2);
                        waitAndStop(1000);
                        step++;
                        removedObject = 0;
                    }
                }
            }*/
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        shouldScan = false;
                        rotate(-90);
                        setSteering(HRIGHT, 0.3);
                        waitAndStop(710);
                        rotate(-8);
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        step++;
                        removedObject = 0;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        shouldScan = false;
                        rotate(-90);
                        setSteering(HLEFT, 0.3);
                        waitAndStop(710);
                        rotate(8);
                        move(BACKWARD, 2);
                        waitAndStop(1000);
                        step++;
                        removedObject = 2;
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        shouldScan = false;
                        rotate(-90);
                        move(FORWARD, 2);
                        waitAndStop(1000);
                        step++;
                        removedObject = 1;
                    }
                }
            }
            telemetry.update();
        }
    }

    private void rotate(int degrees) {
        resetAngle();
        if (degrees < 0) {   // turn right.
            setSteering(LEFT, power);
        } else if (degrees > 0) {   // turn left.
            setSteering(RIGHT, power);
        } else return;
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData(" I 0", "Done");
                telemetry.update();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData(" I 1", "Done");
                telemetry.addData("Status", getAngle() + " | " + degrees);
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData(" I 2", "Done");
                telemetry.addData("Status", getAngle() + " | " + degrees);
                telemetry.update();
            }
        stopAuto(false);
        sleep(1000);
        resetAngle();
    }

    private void useCarlig(Constants.Direction direction) {
        double carligPower = 1;
        switch (direction) {
            case UP:
                if (motorCarlig.getCurrentPosition() > limitEnd) {
                    motorCarlig.setPower(-carligPower);
                } else {
                    motorCarlig.setPower(0);
                    step++;
                }
                break;
            case DOWN:
                if (motorCarlig.getCurrentPosition() < limitStart) {
                    motorCarlig.setPower(carligPower);
                } else {
                    motorCarlig.setPower(0);
                    step++;
                }
                break;
        }
    }

    private void move(Constants.Direction direction) {
        resetAngle();
        switch (direction) {
            case FORWARD:
                motorSF.setPower(-power);
                motorSS.setPower(-power);
                motorDF.setPower(power - checkDirection());
                motorDS.setPower(power - checkDirection());
                break;
            case BACKWARD:
                motorSF.setPower(power);
                motorSS.setPower(power);
                motorDF.setPower(-power + checkDirection());
                motorDS.setPower(-power + checkDirection());
                break;
        }
    }

    private void move(Constants.Direction direction, double powerMultiply) {
        resetAngle();
        double multiplication = powerMultiply > 0 ? powerMultiply : 1;
        switch (direction) {
            case FORWARD:
                motorSF.setPower((-power)*multiplication);
                motorSS.setPower((-power)*multiplication);
                motorDF.setPower((power - checkDirection())*multiplication);
                motorDS.setPower((power - checkDirection())*multiplication);
                break;
            case BACKWARD:
                motorSF.setPower((power)*multiplication);
                motorSS.setPower((power)*multiplication);
                motorDF.setPower((-power + checkDirection())*multiplication);
                motorDS.setPower((-power + checkDirection())*multiplication);
                break;
        }
    }

    private void setSteering(Constants.Direction direction, double rotPower) {
        switch (direction) {
            case LEFT:
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                break;
            case RIGHT:
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                break;
            case HLEFT:
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                break;
            case HRIGHT:
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                break;
        }
    }

    /*
     * System Management
     **/

    private void waitAndStop(int milliseconds) {
        sleep(milliseconds);
        stopAuto(true);
    }

    private void stopAuto(boolean all) {
        motorDS.setPower(0);
        motorDF.setPower(0);
        motorSS.setPower(0);
        motorSF.setPower(0);
        if (all)
            motorCarlig.setPower(0);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.\
        correction *= gain;

        return correction;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}