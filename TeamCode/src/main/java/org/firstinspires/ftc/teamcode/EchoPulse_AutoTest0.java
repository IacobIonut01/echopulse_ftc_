package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomous", group = "FTC")
public class EchoPulse_AutoTest0 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorSF, motorDF, motorDS, motorSS, motorCarlig;
    private int limitEnd;
    private int limitStart;
    double carligPower = 0.8;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test0027", "God bless us");
        telemetry.update();

        motorSF = hardwareMap.get(DcMotor.class, "MotorSF");
        motorDF = hardwareMap.get(DcMotor.class, "MotorDF");
        motorDS = hardwareMap.get(DcMotor.class, "MotorDS");
        motorSS = hardwareMap.get(DcMotor.class, "MotorSS");
        motorCarlig = hardwareMap.get(DcMotor.class, "MotorCarlig");
        limitStart = motorCarlig.getCurrentPosition();
        limitEnd = motorCarlig.getCurrentPosition() - 13000;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        motorSF.setDirection(DcMotor.Direction.FORWARD);
        motorDF.setDirection(DcMotor.Direction.FORWARD);
        motorDS.setDirection(DcMotor.Direction.FORWARD);
        motorSS.setDirection(DcMotor.Direction.FORWARD);
        motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCarlig.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCarlig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();
        runtime.reset();
        waitForStart();
        telemetry.addData("Mode", "Running");
        telemetry.update();
        sleep(1000);
        // TODO: Add func
        while (opModeIsActive()) {
            //move("forward");
            if (motorCarlig.getCurrentPosition() < motorCarlig.getCurrentPosition() + 250)
                motorCarlig.setPower(-power);
            else
                motorCarlig.setPower(0);

            telemetry.addData("Carlig :", motorCarlig.getCurrentPosition());
            telemetry.update();
        }
    }

    private void stopAuto() {
        motorDS.setPower(0);
        motorDF.setPower(0);
        motorSS.setPower(0);
        motorSF.setPower(0);
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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
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
        correction*= gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees) {
        resetAngle();
        if (degrees < 0)
        {   // turn right.
            setSteering("right", false, power);
        }
        else if (degrees > 0)
        {   // turn left.
            setSteering("left", false, power);
        }
        else return;
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}
        stopAuto();
        sleep(1000);
        resetAngle();
    }

    private void startAuto() {
        //rotate(90);
        move("forward");
        sleep(2500);
        stopAuto();
    }

    private void useCarlig(boolean up) {
        if (up && motorCarlig.getCurrentPosition() < limitStart) {
            motorCarlig.setPower(carligPower);
        }
        else if (!up && motorCarlig.getCurrentPosition() > limitEnd) {
            motorCarlig.setPower(-carligPower);
        }
    }

    private void move(String direction) {
        switch (direction.toLowerCase()) {
            case "forward":
                motorSF.setPower(-power + checkDirection());
                motorSS.setPower(-power + checkDirection());
                motorDF.setPower(power);
                motorDS.setPower(power);
                break;
            case "backward":
                motorSF.setPower(power);
                motorSS.setPower(power);
                motorDF.setPower(-power + checkDirection());
                motorDS.setPower(-power + checkDirection());
                break;
        }
    }

    private void setSteering(String direction, boolean useDivider, double rotPower) {
        switch (direction.toLowerCase()) {
            case "left":
                if (!useDivider) {
                    motorSF.setPower(rotPower);
                    motorDF.setPower(rotPower);
                    motorDS.setPower(rotPower);
                    motorSS.setPower(rotPower);
                } else {
                    motorSF.setPower(rotPower / 1.5);
                    motorDF.setPower(rotPower / 1.5);
                    motorDS.setPower(rotPower / 1.5);
                    motorSS.setPower(rotPower / 1.5);
                }
                break;
            case "right":
                if (!useDivider) {
                    motorSF.setPower(-rotPower);
                    motorDF.setPower(-rotPower);
                    motorDS.setPower(-rotPower);
                    motorSS.setPower(-rotPower);
                } else {
                    motorSF.setPower(-rotPower / 1.5);
                    motorDF.setPower(-rotPower / 1.5);
                    motorDS.setPower(-rotPower / 1.5);
                    motorSS.setPower(-rotPower / 1.5);
                }
                break;
            case "hleft":
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                break;
            case "hright":
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                break;
        }
    }

}
