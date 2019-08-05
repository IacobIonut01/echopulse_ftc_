package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.Behaviour.BREAK;
import static org.firstinspires.ftc.teamcode.Constants.Behaviour.FLOAT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HLEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.HRIGHT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Constants.Direction.RIGHT;

@TeleOp(name = "SkyStone_TeleOP", group = "FTC")
public class SkyManual extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();
    //Main DC
    private DcMotor motorSF, motorDF, motorDS, motorSS;
    //Rover Ruckus parts
    private DcMotor motorCarlig;
    //Misc
    private double speedDevider = 1;
    private boolean toggleSpeed = false;//porneste inertia
    private boolean toggleBrake = false;//opreste inertia
    private boolean useLimits = true;
    private int limitEnd;
    private int limitStart;
    private double rotPower = 0.75;

    @Override
    public void runOpMode() {
        Common parts = new Common(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        motorCarlig = parts.getMotorCarlig();
        motorSF.setDirection(DcMotor.Direction.FORWARD);
        motorDF.setDirection(DcMotor.Direction.FORWARD);
        motorDS.setDirection(DcMotor.Direction.FORWARD);
        motorSS.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCarlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialization Status update
        telemetry.addData("Initialized...", " | Waiting for Start | ");
        telemetry.update();
        elapsedTime.reset();
        waitForStart();
        //Controller available commands
        if (opModeIsActive())
            OPactions();
    }

    private void OPactions() {
        limitStart = motorCarlig.getCurrentPosition();
        limitEnd = motorCarlig.getCurrentPosition() - 11000;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double sasiuPowerY = (-gamepad1.left_stick_y / 1.2) / speedDevider;
            double sasiuPowerX = (gamepad1.left_stick_x / 1.5) / speedDevider;

            speedDevider();

            setLimits();

            double carligPower = 0.8;

            if (gamepad1.left_stick_y == 0) {
                motorSF.setPower(0);
                motorDF.setPower(0);
                motorDS.setPower(0);
                motorSS.setPower(0);
            }

            if (-gamepad1.left_stick_y > 0.2 || -gamepad1.left_stick_y < -0.2) {
                motorSF.setPower(-sasiuPowerY);
                motorDF.setPower(sasiuPowerY);
                motorDS.setPower(sasiuPowerY);
                motorSS.setPower(-sasiuPowerY);
            }

            implementSteering();

            //Constant status update
            telemetry.addData("Timp scurs", " : " + elapsedTime.toString());
            telemetry.addData("Status limite: ", useLimits ? "Activate" : "Dezactivate");
            telemetry.addData("Poizite carlig: ", motorCarlig.getCurrentPosition());
            telemetry.update();
            useCarlig(carligPower);
        }
    }

    private void setDirection(DcMotor.Direction direction) {
        motorSF.setDirection(direction);
        motorDF.setDirection(direction);
        motorDS.setDirection(direction);
        motorSS.setDirection(direction);
    }

    private void setLimits() {
        if (gamepad2.y)
            useLimits = false;

        if (gamepad2.x)
            useLimits = true;
    }

    private void implementSteering() {
        if (gamepad1.left_stick_x < -0.2)
            setSteering(HLEFT, false);

        if (gamepad1.left_stick_x > 0.2)
            setSteering(HRIGHT, false);

        if (gamepad1.right_stick_x < 0) {
            if (speedDevider == 1)
                setSteering(LEFT, false);
            else
                setSteering(LEFT, true);
        }
        if (gamepad1.right_stick_x > 0) {
            if (speedDevider == 1)
                setSteering(RIGHT, false);
            else
                setSteering(RIGHT, true);
        }
    }

    private void setSteering(Constants.Direction direction, boolean useDivider) {
        switch (direction) {
            case LEFT:
                if (!useDivider) {
                    motorSF.setPower(rotPower);
                    motorDF.setPower(rotPower);
                    motorDS.setPower(rotPower);
                    motorSS.setPower(rotPower);
                } else {
                    motorSF.setPower(rotPower / 1.25);
                    motorDF.setPower(rotPower / 1.25);
                    motorDS.setPower(rotPower / 1.25);
                    motorSS.setPower(rotPower / 1.25);
                }
                break;
            case RIGHT:
                if (!useDivider) {
                    motorSF.setPower(-rotPower);
                    motorDF.setPower(-rotPower);
                    motorDS.setPower(-rotPower);
                    motorSS.setPower(-rotPower);
                } else {
                    motorSF.setPower(-rotPower / 1.25);
                    motorDF.setPower(-rotPower / 1.25);
                    motorDS.setPower(-rotPower / 1.25);
                    motorSS.setPower(-rotPower / 1.25);
                }
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

    private void speedDevider() {
        if (gamepad1.b && toggleBrake) {
            setBehaviour(BREAK);
            toggleBrake = false;
        } else if (gamepad1.b && !toggleBrake) {
            setBehaviour(FLOAT);
            toggleBrake = true;
        }

        if (gamepad1.a && toggleSpeed) {
            speedDevider = 2.75;
            rotPower = 0.2;
            toggleSpeed = false;
        } //SlowMode ON
        else if (gamepad1.a && !toggleSpeed) {
            speedDevider = 1;
            rotPower = 0.75;
            toggleSpeed = true;
        } //SlowMode OFF
    }

    private void useCarlig(double carligPower) {
        if (useLimits) {
            if (gamepad2.dpad_down && motorCarlig.getCurrentPosition() < limitStart) {
                motorCarlig.setPower(carligPower);
            } else if (gamepad2.dpad_up && motorCarlig.getCurrentPosition() > limitEnd) {
                motorCarlig.setPower(-carligPower);
            } else {
                motorCarlig.setPower(0);
            }
        } else {
            if (gamepad2.dpad_down)
                motorCarlig.setPower(carligPower);
            else if (gamepad2.dpad_up)
                motorCarlig.setPower(-carligPower);
            else motorCarlig.setPower(0);
        }
    }

    private void setBehaviour(Constants.Behaviour behaviour) {
        switch (behaviour) {
            case BREAK:
                motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case FLOAT:
                motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
        }
    }
}
