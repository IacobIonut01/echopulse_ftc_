package org.firstinspires.ftc.teamcode.rover_ruckus;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;

@Disabled
@TeleOp(name = "TokyoDrift", group = "Nadim")
public class TokyoDrift_Rumania extends LinearOpMode {

    private File maneauaDeSuflet = new File(Environment.getExternalStoragePublicDirectory("TokyoDrift") + "/maneaua.mp3");
    private DcMotor motorSF, motorDF, motorDS, motorSS;

    @Override
    public void runOpMode() {
        EchoPulse_Parts parts = new EchoPulse_Parts(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        telemetry.addData("Se incarca maneaua", "");
        telemetry.update();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0) {
                drift(gamepad1.left_stick_x);
            }
            if (gamepad1.left_stick_x == 0) {
                motorSF.setPower(0);
                motorSS.setPower(0);
                motorDF.setPower(0);
                motorDS.setPower(0);
            }
        }
    }

    private void drift(float val) {
        int power = 1;
        if (val>0) {
            motorSF.setPower(-power);
            motorSS.setPower(power);
            motorDF.setPower(power);
            motorDS.setPower(power);
        }
        if (val<0) {
            motorSF.setPower(-power);
            motorSS.setPower(-power);
            motorDF.setPower(power);
            motorDS.setPower(-power);
        }
    }

    private void move(Constants.Direction direction, int ms) {
        int power = 1;
        switch (direction) {
            case FORWARD:
                motorSF.setPower(-power);
                motorSS.setPower(-power);
                motorDF.setPower(power);
                motorDS.setPower(power);
                break;
            case BACKWARD:
                motorSF.setPower(power);
                motorSS.setPower(power);
                motorDF.setPower(-power);
                motorDS.setPower(-power);
                break;
        }
        sleep(ms);
    }

    private void setSteering(Constants.Direction direction) {
        switch (direction) {
            case LEFT:
                motorSF.setPower(1);
                motorDF.setPower(1);
                motorDS.setPower(1);
                motorSS.setPower(1);
                break;
            case RIGHT:
                motorSF.setPower(-1);
                motorDF.setPower(-1);
                motorDS.setPower(-1);
                motorSS.setPower(-1);
                break;
            case HLEFT:
                motorSF.setPower(1);
                motorDF.setPower(1);
                motorDS.setPower(-1);
                motorSS.setPower(-1);
                break;
            case HRIGHT:
                motorSF.setPower(-1);
                motorDF.setPower(-1);
                motorDS.setPower(1);
                motorSS.setPower(1);
                break;
        }
    }
}
