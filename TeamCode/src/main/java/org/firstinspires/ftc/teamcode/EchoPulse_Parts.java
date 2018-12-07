package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

class EchoPulse_Parts {

    private HardwareMap hardwareMap;

    EchoPulse_Parts(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotor getMotorSF() {
        return hardwareMap.dcMotor.get("MotorSF");
    }

    DcMotor getMotorDF() {
        return hardwareMap.dcMotor.get("MotorDF");
    }

    DcMotor getMotorDS() {
        return hardwareMap.dcMotor.get("MotorDS");
    }

    DcMotor getMotorSS() {
        return hardwareMap.dcMotor.get("MotorSS");
    }

    DcMotor getMotorCarlig() {
        return hardwareMap.dcMotor.get("MotorCarlig");
    }

    LynxI2cColorRangeSensor getColorSensor() {
        return hardwareMap.get(LynxI2cColorRangeSensor.class, "cls");
    }

    BNO055IMU getGyro() {
        return hardwareMap.get(BNO055IMU.class, "imu");
    }
}
