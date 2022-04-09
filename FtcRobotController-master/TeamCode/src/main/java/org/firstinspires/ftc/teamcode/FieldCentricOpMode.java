package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "spicy", group = "Wingstop")
public class FieldCentricOpMode extends OpMode {

    MotorEx BL;
    MotorEx FL;
    MotorEx BR;
    MotorEx FR;
    List<BNO055IMU> imus;
    BNO055IMU imu;
    BNO055IMU.Parameters paras;
    MecanumDrive drive;
    float offset;

    @Override
    public void init() {
        BL = new MotorEx(hardwareMap, "motorBL");
        FL = new MotorEx(hardwareMap, "motorFL");
        BR = new MotorEx(hardwareMap, "motorBR");
        FR = new MotorEx(hardwareMap, "motorFR");
        offset = 0;

        imus = hardwareMap.getAll(BNO055IMU.class);
        imu = imus.get(0);
        paras = new BNO055IMU.Parameters();
        paras.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(paras);

        FL.setInverted(true);
        BL.setInverted(true);
        drive = new MecanumDrive(false, FL, FR, BL, BR);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            offset = imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
        }
        drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - offset);

        telemetry.addLine("imu axis 1: " + imu.getAngularOrientation().firstAngle);
        telemetry.addLine("imu axis 2: " + imu.getAngularOrientation().secondAngle);
        telemetry.addLine("imu axis 3: " + imu.getAngularOrientation().thirdAngle);
        telemetry.addLine("offset: " + offset);
    }
}
