package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TankTeleOP2020 extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;


    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        double leftPower;
        double rightPower;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            leftPower = -this.gamepad1.left_stick_y;
            rightPower = -this.gamepad1.right_stick_y;

            frontRight.setPower(rightPower);
            frontLeft.setPower(leftPower);
            backRight.setPower(rightPower);
            backLeft.setPower(leftPower);

            telemetry.addData("Right Power", frontRight.getPower());
            telemetry.addData("Left Power", frontLeft.getPower());
        }
    }
}

