package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TankTeleOP2020 extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        double leftPower;
        double rightPower;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            leftPower = (this.gamepad1.left_stick_y)/2;
            rightPower = (-this.gamepad1.right_stick_y)/2;

            rightFront.setPower(rightPower);
            leftFront.setPower(leftPower);

            telemetry.addData("Right Power", rightFront.getPower());
            telemetry.addData("Left Power", leftFront.getPower());
        }
    }
}

