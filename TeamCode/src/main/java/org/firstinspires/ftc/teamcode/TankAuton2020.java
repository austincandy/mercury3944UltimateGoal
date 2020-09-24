package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This is our auton.
 */

@Autonomous(name="TankAuton2020", group="Linear Opmode")
public class TankAuton2020 extends LinearOpMode {


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;


    private int statusNum = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight_drive");



        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Orientation rotation = new Orientation();
        int block = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            switch (statusNum) {

                case 0:
                    encoderDrive(0.5, 5, "r");
                case 1:
                    encoderDrive(0.5, 5, "l");
                case 2:
                    encoderDrive(0.5, 5, "f");
                case 3:
                    encoderDrive(0.5, 5, "b");
                case 4:
                    // code
                case 5:
                    // code
                default:
                    break;
            }
//            double frontLeftDriveTarget = frontLeftDrive.getCurrentPosition() + 1000;
//            double frontRightDriveTarget = frontRightDrive.getCurrentPosition() + 1000;
//
//            while (opModeIsActive() && (frontLeftDrive.getCurrentPosition() < frontLeftDriveTarget && frontRightDrive.getCurrentPosition() < frontRightDriveTarget)) {
//
//                frontLeftDrive.setPower(1);
//                frontRightDrive.setPower(1);
//
//                telemetry.addData("Encoder", frontLeftDrive.getCurrentPosition());
//                telemetry.addData("Encoder", frontRightDrive.getCurrentPosition());
//                telemetry.update();
//            }
//            frontLeftDrive.setPower(0);
//            frontRightDrive.setPower(0);
        }







    /*

    Initialize the vision targeting system.

    */


//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
    }

    /*

    Lateral drive function, takes "f","b","r","l" for directions.

    */



    public void encoderDrive(double speed, double inches, String direction) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int frontLeftDriveTarget = frontLeftDrive.getCurrentPosition();
            int frontRightDriveTarget = frontRightDrive.getCurrentPosition();
            int backLeftDriveTarget = backLeftDrive.getCurrentPosition();
            int backRightDriveTarget = backRightDrive.getCurrentPosition();

            switch (direction) {
                case "f":
                    frontLeftDriveTarget += inches * COUNTS_PER_INCH;
                    frontRightDriveTarget += inches * COUNTS_PER_INCH;
                    backLeftDriveTarget += inches * COUNTS_PER_INCH;
                    backRightDriveTarget += inches * COUNTS_PER_INCH;

                    frontLeftDrive.setPower(speed);
                    frontRightDrive.setPower(speed);
                    backLeftDrive.setPower(speed);
                    backRightDrive.setPower(speed);
                    while (opModeIsActive() && (frontLeftDrive.getCurrentPosition() < frontLeftDriveTarget && frontRightDrive.getCurrentPosition() < frontRightDriveTarget && backRightDrive.getCurrentPosition() < backRightDriveTarget && backLeftDrive.getCurrentPosition() < backLeftDriveTarget)) {


                        telemetry.addData("Encoder", frontLeftDrive.getCurrentPosition());
                        telemetry.addData("Encoder", frontRightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "b":
                    frontLeftDriveTarget -= inches * COUNTS_PER_INCH;
                    frontRightDriveTarget -= inches * COUNTS_PER_INCH;
                    backLeftDriveTarget -= inches * COUNTS_PER_INCH;
                    backRightDriveTarget -= inches * COUNTS_PER_INCH;

                    frontLeftDrive.setPower(speed);
                    frontRightDrive.setPower(speed);
                    backLeftDrive.setPower(speed);
                    backRightDrive.setPower(speed);
                    while (opModeIsActive() && (frontLeftDrive.getCurrentPosition() > frontLeftDriveTarget && frontRightDrive.getCurrentPosition() > frontRightDriveTarget && backRightDrive.getCurrentPosition() > backRightDriveTarget && backLeftDrive.getCurrentPosition() > backLeftDriveTarget)) {


                        telemetry.addData("Encoder", frontLeftDrive.getCurrentPosition());
                        telemetry.addData("Encoder", frontRightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "r":
                    frontLeftDriveTarget += inches * COUNTS_PER_INCH;
                    frontRightDriveTarget -= inches * COUNTS_PER_INCH;
                    backLeftDriveTarget += inches * COUNTS_PER_INCH;
                    backRightDriveTarget -= inches * COUNTS_PER_INCH;

                    frontLeftDrive.setPower(speed);
                    frontRightDrive.setPower(speed);
                    backLeftDrive.setPower(speed);
                    backRightDrive.setPower(speed);

                    while (opModeIsActive() && (frontLeftDrive.getCurrentPosition() < frontLeftDriveTarget && frontRightDrive.getCurrentPosition() > frontRightDriveTarget && backRightDrive.getCurrentPosition() > backRightDriveTarget && backLeftDrive.getCurrentPosition() < backLeftDriveTarget)) {


                        telemetry.addData("Encoder", frontLeftDrive.getCurrentPosition());
                        telemetry.addData("Encoder", frontRightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                case "l":
                    frontLeftDriveTarget -= inches * COUNTS_PER_INCH;
                    frontRightDriveTarget += inches * COUNTS_PER_INCH;
                    backLeftDriveTarget -= inches * COUNTS_PER_INCH;
                    backRightDriveTarget += inches * COUNTS_PER_INCH;

                    frontLeftDrive.setPower(speed);
                    frontRightDrive.setPower(speed);
                    backLeftDrive.setPower(speed);
                    backRightDrive.setPower(speed);


                    while (opModeIsActive() && (frontLeftDrive.getCurrentPosition() > frontLeftDriveTarget && frontRightDrive.getCurrentPosition() < frontRightDriveTarget && backRightDrive.getCurrentPosition() < backRightDriveTarget && backLeftDrive.getCurrentPosition() > backLeftDriveTarget)) {


                        telemetry.addData("Encoder", frontLeftDrive.getCurrentPosition());
                        telemetry.addData("Encoder", frontRightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                    break;
                default:
                    break;
            }

            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
            backLeftDrive.setPower(0);

            sleep(100);   // pause after each move
            statusNum++;
        }
    }
}