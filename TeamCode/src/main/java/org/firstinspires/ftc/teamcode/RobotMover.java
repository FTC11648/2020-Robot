package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMover {

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;
    BNO055IMU imu;

    public Orientation lastAngles = new Orientation();
    public double globalAngle;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    /* Declare OpMode members. */

    public RobotMover(DcMotor leftDrive, DcMotor rightDrive, DcMotor centerDrive, BNO055IMU imu)
    {
        this.imu = imu;

        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.centerDrive = centerDrive;

    }

    //Sets global angle to 0
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    //Returns global angle
    public double getAngle() {
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

    //LIT DRIFT IN A LINE
    //heading is the direction the robot drifts in, in degrees
    public void drift(double degrees, double heading, double distance) {
        double leftPower;
        double rightPower;
        double centerPower;

        double arcDistance = (degrees/360)*(16.5*Math.PI);


        double turn = arcDistance/distance;

        double straightPower;
        double sidePower;

        //Set start angle to zero
        resetAngle();

        //Do math
        straightPower = Math.cos(Math.toRadians(getAngle()-heading));
        sidePower = Math.sin(Math.toRadians(getAngle()-heading));

        //Do more math (there are two motors on the side)
        leftPower = straightPower + turn;
        rightPower = straightPower - turn;
        centerPower = sidePower*2;

        //Normalize or something
        if(Math.max(Math.max(leftPower, rightPower), centerPower) > 1) {
            double scale = 1/Math.max(Math.max(leftPower, rightPower), centerPower);
            leftPower = leftPower*scale;
            rightPower = rightPower*scale;
            centerPower = centerPower*scale;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        centerDrive.setPower(centerPower);

        while (Math.abs(getAngle()-degrees) >= 1) {
            //This loop drifts
        }

        //Stop
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        centerDrive.setPower(0);
    }

    //Rotates a certain number of degrees
    public void rotate(double degrees) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        double k = 0.005;

        double error = (degrees - getAngle())*k;

        leftPower = -error;
        rightPower = error;

        // set power to rotate.
        leftDrive.setPower(leftPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        rightDrive.setPower(rightPower);

        while (Math.abs(getAngle()-degrees) >= 1) {
            error = (degrees - getAngle())*k;

            leftPower = -error;
            rightPower = error;

            leftDrive.setPower(leftPower);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            rightDrive.setPower(rightPower);
        }

        // turn the motors off.
        leftDrive.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //Returns correction value
    private double getCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        double correction, angle;
        double gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;
        else
            correction = -angle;

        correction = correction * gain;

        return correction;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double centerInches) {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        resetAngle();

        // Determine new target position
        newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newCenterTarget = centerDrive.getCurrentPosition() + (int) (centerInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        centerDrive.setTargetPosition(newCenterTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double correction = getCorrection();

        //Start motion.
        leftDrive.setPower(Math.abs(speed)-correction);
        rightDrive.setPower(Math.abs(speed)+correction);
        centerDrive.setPower(Math.abs(speed));

        while((Math.abs(leftDrive.getCurrentPosition()-newLeftTarget))>2 ||
                (Math.abs(centerDrive.getCurrentPosition()-newCenterTarget))>2 ||
                (Math.abs(rightDrive.getCurrentPosition()-newRightTarget))>2 ) {
            correction = getCorrection();

            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            centerDrive.setPower(Math.abs(speed));
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        centerDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
