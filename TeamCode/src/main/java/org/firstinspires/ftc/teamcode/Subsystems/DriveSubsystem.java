package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.PID;

import java.sql.RowId;
import java.util.HashMap;

public class DriveSubsystem {
    private DcMotor leftDriveF = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveF = null;
    private DcMotor rightDriveB = null;
    private boolean isAuto;
    public double DriveP = 1;
    public double DriveI = 0;
    public double DriveD = 0;
    public int DriveE = 3;
    private PID LDFPID = new PID(DriveP, DriveI, DriveD, DriveE, 1);
    private PID LDBPID = new PID(DriveP, DriveI, DriveD, DriveE, 1);
    private PID RDFPID = new PID(DriveP, DriveI, DriveD, DriveE, 1);
    private PID RDBPID = new PID(DriveP, DriveI, DriveD, DriveE, 1);

    public DriveSubsystem(boolean autonomous) {
        isAuto = autonomous;
    }

    public void Periodic() {

    }

    public void initMotors(HardwareMap hardwareMap) {
        leftDriveF = hardwareMap.get(DcMotor.class, "LDF");
        leftDriveB = hardwareMap.get(DcMotor.class, "LDB");
        rightDriveF = hardwareMap.get(DcMotor.class, "RDF");
        rightDriveB = hardwareMap.get(DcMotor.class, "RDB");

        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        if (isAuto) {
            leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void OPDrive(double Left, double Right) {
        leftDriveF.setPower(Left);
        leftDriveB.setPower(Left);
        rightDriveF.setPower(Right);
        rightDriveB.setPower(Right);
    }

    public void POSDrive(double Speed, int LeftPos, int RightPos) {
        leftDriveF.setTargetPosition(LeftPos);
        leftDriveF.setPower(Math.abs(Speed));
        leftDriveB.setTargetPosition(LeftPos);
        leftDriveB.setPower(Math.abs(Speed));
        rightDriveF.setTargetPosition(RightPos);
        rightDriveF.setPower(Math.abs(Speed));
        rightDriveB.setTargetPosition(RightPos);
        rightDriveB.setPower(Math.abs(Speed));

        leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void PIDDrive(double Speed, int LeftPos, int RightPos, double looptime) {
        leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LDFPID.UpdateMaxSpeed(Speed);
        LDBPID.UpdateMaxSpeed(Speed);
        RDFPID.UpdateMaxSpeed(Speed);
        RDBPID.UpdateMaxSpeed(Speed);

        LDFPID.SetTarget(LeftPos);
        LDBPID.SetTarget(LeftPos);
        RDFPID.SetTarget(RightPos);
        RDBPID.SetTarget(RightPos);

        leftDriveF.setPower(LDFPID.Calculate(leftFPos(), looptime));
        leftDriveB.setPower(LDBPID.Calculate(leftBPos(), looptime));
        rightDriveF.setPower(RDFPID.Calculate(rightFPos(), looptime));
        rightDriveB.setPower(RDBPID.Calculate(rightBPos(), looptime));
    }

    public boolean atTargetPID() {
        return LDFPID.atTarget() && LDBPID.atTarget() && RDFPID.atTarget() && RDBPID.atTarget();
    }

    public boolean atTargetPOS() {
        return leftDriveF.isBusy() && leftDriveB.isBusy() && rightDriveF.isBusy() && rightDriveB.isBusy();
    }

    public int leftFPos() {
        return leftDriveF.getCurrentPosition();
    }

    public int leftBPos() {
        return leftDriveB.getCurrentPosition();
    }

    public int rightFPos() {
        return rightDriveF.getCurrentPosition();
    }

    public int rightBPos() {
        return rightDriveB.getCurrentPosition();
    }

    public void ResetEnc() {
        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void UpdateDrivePID() {
        LDFPID.UpdatePIDE(DriveP, DriveI, DriveD, DriveE);
        LDBPID.UpdatePIDE(DriveP, DriveI, DriveD, DriveE);
        RDFPID.UpdatePIDE(DriveP, DriveI, DriveD, DriveE);
        RDBPID.UpdatePIDE(DriveP, DriveI, DriveD, DriveE);
    }
}
