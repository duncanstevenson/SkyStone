package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class HardwareBase {
    public DriveSubsystem drive;
    public HardwareBase(boolean autonomous){
        drive = new DriveSubsystem(autonomous);
    }
}
