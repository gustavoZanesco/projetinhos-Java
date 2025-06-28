package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DeslocamentoXYZ extends CommandBase {
    private final DriveSubsystem subDrive = RobotContainer.subDriveGlobal;

    public DeslocamentoXYZ() {
        addRequirements(subDrive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Xdesejado", 0); 
        SmartDashboard.putNumber("Ydesejado", 0); 
        SmartDashboard.putNumber("Zdesejado", 0);
    }

    @Override
    public void execute() {
        subDrive.omnidiretional_XYZ(SmartDashboard.getNumber("Xdesejado", 0), 
        SmartDashboard.getNumber("Ydesejado", 0), 
        SmartDashboard.getNumber("Zdesejado", 0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subDrive.pararMotores();
    }

}