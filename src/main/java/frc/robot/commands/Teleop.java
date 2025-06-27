package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveSubsystem;

public class Teleop extends CommandBase
{
    /**
     * Bring in Subsystem and Gamepad instances
     */
    private static DriveSubsystem subDrive = RobotContainer.subDriveGlobal;
 
    private static final OI oi = RobotContainer.oi;

    /**
     * Joystick inputs
     */
    double inputLeftY = 0;
    double inputLeftX = 0;
    double inputRightX = 0;
    double inputRightY = 0;


    public Teleop()
    {
        addRequirements(subDrive);
    }

    /**
     * Code here will run once when the command is called for the first time
     */
    @Override
    public void initialize() 
    {
        //driveTrain.resetYaw();
    }

    /**
     * Code here will run continously every robot loop until the command is stopped
     */
    @Override
    public void execute() 
    {
        /**
         * Get Joystick data
         */
        inputLeftX = -oi.getLeftDriveX();
        inputLeftY = oi.getLeftDriveY();
        inputRightX = -oi.getRightDriveX();
        inputRightY = -oi.getRightDriveY();

        // ativacao dos atuadores com as entradas do controle

        if ( oi.getDriveLeftTrigger() ) { inputLeftX = 0; };
        if ( oi.getDriveRightTrigger() ) { inputLeftY = 0; }; 
        
        subDrive.omnidiretional_Local(-inputLeftX * 53.4, -inputLeftY * 53.4, -inputRightX * 53.4   );

        SmartDashboard.putNumber("vertical esq", -inputLeftX);
        SmartDashboard.putNumber("horizontal esq", -inputLeftY);
        SmartDashboard.putNumber("horizontal dir", -inputRightX);
        //if ( oi.getBotaoResetGiroscopio() ) { subBase.giroscopio.reset(); subBase.resetEncoder(); };
    }

    /**     
     * When the comamnd is stopped or interrupted this code is run
     * <p>
     * Good place to stop motors in case of an error
     */
    @Override
    public void end(boolean interrupted)
    {
        subDrive.omnidiretional_Local(0, 0, 0);
    }

    /**
     * Check to see if command is finished
     */
    @Override
    public boolean isFinished()
    {
        return oi.getBotaoResetGiroscopio();
    }

}