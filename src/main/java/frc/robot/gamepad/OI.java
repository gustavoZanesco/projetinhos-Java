package frc.robot.gamepad;


//Import the joystick class
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OI extends SubsystemBase 
{
    //Create the joystick
    Joystick drivePad;

    public OI()
    {
        //initialize the joystick 
        drivePad = new Joystick(GamepadConstants.USB_PORT);
        SmartDashboard.putNumber("pino joystick", 0);
    }

    /**
     * Drive Controller
     */

        /**
         * @return the y-axis value from the drivePad right joystick
         */
        public double getRightDriveY()
        {
            double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
            if(Math.abs(joy) < 0.05)
                return 0.0;
            else  
                return joy;
        }

        /**
         * @return the x-axis value from the drivePad right Joystick
         */
        public double getRightDriveX()
        {
            double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
            if(Math.abs(joy) < 0.05)
                return 0.0;
            else
                return joy;
        }

        /**
         * @return the y-axis value from the drivePad left joystick
         */
        public double getLeftDriveY()
        {
            double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
            if(Math.abs(joy) < 0.05)
                return 0.0;
            else  
                return joy;
        }
    
        /**
         * @return the x-axis value from the drivePad left Joystick
         */
        public double getLeftDriveX()
        {
            double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
            if(Math.abs(joy) < 0.05)
                return 0.0;
            else
                return joy;
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveRightTrigger()
        {
            return drivePad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getBotaoPreciso()
        {
            return drivePad.getRawButton(GamepadConstants.RIGHT_BUMPER);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveLeftTrigger()
        {
            return drivePad.getRawButton(GamepadConstants.LEFT_TRIGGER);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveLeftBumper()
        {
            return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveXButton()
        {
            return drivePad.getRawButton(GamepadConstants.SHARE_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getTriangle()
        {
            return drivePad.getRawButton(GamepadConstants.TRIANGLE_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getSquare()
        {
            return drivePad.getRawButton(GamepadConstants.SQUARE_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getCircle()
        {
            return drivePad.getRawButton(GamepadConstants.CIRCLE_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getXis()
        {
            return drivePad.getRawButton(GamepadConstants.X_BUTTON) ;
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveBackButton()
        {
            return drivePad.getRawButton(GamepadConstants.SHARE_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveStartButton()
        {
            return drivePad.getRawButton(GamepadConstants.OPTIONS_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveRightAnalogButton()
        {
            return drivePad.getRawButton(GamepadConstants.RIGHT_ANALOG_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getDriveLeftAnalogButton()
        {
            return drivePad.getRawButton(GamepadConstants.LEFT_ANALOG_BUTTON);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean modoSeguroPS4()
        {
            return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
        }

        /**
         * @return a true or false depending on the input
         */
        public boolean getBotaoResetGiroscopio()
        {
            return drivePad.getRawButton(GamepadConstants.TOUCHPAD_BUTTON);
        }

        /**
         * @return the numeric value of the D Pad hat. D UP is 0 and rotates CW. Ex D Right is 90, D Down is 180 
         */
        public int getPOV()
        {
            return drivePad.getPOV();
        }

        @Override
        public void periodic() {
            SmartDashboard.putBoolean("botao X", drivePad.getRawButton( (int) SmartDashboard.getNumber("pino joystick", 0) ));
        }
}