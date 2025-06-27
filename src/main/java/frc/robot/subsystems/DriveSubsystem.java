package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final TitanQuad roda1, roda2, roda3;

    private final TitanQuadEncoder encoder1, encoder2, encoder3;

    private final AHRS giroscopio;

    private final PIDController pidRoda1 = new PIDController(0, 0, 0), 
    pidRoda2 = new PIDController(0, 0, 0), 
    pidRoda3 = new PIDController(0, 0, 0);

    double[] posicaoAtual = new double[] {

    };

    public DriveSubsystem() {
        roda1 = new TitanQuad(Constants.titanID, 20000, Constants.roda1);
        roda2 = new TitanQuad(Constants.titanID, 20000, Constants.roda2);
        roda3 = new TitanQuad(Constants.titanID, 20000, Constants.roda3);

        encoder1 = new TitanQuadEncoder(roda1, Constants.roda1, Constants.distPorTick);
        encoder2 = new TitanQuadEncoder(roda3, Constants.roda2, Constants.distPorTick);
        encoder3 = new TitanQuadEncoder(roda3, Constants.roda3, Constants.distPorTick);

        giroscopio = new AHRS(SPI.Port.kMXP);
    }

    double vXrampa = 0, vYrampa = 0, vZrampa = 0;
    double encpassado = 0;

    public void omnidiretional_Local(double vXrDes, double vYrDes, double vZrDes) {
        double velMinX = 10.68, 
        velMinY = 8.01, //8.01
        velMinZ = 16.2, 
        velMaxX = 1, 
        velMaxY = 1, 
        velMaxZ = 1;

        vXrampa = rampaVelocidade(vXrampa, vXrDes, 3, 4);
        vYrampa = rampaVelocidade(vYrampa, vYrDes, 3, 4);
        vZrampa = rampaVelocidade(vZrampa, vZrDes, 8, 9);

        // limitador de velocidade minima ( evita que o robo pare sem chegar no
        // setPoint)
        double vXr = aplicaVelocidadeMinima(vXrampa, velMinX);
        double vYr = aplicaVelocidadeMinima(vYrampa, velMinY);
        double vZr = aplicaVelocidadeMinima(vZrampa, velMinZ);

        SmartDashboard.putString("vXr", vXr + "");
        SmartDashboard.putString("vYr", vYr + "");
        SmartDashboard.putString("vZr", vZr + "");

        // cinematica direta para o calculo da velocidade das rodas
        double vR1 = (vXr / 2) + ((Math.sqrt(3) / 2) * vYr) + (vZr);
        double vR2 = (vXr / 2) - ((Math.sqrt(3) / 2) * vYr) + (vZr);
        double vR3 = (-vXr) + (vZr);

        //controle feedForward
        double ffv1 = 1 * vR1;
        double ffv2 = 1 * vR2;
        double ffv3 = 1 * vR3;

        SmartDashboard.putString("v1 cm/s", ffv1 +"");
        SmartDashboard.putString("v2 cm/s", ffv2 +"");
        SmartDashboard.putString("v3 cm/s", ffv3 +"");

        double v1norm = MathUtil.clamp((ffv1 / 53.4), -velMaxX, velMaxX);
        double v2norm = MathUtil.clamp((ffv2 / 53.4), -velMaxY, velMaxY);
        double v3norm = MathUtil.clamp((ffv3 / 53.4), -velMaxZ, velMaxZ);

        SmartDashboard.putString("v1", v1norm +"");
        SmartDashboard.putString("v2", v2norm +"");
        SmartDashboard.putString("v3", v3norm +"");

        double encAtual = encoder2.getEncoderDistance();
        double deltaEnc = encAtual - encpassado;
        encpassado = encAtual;

        SmartDashboard.putString("v2Atual", deltaEnc +"");
        // acionamento de atuadores
        //roda1.set(v1norm);
        //roda2.set(v2norm);
        //roda3.set(v3norm);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("v1", roda1.getRPM()+"");
        SmartDashboard.putString("v2", roda2.getRPM()+"");
        SmartDashboard.putString("v3", roda3.getRPM()+"");
    }

    /*---------------- CLASSES AUXILIARES ----------------*/

    public double rampaVelocidade(double atual, final double desejada, final double deltaAcel,
            final double deltaDesacel) {

        final double erro = desejada - atual;

        if (Math.abs(erro) < Math.min(deltaAcel, deltaDesacel)) {
            return desejada;
        }
        // Se está acelerando (indo de valor men%or para maior)
        if (erro > 0) {
            return atual += Math.min(erro, deltaAcel);
        } else {
            // Está desacelerando (diminuindo)
            return atual += Math.max(erro, -deltaDesacel);
        }
    }

    private double aplicaVelocidadeMinima(final double velNorm, final double min) {

        if (velNorm == 0)
            return 0;
        if (Math.abs(velNorm) < min) {
            return Math.copySign(min, velNorm);
        }
        return velNorm;
    }
}