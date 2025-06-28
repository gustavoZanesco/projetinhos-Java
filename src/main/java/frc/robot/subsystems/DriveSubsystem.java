package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final TitanQuad roda1, roda2, roda3;

    private final TitanQuadEncoder encoder1, encoder2, encoder3;

    private final AHRS giroscopio;

    /*
     * private final PIDController pidRoda1 = new PIDController(0, 0, 0), pidRoda2 =
     * new PIDController(0, 0, 0), pidRoda3 = new PIDController(0, 0, 0);
     */

    boolean reachedGoal = false;

    double[] posicaoGlobal = new double[] {
        0,
        0,
        0
    };

    public DriveSubsystem() {
        roda1 = new TitanQuad(Constants.titanID, 20000, Constants.roda1);
        roda2 = new TitanQuad(Constants.titanID, 20000, Constants.roda2);
        roda3 = new TitanQuad(Constants.titanID, 20000, Constants.roda3);

        encoder1 = new TitanQuadEncoder(roda1, Constants.roda1, Constants.distPorTick);
        encoder2 = new TitanQuadEncoder(roda3, Constants.roda2, Constants.distPorTick);
        encoder3 = new TitanQuadEncoder(roda3, Constants.roda3, Constants.distPorTick);

        giroscopio = new AHRS(SPI.Port.kMXP);

        SmartDashboard.putNumber("Xdesejado", 0); 
        SmartDashboard.putNumber("Ydesejado", 0); 
        SmartDashboard.putNumber("Zdesejado", 0);
    }

    double vXrampa = 0, vYrampa = 0, vZrampa = 0;
    double encpassado = 0;

    /**
     * Método utilizado para deslocar o robô no plano de referência global.
     *  <p>
     * Esse metodo não realiza nem calcula trajetórias que evitem obstáculos.
     * Apenas desloca o robo no plano onde não há obstáculos.
     */
    public void omnidiretional_XYZ(double Xdes, double Ydes, double Zdes) {
        double tolLinear = 5, tolAngular = 1;

        double[] posicaoDesejada = new double[] { Xdes, Ydes, Zdes };

        double Xerro = (posicaoDesejada[0] - posicaoGlobal[0]);
        double Yerro = (posicaoDesejada[1] - posicaoGlobal[1]);
        double Zerro = (posicaoDesejada[2] - posicaoGlobal[2]);

        final boolean xReached = Math.abs(Xerro) < tolLinear;
        final boolean yReached = Math.abs(Yerro) < tolLinear;
        final boolean zReached = Math.abs(Zerro) < tolAngular;

        if (xReached) { Xerro = 0; }
        if (yReached) { Yerro = 0; }
        if (zReached) { Zerro = 0; }

        reachedGoal = xReached && yReached && zReached;

        double vXg = Xerro;
        double vYg = Yerro;
        double vZg = Zerro * 0.5;

        omnidiretional_Global(vXg, vYg, vZg);
    }

    public void omnidiretional_Global(double vXg, double vYg, double vZg) {
        double angRad = Math.toRadians(posicaoGlobal[2]);

        double vXr = (Math.cos(angRad) * vXg) + (-Math.sin(angRad) * vYg);
        double vYr = (Math.sin(angRad) * vXg) + (Math.cos(angRad) * vYg);
        double vZr = vZg;

        omnidiretional_Local(vXr, vYr, vZr);
    }

    public void omnidiretional_Local(double vXr, double vYr, double vZr) {
        double velMinX = 5, velMinY = 5, velMinZ = 5, velMaxX = 0.75, velMaxY = 0.75, velMaxZ = 0.75;

        vXrampa = rampaVelocidade(vXrampa, vXr, 1, 4);
        vYrampa = rampaVelocidade(vYrampa, vYr, 1, 4);
        vZrampa = rampaVelocidade(vZrampa, vZr, 3, 7);

        // limitador de velocidade minima ( evita que o robo pare sem chegar no
        // setPoint)
        double vX = vXrampa;
        double vY = vYrampa;
        double vZ = vZrampa;

        SmartDashboard.putString("vXr", vX + "");
        SmartDashboard.putString("vYr", vY + "");
        SmartDashboard.putString("vZr", vZ + "");

        // cinematica direta para o calculo da velocidade das rodas
        double vR1 = (vX / 2) + ((Math.sqrt(3) / 2) * vY) + (vZ);
        double vR2 = (vX / 2) - ((Math.sqrt(3) / 2) * vY) + (vZ);
        double vR3 = (-vX) + (vZ);

        // controle feedForward
        double ffv1 = 1 * vR1;
        double ffv2 = 1 * vR2;
        double ffv3 = 1 * vR3;

        SmartDashboard.putString("v1 cm/s", ffv1 + "");
        SmartDashboard.putString("v2 cm/s", ffv2 + "");
        SmartDashboard.putString("v3 cm/s", ffv3 + "");

        double v1norm = MathUtil.clamp((ffv1 / 53.4), -velMaxX, velMaxX);
        double v2norm = MathUtil.clamp((ffv2 / 53.4), -velMaxY, velMaxY);
        double v3norm = MathUtil.clamp((ffv3 / 53.4), -velMaxZ, velMaxZ);

        SmartDashboard.putString("v1", v1norm + "");
        SmartDashboard.putString("v2", v2norm + "");
        SmartDashboard.putString("v3", v3norm + "");

        if (reachedGoal) {
            pararMotores();
            v1norm = 0;
            v2norm = 0;
            v3norm = 0;
        }

        // acionamento de atuadores
        roda1.set(v1norm);
        roda2.set(v2norm);
        roda3.set(v3norm);
    }

    private double encoder1Passado = 0, encoder2Passado = 0, encoder3Passado = 0;

    /**
     * método utilizado para atualizar a posição do robô
     */
    private void odometriaUpdate() {
        // velocidade das rodas
        final double encoder1Atual = encoder1.getRaw();
        double deltaEncoder1 = encoder1Atual - encoder1Passado;
        encoder1Passado = encoder1Atual;

        final double encoder2Atual = encoder2.getRaw();
        double deltaEncoder2 = encoder2Atual - encoder2Passado;
        encoder2Passado = encoder2Atual;

        final double encoder3Atual = encoder3.getRaw();
        double deltaEncoder3 = encoder3Atual - encoder3Passado;
        encoder3Passado = encoder3Atual;

        double vR1 = ((2 * Math.PI * Constants.raioRoda * deltaEncoder1) / 1464);
        double vR2 = ((2 * Math.PI * Constants.raioRoda * deltaEncoder2) / 1464);
        double vR3 = ((2 * Math.PI * Constants.raioRoda * deltaEncoder3) / 1464);

        // cinematica inversa para calcular velocidades locais ( robô )
        final double vXr = (vR1 / 3) + (vR2 / 3) - ((vR3 * 2) / 3);
        final double vYr = (vR1 / Math.sqrt(3)) + (-vR2 / Math.sqrt(3));
        final double vZr = (vR1 / (3 * Constants.raioRobo)) + (vR2 / (3 * Constants.raioRobo)) + (vR3 / (3 * Constants.raioRobo));

        final double Zg = posicaoGlobal[2];
        final double ZgRad = Math.toRadians(Zg);

        // matriz de rotacao inversa para o cálculo das velocidades globais
        double vXg = (Math.cos(ZgRad) * vXr) + (Math.sin(ZgRad) * vYr);
        double vYg = (-Math.sin(ZgRad) * vXr) + (Math.cos(ZgRad) * vYr);
        double vZg = Math.toDegrees(vZr);

        // atualizacao dos eixos globais
        posicaoGlobal[0] += vXg;
        posicaoGlobal[1] += vYg;
        posicaoGlobal[2] += vZg;
    }

    public void atualizarPosicaoGlobal(double x, double y, double z) {
        this.posicaoGlobal[0] = x;
        this.posicaoGlobal[1] = y;
        this.posicaoGlobal[2] = z;
    }

    public void pararMotores() {
        roda1.stopMotor();
        roda2.stopMotor();
        roda3.stopMotor();
    }

    @Override
    public void periodic() {
        //posicaoGlobal[2] = giroscopio.getAngle();
        odometriaUpdate();
        if (reachedGoal) { pararMotores(); }

        SmartDashboard.putString("X", posicaoGlobal[0] + "");
        SmartDashboard.putString("Y", posicaoGlobal[1] + "");
        SmartDashboard.putString("Z", posicaoGlobal[2] + "");
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