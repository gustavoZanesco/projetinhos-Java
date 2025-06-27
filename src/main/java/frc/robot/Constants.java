/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int titanID = 23,
    roda1 = 1,
    roda2 = 2,
    roda3 = 0;

    public static double raioRobo = 15, // ajustar novamente
    raioRoda = 4.9,
    distPorTick = ( 2 * Math.PI * raioRoda ) / 1464,
    raioEngrenagem = 2.5 / 2,
    distanciaPorTickElevador = ( 2 * Math.PI * raioEngrenagem ) / 1464;
}
