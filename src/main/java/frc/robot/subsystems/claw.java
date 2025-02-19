package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class claw extends SubsystemBase {
     // All hardware classes already have WPILib integration
     final TalonFX m_intake = new TalonFX(31);
     final TalonFX m_mount = new TalonFX(32);
     
 
     final TalonFXSimState m_intakeSim = m_intake.getSimState();
     final TalonFXSimState m_mountSim = m_mount.getSimState();
 
     final VelocityVoltage m_intakeFXOut = new VelocityVoltage(0).withSlot(0);
     final PositionVoltage m_mountFXOut = new PositionVoltage(0).withSlot(0);
 
     final TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();
     final TalonFXConfiguration m_mountConfig = new TalonFXConfiguration();
     



     public claw(){
       /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
       m_intakeConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
       m_intakeConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
       m_intakeConfig.Slot0.kI = 0; // No output for integrated error
       m_intakeConfig.Slot0.kD = 0; // No output for error derivative
       m_intakeConfig.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    // Peak output of 8 volts
    m_intakeConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));


     }
 

    
}
