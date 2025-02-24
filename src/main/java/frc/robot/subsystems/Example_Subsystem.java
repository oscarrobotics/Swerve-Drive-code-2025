// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.units.measure.*;




// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.TorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// public class Example_Subsystem extends SubsystemBase { 

//     // Eample subsytem resembles a shooter on a pivot base;

//     //velocity torque foc exaple code, for when you have to control the speed of the mechanism 
//     //will be the motor used for the shooter wheel
//     private final TalonFX m_velocityFoC = new TalonFX(0);

//     private final TalonFXSimState  m_velocityFOC_sim = m_velocityFoC.getSimState();

//     private final VelocityTorqueCurrentFOC m_velocityFOC_out = new VelocityTorqueCurrentFOC(0);

//     // position torquecurrentFox example code, for when have to control the position of the nechanism
//     // will be the motor used for the twist mount
//     private final TalonFX m_positionFOC = new TalonFX(0);
    
//     private final TalonFXSimState m_positionFOC_sim =  m_positionFOC.getSimState();

//     private final PositionTorqueCurrentFOC m_positionFOC_out = new PositionTorqueCurrentFOC(0);

//     //only a single neutral motor request is required for the system as it is always the same value
//     private final NeutralOut m_brake = new NeutralOut();



    
//     // Sysid Routine code. Sysid is a gui tool for runing test inputis into the mechanism 
//     // and measureing how it responds, from this data Sysid will tell you parameters used
//     // in the configuration of the controler behavior so the mechanism moves correctly.
//     // this is an alternative to guessing/estaimateing the parameters

//     // Sysid Code For charaterizing the velocity based mechanism
//     private final VelocityTorqueCurrentFOC m_positionFOC_sysid_out = new VelocityTorqueCurrentFOC(0);
    


//     // Constants that describe the mexhanims properties,limits, and relations, to be used to do calualions in 
//     //metions to control and monitor the mechamism 

//     public final AngularVelocity k_max_shooter_speed = RevolutionsPerSecond.of(5000/60.0);
    
//     public final Angle k_min_angle =  Degrees.of(0);
//     public final Angle k_max_angle = Degrees.of(90);

   




    

//     public Example_Subsystem(){

//         // configure the motor controller 
//         TalonFXConfiguration velocity_configs = new TalonFXConfiguration();

//         /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
//         /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
//         velocity_configs.Slot0.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
//         velocity_configs.Slot0.kP = 5; // An error of 1 rotation per second results in 5 A output
//         velocity_configs.Slot0.kI = 0; // No output for integrated error
//         velocity_configs.Slot0.kD = 0; // No output for error derivative
//         // Peak output of 20 A
//         velocity_configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(20))
//         .withPeakReverseTorqueCurrent(Amps.of(-20));

//         /* Retry config apply up to 5 times, report if failure */
//         StatusCode status = StatusCode.StatusCodeNotInitialized;
//         for (int i = 0; i < 5; ++i) {
//         status = m_velocityFoC.getConfigurator().apply(velocity_configs);
//         if (status.isOK()) break;
//         }
//         if (!status.isOK()) {
//         System.out.println("Could not apply configs, error code: " + status.toString());
//         }
        

        
        
//         TalonFXConfiguration  position_configs = new TalonFXConfiguration();
//        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    
//         position_configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
//         position_configs.Slot1.kI = 0; // No output for integrated error
//         position_configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
//         // Peak output of 20 A
//         position_configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(20))
//         .withPeakReverseTorqueCurrent(Amps.of(-20));

        
//         // status = StatusCode.StatusCodeNotInitialized;
//         for (int i = 0; i < 5; ++i) {
//         status = m_positionFOC.getConfigurator().apply(position_configs);
//         if (status.isOK()) break;
//         }
//         if (!status.isOK()) {
//         System.out.println("Could not apply configs, error code: " + status.toString());
//         }
        
//         //set the position of the mechanism to 0, this is not a control but a delclaration that the position it is in is 0
//         m_positionFOC.setPosition(0);


//     }
    

//     // internal method to  set the position using a double vaule, vulnerable to using the wrong unit as this method is not unit aware
//     private void set_turntable_positon(double posision){

//         if (posision> k_max_angle.magnitude()){
            
//             // logger.log(position + " requested is greater than the max position ");
//             posision = k_max_angle.magnitude();

//         }
//         else if (posision< k_min_angle.magnitude()){

//             //logger.log(position + " requested is less than the minimum position");
//             posision = k_min_angle.magnitude();

//         }

//         m_positionFOC.setControl(m_positionFOC_out.withPosition(posision));


//     }

//     // internal methond to set the positon using a unit aware object
//     private void set_turntable_positon(Distance posision){

//            // gt is greater than 
//         if (posision.gt( k_max_angle)){
            
//             // logger.log(position + " requested is greater than the max position ");
//             posision = k_max_angle;

//         }
//         else if (posision.lt(k_min_angle)){

//             //logger.log(position + " requested is less than the minimum position");
//             posision = k_min_angle;

//         }

//         m_positionFOC.setControl(m_positionFOC_out.withPosition(1));


//     }


//     // roboto container(or other commands) can call this methond to get acces to a
//     // command that will call the internal private method that directly controls the 
//     // motor with the postion suppliied when this method was called
//     // ie, some part of code wants the position to be X, so it calls 
//     // command = m_EX_Subsystem.getposition_commmand(x) and command now
//     // contains a command of run(()->set_turntable_posiotn(x))
//     // it will then pass/bind this command to the Command Scheduler to run 
//     // when approprite( determinted by the binding type and the command type, ie run,...)
//     // the command sceduler will then call ()->set_turntable_position(x).
//     // the ()-> is needted becuse the Command scheduler cannont provide parameters at 
//     // time it would call it, so instead ()-> is a "wrapper" method with no name,
//     // called an anonomus function that takes no parameters, what "()" means, but calls
//     // the inner method that is "hard coded" with the value x at the time m_EX_Subsystem.getposition_commmand(x)
//     // is called, giving you a new run command/anonomus method for every postion, a meathod that writes methods to 
//     // call methdods

//     //note that this method returns a command and is no a command itself
//     public Command get_position_command(Angle position){

//             //  \/ run command template returns a run command that does the thing insided      
//         return run(()-> set_turntable_positon(position));
//                 // /\ annonmust meathod with no parameters 
//                         // /\ refrernce to method with "hard coded " postion value
//             }       

    



//     // private final SysIdRoutine m_sysIdRoutineVelocityFOC = new SysIdRoutine(
//     //     new SysIdRoutine.Config(
//     //         null,        // Use default ramp rate (1 V/s)
//     //         Volts.of(7), // Use dynamic voltage of 7 V
//     //         null,        // Use default timeout (10 s)
//     //         // Log state with SignalLogger class
//     //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
//     //     ),
//     //     new SysIdRoutine.Mechanism(
//     //         volts -> setControl(m_positionFOC_out.withVolts(volts)),
//     //         null,
//     //         this
//     //     )

//     );



// }
