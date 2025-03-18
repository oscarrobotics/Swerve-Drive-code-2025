package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.*;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Map;
import java.util.function.BooleanSupplier;


public class Claw extends SubsystemBase {
     // All hardware classes already have WPILib integration
    
    final TalonFX m_mount = new TalonFX(61); 
  
    //only a single neutral motor request is required for the system as it is always the same value
    private final NeutralOut m_brake = new NeutralOut();

 
    final TalonFXSimState m_mountSim = m_mount.getSimState();
 


    final PositionTorqueCurrentFOC m_mountFXOut = new PositionTorqueCurrentFOC(0).withSlot(0);
    final MotionMagicExpoTorqueCurrentFOC m_mountFXOut_mm = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    
    final CANcoder m_mount_encoder = new CANcoder(17);
    

     
    public final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    

    
    public final AngularVelocity k_max_arm_speed = RevolutionsPerSecond.of(5/60.0);



    // physical dimentions of the robot
    public final Distance k_intake_length = Inches.of(12);
    public final Distance k_algehook_center_length = Inches.of(20);

    //Mount/Claw wrist angles
    public final Angle k_min_angle =  Rotation.of(-.15);
    public final Angle k_max_angle = Degrees.of(0.3);

    public final Angle k_load_coral_position = Degrees.of(0);

    public final Angle k_stowed = Rotation.of(0.29);
    public final Angle k_load = Rotation.of(0.18);

    public Angle k_coral_position_1 = Rotation.of(0.27);
    public Angle k_coral_position_mid = Rotation.of(0.2);//middle scoreing heights
    public Angle k_coral_position_shoot_alge = Rotation.of(0.18);//middle scoreing heights
    public Angle k_coral_position_get_alge = Rotation.of(0.05);//middle scoreing heights
    public Angle k_coral_position_high = Rotation.of(0.04);//top scoreing height
   
    public Angle k_coral_position_floor = Rotation.of(-0.07);

    public final Angle k_alge_position_1 = Rotation.of(0);
    public final Angle k_alge_position_2 = Rotation.of(0);
    public final Angle k_alge_position_3 = Rotation.of(0);
    public final Angle k_alge_position_4 = Rotation.of(0);

    public final Angle k_process_alge_position = Rotation.of(0);
    public final Angle k_barge_alge_position = Rotation.of(0);

    
    

    
    TalonFXConfiguration  m_mount_config = new TalonFXConfiguration();

// variables controlling the movement of the claw wrist/pivoter
    private final double k_default_mount_ks = 0; // output to overcome static friction
    private final double k_default_mount_kp = 130; //proportional
    private final double k_default_mount_ki = 0; //integral
    private final double k_default_mount_kd = 30; //derivative
    private final double k_default_mount_kg = 2; // gravity; minimum ampage(?) for movement to account for opposing forces
    private final double k_default_mount_kff = 0;
    // mm_expo gains
    private final double k_default_mount_kV = 10; // voltage required to maintain a given velocity, in V/rps
    private final double k_default_mount_kA = 3; // voltage required to apply a given acceleration, in V/(rps/s)
    private final double k_default_mount_cVelocity = 0.4; // used for both mm and mm_expo
    
    private final double k_mount_current_limit = 30; // maximum ampage, usually whining from the motor means this is too low





    private ShuffleboardTab  m_mount_tab = Shuffleboard.getTab("Mount Tuning");

    // // private GenericEntry sh_sim= m_tab.add("Elevator Sim", m_mech2d);
    // private GenericEntry sh_mount_kp = m_mount_tab.add("Mount kP", k_default_mount_kp).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",10,"max",200)).getEntry(); 
    // private GenericEntry sh_mount_ki = m_mount_tab.add("Mount kI", k_default_mount_ki).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",100)).getEntry();
    // private GenericEntry sh_mount_kd = m_mount_tab.add("Mount kD", k_default_mount_kd).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",40)).getEntry();
    // private GenericEntry sh_mount_kg = m_mount_tab.add("Mount kG", k_default_mount_kg).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-10,"max",10)).getEntry();
    // // private GenericEntry sh_mount_kff = m_mount_tab.add("Mount kff", k_default_mount_kff).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",40)).getEntry();
    // // private GenericEntry sh_mount_kff_offset = m_mount_tab.add("Mount kff offset", k_default_mount_kff_offset).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-10,"max",10)).getEntry();
    // private GenericEntry sh_mount_current_limit = m_mount_tab.add("Mount Current Limit", k_mount_current_limit).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",150)).getEntry();
    // private GenericEntry sh_mount_cvelocity = m_mount_tab.add("Mount Cruise Velocity", k_default_mount_cVelocity).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).getEntry();
    // private GenericEntry sh_mount_kv = m_mount_tab.add("Mount kV", k_default_mount_kV).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();
    // private GenericEntry sh_mount_ka = m_mount_tab.add("Mount kA", k_default_mount_kA).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();

    private GenericEntry sh_coral_position_1 = m_mount_tab.addPersistent("coral_position_1", k_coral_position_1.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_min_angle.magnitude(),"max",k_max_angle.magnitude())).getEntry(); 
    private GenericEntry sh_coral_position_2 = m_mount_tab.addPersistent("coral_position_2", k_coral_position_mid.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_min_angle.magnitude(),"max",k_max_angle.magnitude())).getEntry();
    private GenericEntry sh_coral_position_3 = m_mount_tab.addPersistent("coral_position_3", k_coral_position_high.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_min_angle.magnitude(),"max",k_max_angle.magnitude())).getEntry();
    private GenericEntry sh_coral_position_4 = m_mount_tab.addPersistent("coral_position_4", k_coral_position_floor.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_min_angle.magnitude(),"max",k_max_angle.magnitude())).getEntry();
    // private GenericEntry sh_mount_kff = m_mount_tab.add("Mount kff", k_default_mount_kff).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",40)).getEntry();
    // private GenericEntry sh_mount_kff_offset = m_mount_tab.add("Mount kff offset", k_default_mount_kff_offset).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-10,"max",10)).getEntry();
    // private GenericEntry sh_mount_current_limit = m_mount_tab.add("Mount Current Limit", k_mount_current_limit).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",150)).getEntry();
    // private GenericEntry sh_mount_cvelocity = m_mount_tab.add("Mount Cruise Velocity", k_default_mount_cVelocity).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).getEntry();
    // private GenericEntry sh_mount_kv = m_mount_tab.add("Mount kV", k_default_mount_kV).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();
    // private GenericEntry sh_mount_ka = m_mount_tab.add("Mount kA", k_default_mount_kA).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();




//where things relating to the claws function is stored
    public Claw(){

        // configure the motor controller
        

        m_mount_config.Slot0.kP = k_default_mount_kp; // An error of 1 rotation results in 60 A output
        m_mount_config.Slot0.kI = k_default_mount_ki; // No output for integrated error
        m_mount_config.Slot0.kD = k_default_mount_kd; // A velocity of 1 rps results in 6 A output
        m_mount_config.Slot0.kG = k_default_mount_kg;
        m_mount_config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        // Peak output of 5 A
        m_mount_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(k_mount_current_limit)) //torque limit = max speed
        .withPeakReverseTorqueCurrent(Amps.of(-k_mount_current_limit)); // maximum speed it can go back, clockwise

        m_mount_config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);//Positive motor output = clockwise motion
        m_mount_config.MotorOutput.NeutralMode=NeutralModeValue.Brake; //When the motor is nuetral or disabled the default is Brake/dont move
            


        // bind the remote encoder to the mount motor

        CANcoderConfiguration m_mount_encoder_config = new CANcoderConfiguration(); // Class for CANcoder, a CAN based magnetic encoder that provides absolute and relative position along with filtered velocity. This handles the configurations for the com.ctre.phoenix6.hardware.CANcoder
        m_mount_encoder_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.6)); //The positive discontinuity point of the absolute sensor in rotations. This determines the point at which the absolute sensor wraps around, keeping the absolute position in the range [x-1, x).
        m_mount_encoder_config.MagnetSensor.withMagnetOffset(-0.339844); // This offset is added to the reported position, allowing the application to trim the zero position. When set to the default value of zero, position reports zero when magnet north pole aligns with the LED.
        m_mount_encoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //Direction of the sensor to determine positive rotation, as seen facing the LED side of the CANcoder. Counterclockwise motion also = positive motion
        
        

        
        m_mount_config.Feedback.FeedbackRemoteSensorID = m_mount_encoder.getDeviceID(); // Device id is [0,62]
        m_mount_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; //Talon will fuse another sensor's information with the internal rotor, which provides the best possible position and velocity for accuracy and bandwidth (this also requires setting FeedbackRemoteSensorID).
        m_mount_config.Feedback.SensorToMechanismRatio = 1.0; //The ratio of sensor rotations to the mechanism's output. 1+ is seen as a reduction
        m_mount_config.Feedback.RotorToSensorRatio = 100; //The ratio of motor rotor rotations to remote sensor rotations. 1+ is seen as a reduction
        
        //for motion magic controls
        m_mount_config.MotionMagic.MotionMagicCruiseVelocity = k_default_mount_cVelocity;
        m_mount_config.MotionMagic.MotionMagicExpo_kV = k_default_mount_kV;
        m_mount_config.MotionMagic.MotionMagicExpo_kA = k_default_mount_kA;



        // m_fx.getConfigurator().apply(fx_cfg);
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_mount_encoder.getConfigurator().apply(m_mount_encoder_config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        
        status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_mount.getConfigurator().apply(m_mount_config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        //set the position of the mechanism to 0, this is not a control but a delclaration that the position it is in is 0
        // m_mount.setPosition(0);

     

        SmartDashboard.putData("Update mount positions", new InstantCommand(this::update_locations));
        // SmartDashboard.putData("Update intake PID", new InstantCommand(this::configure_intake_from_dash));


    }



    // internal methond to set the positon using a unit aware object
    private void set_mount_angle_mm(Angle position){

        //    gt is greater than 
        // if (position.gt( k_max_angle)){
            
        //     // logger.log(position + " requested is greater than the max position ");
        //     position = k_max_angle;

        // }
        // else if (position.lt(k_min_angle)){

        //     //logger.log(position + " requested is less than the minimum position");
        //     position = k_min_angle;

        // }

        m_mount.setControl(m_mountFXOut_mm.withPosition(position));


    }

 




    public BooleanSupplier at_position(double tolerance){
        
        BooleanSupplier position_trigger = ()->{
            return Math.abs(m_mount.getClosedLoopError().getValueAsDouble())<tolerance
            && Math.abs(m_mount.getVelocity().getValueAsDouble())<tolerance;
        } ;
        
        return position_trigger;
    }

    public BooleanSupplier at_position(){

        return at_position(0.005);
    }

    public double get_reference(){

        return m_mount.getClosedLoopReference().getValueAsDouble();
        
    }
        

    



   
   
 
    

    //note that this method returns a command and is no a command itself
    public Command set_position_command_mm(Angle position){

        //  \/ run command template returns a run command that does the thing insided      
        return run(()-> set_mount_angle_mm(position));

    }

       
    //These values are in smart dashboard
    public void publish_mount_data(){

        // put data important for charaterizing the data to the smart dashboard
        double set_point =m_mount.getClosedLoopReference().getValueAsDouble();
        double error = m_mount.getClosedLoopError().getValueAsDouble();
        double tcurrent = m_mount.getTorqueCurrent().getValueAsDouble();
        double velocity = m_mount.getVelocity().getValueAsDouble();
        double acceleration = m_mount.getAcceleration().getValueAsDouble();
        double position = m_mount.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Mount Set Point", set_point);
        SmartDashboard.putNumber("Mount Error", error);
        SmartDashboard.putNumber("Mount Torque Current", tcurrent);
        SmartDashboard.putNumber("Mount Velocity", velocity);
        SmartDashboard.putNumber("Mount Acceleration", acceleration);
        SmartDashboard.putNumber("Mount Position", position);

        // SmartDashboard.putData("Claw Sim", m_mech2d);


        // SmartDashboard.putData("PID_Verification", m_elevator_motor.getClosedLoopSlot()
        
    }

    public void configure_mount_from_dash(){
        // configure the motor from the smart dashboard
        m_mount_config.Slot0.kP = SmartDashboard.getNumber("Mount kP", k_default_mount_kp); 
        m_mount_config.Slot0.kI = SmartDashboard.getNumber("Mount kI", k_default_mount_ki);
        m_mount_config.Slot0.kD = SmartDashboard.getNumber("Mount kD", k_default_mount_kd);
        m_mount_config.Slot0.kG = SmartDashboard.getNumber("Mount kG", k_default_mount_kg);
       

        m_mount_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(SmartDashboard.getNumber("Mount Current Limit", k_mount_current_limit)))
        .withPeakReverseTorqueCurrent(Amps.of(-SmartDashboard.getNumber("Mount Current Limit", k_mount_current_limit)));

        m_mount_config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Elevator Cruise Velocity", k_default_mount_cVelocity);
        m_mount_config.MotionMagic.MotionMagicExpo_kV = SmartDashboard.getNumber("Elevator kV", k_default_mount_kV);
        m_mount_config.MotionMagic.MotionMagicExpo_kA = SmartDashboard.getNumber("Elevator kA", k_default_mount_kA);


        
        m_mount.getConfigurator().apply(m_mount_config);


    }

    @Override
    public void periodic() {
        super.periodic();
     
        publish_mount_data();
    }


  



    public void update_locations(){
       
        k_coral_position_1 = Rotation.of(sh_coral_position_1.getDouble(k_coral_position_1.magnitude()));
        k_coral_position_mid = Rotation.of(sh_coral_position_2.getDouble(k_coral_position_mid.magnitude()));
        k_coral_position_high = Rotation.of(sh_coral_position_3.getDouble(k_coral_position_high.magnitude()));
        k_coral_position_floor = Rotation.of(sh_coral_position_4.getDouble(k_coral_position_floor.magnitude()));


    }
    

    
    
}
