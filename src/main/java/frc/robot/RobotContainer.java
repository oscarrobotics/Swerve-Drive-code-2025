// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Eleclaw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    
    




    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric strafe = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController controlstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Elevator elevator = new Elevator();
    public final Claw claw = new Claw();


    //Used to keep track of which direction the controller should drive the robot
    //in the condes curret state the orentation of the robot to the field can be messed up 
    //by certain parts of the executrion of autonomous, and the robot will drive opposite of 
    //what the driver expects, as a "temporary" fix, this will allow the driver to flip the direction 
    //of the axis to correct for this if needed, should not be needed after vision is implemented
    // but may still be kept just in case
    double forward_dir = -1;
    double side_dir = -1;

    // public final Eleclaw eleclaw = new Eleclaw(elevator, claw);



    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {


        NamedCommands.registerCommand("EjectCoral", claw.outtake_coral_command());
        
        NamedCommands.registerCommand("PickCoral", claw.intake_coral_command());
        autoChooser = AutoBuilder.buildAutoChooser("tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        //code used for both intake and outake of coral from the claw during auto and teleop 

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        

        //sets the drive train to use the joystick for control by default action eveyloop
        //the speed control based on evelator position is also implemented here as well
        //as the driver orientation flip apllication

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY()*Math.abs(joystick.getLeftY() )* MaxSpeed*forward_dir *  (elevator.is_stowed()&& !joystick.rightBumper().getAsBoolean() ? 1:0.3)) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX()*Math.abs(joystick.getLeftX() ) * MaxSpeed *side_dir* (elevator.is_stowed()&& !joystick.rightBumper().getAsBoolean() ? 1:0.3)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
      

        // gives the driver the ability to strafe the robot in a robot centric manner to assit with lining up with field elements
        // may need to implement a way to adjust the speed of this to allow for more precise control
        joystick.povUp().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityX(MaxSpeed*0.3).withVelocityY(0)));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityY(MaxSpeed*0.3).withVelocityX(0)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityX(-MaxSpeed*0.3).withVelocityY(0)));
        joystick.povRight().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityY(-MaxSpeed*0.3).withVelocityX(0)));
        // todo potentially
        // joystick.povDownLeft(). 
        // joystick.povDownRight().
        //  etc....
        

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(joystick.getLeftY(), joystick.getLeftX()))
        // ));


        // toggles the values of the forward and side direction variables that control the direction of the robot
        joystick.x().onTrue(new InstantCommand(()->flip_for()));
        joystick.y().onTrue(new InstantCommand(()->flip_side()));
     
       


        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

    //    controlstick.y().onTrue(elevator.get_posiCommand(25.0));
        // controlstick.leftBumper().onTrue(claw.position_command(claw.k_load_coral_position));
        //binds buttons to elevator position commands
        controlstick.a().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_1));
        controlstick.b().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2));
        controlstick.x().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3));
        controlstick.y().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4));

        controlstick.povUp().onTrue(claw.set_position_command_mm(claw.k_coral_position_1));
        controlstick.povLeft().onTrue(claw.set_position_command_mm(claw.k_coral_position_2));
        controlstick.povDown().onTrue(claw.set_position_command_mm(claw.k_coral_position_3));
        controlstick.povRight().onTrue(claw.set_position_command_mm(claw.k_coral_position_4));


        //binds buttons to intake and outtake commands
        controlstick.leftBumper().onTrue(claw.outtake_coral_command());
        controlstick.rightBumper().onTrue(claw.intake_coral_command());

        

        
        System.out.println("bindings configured");

        
        // controlstick.leftBumper().onTrue(claw.position_command(claw.k_load_coral_position));

        // elevator.setDefaultCommand(elevator.set_position_command_angle(elevator.k_stowed));
        // elevator.setDefaultCommand(new InstantCommand(elevator::configure_from_dash, elevator));
        // SmartDashboard.putData("configure elevator", new InstantCommand(elevator::configure_from_dash));
        // claw.setDefaultCommand(claw.set_position_command_mm(claw.k_stowed));
            


    }

    void flip_for(){
        forward_dir=forward_dir*-1;

    }
    
    void flip_side(){
        side_dir=side_dir*-1;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
