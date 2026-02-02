#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"



// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-13, -14, -15}, pros::MotorGearset::blue); // left motor group - ports 13, 14, 15 (reversed)
pros::MotorGroup rightMotors({16, 17, 18}, pros::MotorGearset::blue); // right motor group - ports 16, 17, 18 (normal)

//MOTORS

//Intake Motors
pros::Motor IntakeMotor1 (12, pros::MotorGearset::blue);

pros::Motor IntakeMotor2 (-19, pros::MotorGearset::blue);

//PISTONS

//scraper piston
pros::adi::DigitalOut Scraper('A');

bool flagState = false;

//Triple State piston (1)
pros::adi::DigitalOut TripleState1('B');

bool Triplebool1 = false; 

//Triple State piston (2)
pros::adi::DigitalOut TripleState2('D');

bool Triplebool2 = true;

//descord piston
pros::adi::DigitalOut descore('C');

bool debool = false;

// Inertial Sensor on port 9
pros::Imu imu(9);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 7, not reversed
//pros::Rotation horizontalEnc(10);
// vertical tracking wheel encoder. Rotation sensor, port 10, not reversed
pros::Rotation verticalEnc(10);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13, // 13.7 inch track width, original was 10
                              lemlib::Omniwheel::NEW_325, // using new 2.75" omnis, original was 4"
                              450, // drivetrain rpm is 450, original was 360
                              0 // horizontal drift is 2. If we had traction wheels, it would have been 8. Original was at 2
);
    
// lateral motion controller
lemlib::ControllerSettings linearController(7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            35, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 //  maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4.8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             35, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.005 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors b

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    
    // Path
 	chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(90, 999999);
    TripleState1.set_value(true);
    TripleState2.set_value(false);
    IntakeMotor1.move(127);
    IntakeMotor2.move(127);
    chassis.moveToPoint(0, 32, 4000, {.maxSpeed = 38});
    chassis.waitUntilDone();
    pros::delay(2000);
    IntakeMotor1.move(0);
    IntakeMotor2.move(0);
    chassis.turnToHeading(110, 1000);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(39, 5, 2000, {.maxSpeed = 64});
	chassis.waitUntilDone();
    chassis.turnToHeading(155, 1000);
    pros::delay(500);
	/*
    flagState = !flagState;
    Scraper.set_value(flagState);
    pros::delay(500);
    chassis.moveToPoint(37, -28, 2000, {.maxSpeed = 100});
    IntakeMotor1.move(127);
    IntakeMotor2.move(127);
    pros::c::delay(500);
    chassis.moveToPoint(37, -28, 500);
    pros::c::delay(250);
    chassis.moveToPoint(31, 10, 1000, {.forwards = false});
    pros::delay(1000);
    IntakeMotor1.move(-127);
    IntakeMotor2.move(-127);
    pros::delay(150);
    IntakeMotor1.move(0);
    IntakeMotor2.move(0);
    pros::delay(150);
    TripleState1.set_value(false);
    TripleState2.set_value(false);
    IntakeMotor1.move(127);
    IntakeMotor2.move(127);
*/
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {    //the whole robot intakes + pneumatics comprassing 
            IntakeMotor1.move(127);
            IntakeMotor2.move(127);
            TripleState1.set_value(true);
            TripleState2.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { //the whole robot intakes + pneumatics extending 
            IntakeMotor1.move(127);
            IntakeMotor2.move(127);
            TripleState1.set_value(false);
            TripleState2.set_value(false);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { //the whole robot intakes
            IntakeMotor1.move(127);
            IntakeMotor2.move(127);
            TripleState1.set_value(true);
            TripleState2.set_value(false);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { //the whole robot outtakes
            IntakeMotor1.move(-127);
            IntakeMotor2.move(-127);
            TripleState1.set_value(true);
            TripleState2.set_value(false);
        } else {    //stops all of the processes 
            IntakeMotor1.move(0);
            IntakeMotor2.move(0);
            TripleState1.set_value(true);
            TripleState2.set_value(false);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {   //scraper
            flagState = !flagState;
            Scraper.set_value(flagState);
        } 
        

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {   //descore
            debool = !debool;
            descore.set_value(debool);
        }  
    }
}
