#include "main.h"
#include "Lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cerrno>

// currently playing silksong so zero coding will get done 

/**
TROUBLESHOOTING:
IF CODE WILL NOT UPLOAD :
- CRTL - S, Clean, Build, Upload.

IF EXIT CODE 2
- Get new wire or upload directly to brain, ports fried or wire fried.
*/

// add a nope mech 

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group - ports 1 2 3
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - ports 4 5 6

// optical sensor for alliance colour detection
pros::Optical allianceSensor(17); // optical sensor on port 17

// ---- PISTONS ----
// Scraper piston and nope toggle
pros::adi::DigitalOut scraper('A');
// true  -> scraper DOWN
// false -> scraper UP
bool scraperDown = false;  

pros::adi::DigitalOut noperopedescore('B');
// true  -> nope UP (non-blocking)
// false -> nope DOWN (blocking)
bool noperopedescoreToggle = false;

pros::adi::DigitalOut nopepiston('D');
bool nopepistonbool = true;

pros::adi::DigitalOut outtakefunction('C');
bool outtakefunctiontoggle = false;


//booleans for director motor
bool directorReversePulseActive = false;


//MOTORS:

//flexwheel intake motor
pros::Motor intakechain(18, pros::MotorGearset::green); // 11W motor

//conveyer motors to the top
pros::Motor conveyerMotor(19, pros::MotorGearset::green); //5.5W Motor

//indexer / top roller motors
pros::Motor directormotor(20, pros::MotorGearset::blue); //11W motor

//Sensors:


// tracking wheels + inertial sensors

// Inertial Sensor on port 10
pros::Imu imu(6);
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed (will change later in testing)
pros::Rotation horizontalEnc(4);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (will change later in testing)
pros::Rotation verticalEnc(-5);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.45); //subject to change
// vertical tracking wheel Right. 2.75" diameter, 2.5" offset TBD, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.5); //subject to change

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325,  // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1,
                            nullptr, // vertical tracking wheel 2, 
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// lateral motion controller, kp 10, kd 55, 3, 0.5, 100, 1.5, 500, 20
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              42, // derivative gain (kD)
                                              3, // anti windup
                                              0.3, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
);
// angular motion controller Kp 8 and Kd 78 presviolsy, small error 0.5, smalltimeout 100, large error 1.5, large errorr timeout 500, slew 0
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
											  53, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              400, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (lew)
);

// sensors for odometry

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // 1) joystick deadband (in joystick units out of 127)
                                  0, // 2) minimum motor output (out of 127) once you leave deadband
                                  1.005// 3) exponential gain (dimensionless)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

bool mode = false;
bool scraperToggle = false;




// mode = false -> Hoard style (inverse)
// mode = true  -> eject style (together)
// scraper = true -> on
// scraper = false -> off
// nope = true -> nope off
// nope = false -> nope on (blocking)


void scraperNopeControl(bool mode, bool scraperDown) {
    if (!scraperDown) {
        // Scraper DOWN -> force nope DOWN
        scraper.set_value(true);   // scraper down
        noperopedescore.set_value(false);      // nope always down
    } else {
        // Scraper UP -> behavior depends on mode
        scraper.set_value(false);    // scraper up
        if (!mode) {
            noperopedescore.set_value(false);  // Hoard mode -> nope down
            directormotor.move(127);
            pros::delay(200);
            directormotor.move(0);
        } else {
            noperopedescore.set_value(true);   // eject mode -> nope up
        }
    }
}


void directorReversePulse() {
    if (directorReversePulseActive) return;
    directorReversePulseActive = true;


    pros::Task([] {
        // initially go backwards to prevent jamming
        directormotor.move(60);
        conveyerMotor.move(80);
        intakechain.move(-120);


        pros::delay(100);
        //score mid goal for like 1.7 sum seconds
        directormotor.move(-60);
        conveyerMotor.move(100);
        intakechain.move(120);


        pros::delay(1700);
        //stop
        directormotor.move(0);
        conveyerMotor.move(0);
        intakechain.move(0);


        directorReversePulseActive = false;
    });
}










/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen


    chassis.calibrate(); // calibrate sensors


    // Set motors to brake mode
    leftMotors.set_brake_mode(pros::MotorBrake::coast);
    rightMotors.set_brake_mode(pros::MotorBrake::coast);
    intakechain.set_brake_mode(pros::MotorBrake::brake);
    directormotor.set_brake_mode(pros::MotorBrake::brake);
   
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms


    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs


    // %d  int
    // %u  unsigned int
    // %f  float
    // %.2f  float w/ 2 decimals
    // %s  C-string
    // %c  char
    // %%  percent sign
    //
    // Width/Alignment:
    // %4d   pad to width 4
    // %04d  pad with zeros
    // %-4s  left-align string in width 4
    // %6.2f width 6, 2 decimals


    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "221K Origin");
            pros::lcd::print(4, "Indexer Current: %.2f A", directormotor.get_current_draw() / 1000.0);//checking wattage
            pros::lcd::print(5, "Intake Current: %.2f A", intakechain.get_current_draw() / 1000.0); //checking wattage
            pros::lcd::print(6, "Scraper actuation: %s", scraperDown ? "UP" : "DOWN");
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose() );
            // delay to save resources
            pros::delay(50);
        }
    });
}


/**
 * Runs while the robot is disabled
 */
void disabled() {
    // engage hold mode on drivetrain
    leftMotors.set_brake_mode(pros::MotorBrake::hold);
    rightMotors.set_brake_mode(pros::MotorBrake::hold);


    // set motor velocities to 0 so u cant get shoved around
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);




    // optionally keep looping to continuously enforce hold mode
    while (true) {
        pros::delay(100); // small delay to save resources
    }
}


/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy




void autonomous() {
    chassis.setPose(0, 0, 0);
    noperopedescore.set_value(true); //in horde mode
    nopepiston.set_value(false); //hode mode
	chassis.moveToPoint(0, 33, 1000,{.maxSpeed=85}); //go ahead of the match loader
	chassis.waitUntilDone();
	scraper.set_value(true);
	chassis.turnToHeading(-90,500); //turn to match loader
	chassis.waitUntilDone();

	intakechain.move(127);
    conveyerMotor.move(127);
    directormotor.move(127);

   
	chassis.moveToPose(-9.5, 32.25, -90, 1000);
	pros::delay(1200); //intake until 3 balls
    chassis.waitUntilDone();
    
 

    chassis.moveToPose(23, 32.25, -90, 1500, {.forwards = false}); //move into long goal
    chassis.waitUntil(23); //at fifteen inches stop movingintake so everythin can go up

    intakechain.move(0);
    conveyerMotor.move(0);
    directormotor.move(0);
    scraper.set_value(false);
    noperopedescore.set_value(false);
    nopepiston.set_value(true); //go into score mode as well

    chassis.waitUntil(30); //wait until more close to goal
    intakechain.move(127);
    conveyerMotor.move(127);
    directormotor.move(127);

    leftMotors.move(-25);
    rightMotors.move(-25);
    pros::delay(900); //score for a second

    leftMotors.move(0);
    rightMotors.move(0);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0); //reset coordinates

    chassis.moveToPoint(0, 12, 700);
    chassis.waitUntilDone();

    
    chassis.turnToHeading(-129.7, 700); 
    chassis.waitUntilDone();

    noperopedescore.set_value(true);
    nopepiston.set_value(false);
    
    chassis.moveToPose(-26.1, -8.9, -129.7, 1500, {.maxSpeed=85});
    chassis.waitUntil(12);
    
    intakechain.move(127);
    conveyerMotor.move(127);
    directormotor.move(127);
    scraper.set_value(true);

    chassis.waitUntilDone();
    chassis.turnToHeading(47.1, 500);
    chassis.waitUntilDone();


    chassis.moveToPoint(-39.9, -19.9, 900, {.forwards=false});
    chassis.waitUntilDone();

    intakechain.move(127);
    conveyerMotor.move(100);
    directormotor.move(-100);
    pros::delay(750);
    chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 1000);
    chassis.waitUntilDone();
    
    intakechain.move(127);
    conveyerMotor.move(127);
    directormotor.move(127);

    chassis.moveToPoint(-73, -11.7, 800);
    chassis.waitUntil(15);
    scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(-208.3, 500);
    scraper.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-63.9, -23.5, 800);
    outtakefunction.set_value(true);
    intakechain.move(-100);
    conveyerMotor.move(-127);
    directormotor.move(-127);



}

/**
 * Runs in driver control
 */
void opcontrol() {
    // Set motors to brake mode except for DT motors
    leftMotors.set_brake_mode(pros::MotorBrake::coast);
    rightMotors.set_brake_mode(pros::MotorBrake::coast);
    intakechain.set_brake_mode(pros::MotorBrake::brake);
    directormotor.set_brake_mode(pros::MotorBrake::brake);
    conveyerMotor.set_brake_mode(pros::MotorBrake::brake);


    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // math for slowing down turning at max controller value (127)
        rightX = rightX * (90.0 / 127.0); // dividing by 127 yields slower turn rate at max joystick level


        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);




    // toggle scraper when L2 is pressed
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        scraperDown = !scraperDown; // flip scraper state
        scraper.set_value(scraperDown);
    }


    // toggle nope roped to the descore when L1 is pressed acts as a dual mechanism (now i think he added a second piston too)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        noperopedescoreToggle = !noperopedescoreToggle;
        nopepistonbool = !nopepistonbool;
        nopepiston.set_value(nopepistonbool);
        noperopedescore.set_value(noperopedescoreToggle);
    }


    // print current mode to controller screen
    controller.print(1, 0, "Mode: %s", noperopedescoreToggle ? "HOARD" : "TOGGLE");




    // reminder to get the paddles i need them badly
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        directorReversePulse();
    }




    // Control intake, conveyor, director, and outtake piston together
    // put a boolean under literally everything so it wouldnt override w task
    if (!directorReversePulseActive) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakechain.move(127);
            conveyerMotor.move(127);
            directormotor.move(127);
            outtakefunction.set_value(false);


        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakechain.move(-127);
            conveyerMotor.move(-60);
            directormotor.move(-60);
            outtakefunction.set_value(true);


        } else {
            intakechain.move(0);
            conveyerMotor.move(0);
            directormotor.move(0);
            outtakefunction.set_value(false);
        }
    }


    // might also add a colour sensor like snacky so we can
    // distance sensor placed around the front of the outtake



    //delay to save resources ig
    pros::delay(10);


    }


}


