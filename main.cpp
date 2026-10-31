#include <Arduino.h>
#include <math.h>
#include <string>
#include <cstring>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// ECE493 HOVERBOARD CONTROL
//
// MODES:
//
// MANUAL BLE:
// F = Forward
// B = Reverse
// L = Left
// R = Right
// S = Stop
//
// MSPEED 15 = Manual speed
// TURN 15   = Manual turning strength
//
// AUTO:
// AUTO or BF = Hallway navigation
//
// FRAME IS MOUNTED BACKWARDS:
// Old reverse is now physical forward.
// Turning logic is also flipped.
// =====================================================


// =====================================================
// MOTOR PINS
// =====================================================

// LEFT MOTOR
const int LEFT_PWM_PIN   = 4;
const int LEFT_DIR_PIN   = 5;
const int LEFT_BRAKE_PIN = 6;

// RIGHT MOTOR
const int RIGHT_PWM_PIN   = 7;
const int RIGHT_DIR_PIN   = 15;
const int RIGHT_BRAKE_PIN = 16;


// =====================================================
// ULTRASONIC SENSOR PINS
// =====================================================

// LEFT SENSOR
const int LEFT_TRIG_PIN = 17;
const int LEFT_ECHO_PIN = 18;

// RIGHT SENSOR
const int RIGHT_TRIG_PIN = 36;
const int RIGHT_ECHO_PIN = 37;


// =====================================================
// PWM CONFIGURATION
// =====================================================

const int LEFT_PWM_CHANNEL  = 0;
const int RIGHT_PWM_CHANNEL = 1;

const int PWM_FREQUENCY  = 10000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX        = 255;


// =====================================================
// MOTOR DIRECTIONS
// =====================================================
//
// FRAME IS BACKWARDS.
//
// Previous reverse is now physical forward.
// =====================================================

const int LEFT_FORWARD_LEVEL  = LOW;
const int RIGHT_FORWARD_LEVEL = HIGH;


// =====================================================
// SPEED SETTINGS
// =====================================================

// Maximum allowed motor speed.
const int MAX_NAV_SPEED = 20;


// AUTO cruise speed.
int baseSpeedPercent = 10;


// Manual BLE speed.
int manualSpeedPercent = 15;


// =====================================================
// MANUAL TURN STRENGTH
// =====================================================
//
// TURN 0
// Almost no steering difference.
//
// TURN 10
// Very small steering correction.
//
// TURN 20
// Gentle turn.
//
// TURN 40
// Medium turn.
//
// TURN 60
// Strong turn.
//
// TURN 100
// Inside wheel stops.
//
// Start at 15 for smooth hallway adjustments.
// =====================================================

int manualTurnPercent = 15;


// =====================================================
// AUTO STEERING SETTINGS
// =====================================================

float centerDeadbandCm = 5.0;

float steeringGain = 0.60;

int maxSteeringCorrection = 20;


// =====================================================
// AUTO FRONT OBSTACLE SETTINGS
// =====================================================

// Both sensors must be below this value
// before AUTO begins slowing.

float frontSlowDistanceCm = 45.0;


// Stop when both sensors indicate an object
// approximately this close.

float frontStopDistanceCm = 20.0;


// =====================================================
// SENSOR DATA
// =====================================================

float leftDistanceCm  = -1.0;
float rightDistanceCm = -1.0;


// =====================================================
// SENSOR FILTERING
// =====================================================

const float FILTER_ALPHA = 0.35;


// Sensor refresh rate.
const unsigned long SENSOR_INTERVAL_MS = 120;

unsigned long lastSensorUpdate = 0;


// If a sensor gets no valid measurement for this
// long, mark it invalid.

const unsigned long SENSOR_STALE_MS = 1000;

unsigned long lastLeftValidMs  = 0;
unsigned long lastRightValidMs = 0;


// =====================================================
// DEBUG TIMER
// =====================================================

const unsigned long DEBUG_INTERVAL_MS = 500;

unsigned long lastDebugPrint = 0;


// =====================================================
// BLE TELEMETRY TIMER
// =====================================================

const unsigned long BLE_TELEMETRY_INTERVAL_MS = 500;

unsigned long lastBleTelemetry = 0;


// =====================================================
// MANUAL BLE DEADMAN TIMER
// =====================================================
//
// If manual BLE driving is active and the ESP32
// receives no new manual command for 5 seconds,
// stop the hoverboard.
//
// KEEP can refresh this timer.
// =====================================================

const unsigned long MANUAL_TIMEOUT_MS = 50000;

unsigned long lastManualCommand = 0;


// =====================================================
// OVERALL SOFTWARE RUN LIMIT
// =====================================================

const unsigned long RUN_LIMIT_MS = 300000;

unsigned long lastDriveStart = 0;


// =====================================================
// DRIVE MODES
// =====================================================

enum DriveMode
{
    MODE_STOPPED,

    MODE_AUTO,

    MODE_MANUAL_FORWARD,

    MODE_MANUAL_REVERSE,

    MODE_MANUAL_LEFT,

    MODE_MANUAL_RIGHT
};


DriveMode driveMode =
    MODE_STOPPED;


// =====================================================
// CONTROL SOURCE
// =====================================================

enum CommandSource
{
    SOURCE_NONE,

    SOURCE_SERIAL,

    SOURCE_BLE
};


CommandSource activeControlSource =
    SOURCE_NONE;


// =====================================================
// CURRENT MOTOR OUTPUTS
// =====================================================

int leftAppliedSpeed  = 0;
int rightAppliedSpeed = 0;

int currentForwardBase = 0;

float currentSteeringCorrection = 0.0;


// =====================================================
// BLE UART SETTINGS
// =====================================================

#define BLE_DEVICE_NAME \
"ECE493-Hoverboard"


#define BLE_SERVICE_UUID \
"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"


// nRF Connect -> ESP32
#define BLE_RX_UUID \
"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"


// ESP32 -> nRF Connect
#define BLE_TX_UUID \
"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


BLEServer *bleServer =
    nullptr;


BLECharacteristic *bleTxCharacteristic =
    nullptr;


volatile bool bleConnected =
    false;


volatile bool bleDisconnectEvent =
    false;


// =====================================================
// BLE COMMAND BUFFER
// =====================================================

portMUX_TYPE bleCommandMux =
    portMUX_INITIALIZER_UNLOCKED;


char blePendingCommand[81] =
    {0};


volatile bool bleCommandReady =
    false;


// =====================================================
// USB SERIAL COMMAND BUFFER
// =====================================================

String serialCommandBuffer = "";


// =====================================================
// PWM HELPER
// =====================================================

int percentToPWM(
    int percent
)
{
    percent =
        constrain(
            percent,
            0,
            100
        );

    return
        (percent * PWM_MAX) / 100;
}


// =====================================================
// READ ULTRASONIC DISTANCE
// =====================================================

float readDistanceCm(
    int trigPin,
    int echoPin
)
{
    digitalWrite(
        trigPin,
        LOW
    );

    delayMicroseconds(2);


    digitalWrite(
        trigPin,
        HIGH
    );

    delayMicroseconds(10);


    digitalWrite(
        trigPin,
        LOW
    );


    unsigned long duration =
        pulseIn(
            echoPin,
            HIGH,
            30000
        );


    if (
        duration == 0
    )
    {
        return -1.0;
    }


    float distance =
        (duration * 0.0343) / 2.0;


    if (
        distance < 2.0 ||
        distance > 400.0
    )
    {
        return -1.0;
    }


    return distance;
}


// =====================================================
// FILTER SENSOR DISTANCE
// =====================================================

float filterDistance(
    float oldValue,
    float newValue
)
{
    // Keep last good value on temporary missed echo.

    if (
        newValue < 0
    )
    {
        return oldValue;
    }


    if (
        oldValue < 0
    )
    {
        return newValue;
    }


    return

        (FILTER_ALPHA * newValue)

        +

        (
            (1.0 - FILTER_ALPHA)
            *
            oldValue
        );
}


// =====================================================
// READ BOTH ULTRASONIC SENSORS
// =====================================================

void readBothSensors()
{
    unsigned long now =
        millis();


    // =================================================
    // LEFT
    // =================================================

    float newLeft =
        readDistanceCm(
            LEFT_TRIG_PIN,
            LEFT_ECHO_PIN
        );


    if (
        newLeft > 0
    )
    {
        leftDistanceCm =
            filterDistance(
                leftDistanceCm,
                newLeft
            );


        lastLeftValidMs =
            now;
    }

    else if (
        lastLeftValidMs == 0 ||

        now -
        lastLeftValidMs >
        SENSOR_STALE_MS
    )
    {
        leftDistanceCm =
            -1.0;
    }


    // Reduce ultrasonic cross-talk.

    delay(40);


    // =================================================
    // RIGHT
    // =================================================

    float newRight =
        readDistanceCm(
            RIGHT_TRIG_PIN,
            RIGHT_ECHO_PIN
        );


    if (
        newRight > 0
    )
    {
        rightDistanceCm =
            filterDistance(
                rightDistanceCm,
                newRight
            );


        lastRightValidMs =
            millis();
    }

    else if (
        lastRightValidMs == 0 ||

        millis() -
        lastRightValidMs >
        SENSOR_STALE_MS
    )
    {
        rightDistanceCm =
            -1.0;
    }
}


// =====================================================
// STOP LEFT MOTOR
// =====================================================

void stopLeftMotor()
{
    ledcWrite(
        LEFT_PWM_CHANNEL,
        0
    );


    digitalWrite(
        LEFT_BRAKE_PIN,
        HIGH
    );


    leftAppliedSpeed =
        0;
}


// =====================================================
// STOP RIGHT MOTOR
// =====================================================

void stopRightMotor()
{
    ledcWrite(
        RIGHT_PWM_CHANNEL,
        0
    );


    digitalWrite(
        RIGHT_BRAKE_PIN,
        HIGH
    );


    rightAppliedSpeed =
        0;
}


// =====================================================
// STOP BOTH MOTORS
// =====================================================

void stopAll()
{
    stopLeftMotor();

    stopRightMotor();


    driveMode =
        MODE_STOPPED;


    activeControlSource =
        SOURCE_NONE;


    currentForwardBase =
        0;


    currentSteeringCorrection =
        0.0;
}


// =====================================================
// LEFT PHYSICAL FORWARD
// =====================================================

void setLeftForward(
    int speedPercent
)
{
    speedPercent =
        constrain(
            speedPercent,
            0,
            MAX_NAV_SPEED
        );


    if (
        speedPercent <= 0
    )
    {
        stopLeftMotor();

        return;
    }


    digitalWrite(
        LEFT_DIR_PIN,
        LEFT_FORWARD_LEVEL
    );


    digitalWrite(
        LEFT_BRAKE_PIN,
        LOW
    );


    ledcWrite(
        LEFT_PWM_CHANNEL,
        percentToPWM(
            speedPercent
        )
    );


    leftAppliedSpeed =
        speedPercent;
}


// =====================================================
// RIGHT PHYSICAL FORWARD
// =====================================================

void setRightForward(
    int speedPercent
)
{
    speedPercent =
        constrain(
            speedPercent,
            0,
            MAX_NAV_SPEED
        );


    if (
        speedPercent <= 0
    )
    {
        stopRightMotor();

        return;
    }


    digitalWrite(
        RIGHT_DIR_PIN,
        RIGHT_FORWARD_LEVEL
    );


    digitalWrite(
        RIGHT_BRAKE_PIN,
        LOW
    );


    ledcWrite(
        RIGHT_PWM_CHANNEL,
        percentToPWM(
            speedPercent
        )
    );


    rightAppliedSpeed =
        speedPercent;
}


// =====================================================
// LEFT PHYSICAL REVERSE
// =====================================================

void setLeftReverse(
    int speedPercent
)
{
    speedPercent =
        constrain(
            speedPercent,
            0,
            MAX_NAV_SPEED
        );


    if (
        speedPercent <= 0
    )
    {
        stopLeftMotor();

        return;
    }


    digitalWrite(
        LEFT_DIR_PIN,
        !LEFT_FORWARD_LEVEL
    );


    digitalWrite(
        LEFT_BRAKE_PIN,
        LOW
    );


    ledcWrite(
        LEFT_PWM_CHANNEL,
        percentToPWM(
            speedPercent
        )
    );


    leftAppliedSpeed =
        speedPercent;
}


// =====================================================
// RIGHT PHYSICAL REVERSE
// =====================================================

void setRightReverse(
    int speedPercent
)
{
    speedPercent =
        constrain(
            speedPercent,
            0,
            MAX_NAV_SPEED
        );


    if (
        speedPercent <= 0
    )
    {
        stopRightMotor();

        return;
    }


    digitalWrite(
        RIGHT_DIR_PIN,
        !RIGHT_FORWARD_LEVEL
    );


    digitalWrite(
        RIGHT_BRAKE_PIN,
        LOW
    );


    ledcWrite(
        RIGHT_PWM_CHANNEL,
        percentToPWM(
            speedPercent
        )
    );


    rightAppliedSpeed =
        speedPercent;
}


// =====================================================
// MANUAL MODE HELPERS
// =====================================================

bool isManualForwardMode()
{
    return

        driveMode ==
            MODE_MANUAL_FORWARD

        ||

        driveMode ==
            MODE_MANUAL_LEFT

        ||

        driveMode ==
            MODE_MANUAL_RIGHT;
}


bool isAnyManualMode()
{
    return

        isManualForwardMode()

        ||

        driveMode ==
            MODE_MANUAL_REVERSE;
}


// =====================================================
// MANUAL INSIDE-WHEEL SPEED
// =====================================================

int getManualInsideWheelSpeed()
{
    float ratio =

        (
            100.0 -
            manualTurnPercent
        )

        / 100.0;


    int insideSpeed =

        (int)round(

            manualSpeedPercent *
            ratio
        );


    return

        constrain(
            insideSpeed,
            0,
            manualSpeedPercent
        );
}


// =====================================================
// MANUAL FORWARD
// =====================================================
//
// F -> L -> R -> F can all transition immediately.
//
// No braking or delay unless changing from reverse.
// =====================================================

void manualForward(
    CommandSource source
)
{
    // Only delay if changing electrical direction.

    if (
        driveMode ==
        MODE_MANUAL_REVERSE
    )
    {
        stopLeftMotor();

        stopRightMotor();


        delay(200);
    }


    driveMode =
        MODE_MANUAL_FORWARD;


    activeControlSource =
        source;


    lastManualCommand =
        millis();


    lastDriveStart =
        millis();


    setLeftForward(
        manualSpeedPercent
    );


    setRightForward(
        manualSpeedPercent
    );


    Serial.print(
        "MANUAL F | L="
    );

    Serial.print(
        leftAppliedSpeed
    );

    Serial.print(
        "% R="
    );

    Serial.print(
        rightAppliedSpeed
    );

    Serial.println("%");
}


// =====================================================
// MANUAL LEFT
// =====================================================
//
// FLIPPED TURNING LOGIC:
//
// LEFT TURN:
//
// LEFT wheel  = full speed
// RIGHT wheel = reduced speed
//
// Transition from F/R/L is immediate.
// =====================================================

void manualLeft(
    CommandSource source
)
{
    if (
        driveMode ==
        MODE_MANUAL_REVERSE
    )
    {
        stopLeftMotor();

        stopRightMotor();


        delay(200);
    }


    driveMode =
        MODE_MANUAL_LEFT;


    activeControlSource =
        source;


    lastManualCommand =
        millis();


    lastDriveStart =
        millis();


    int insideSpeed =
        getManualInsideWheelSpeed();


    // FLIPPED LEFT TURN

    setLeftForward(
        manualSpeedPercent
    );


    setRightForward(
        insideSpeed
    );


    Serial.print(
        "MANUAL L | L="
    );

    Serial.print(
        leftAppliedSpeed
    );

    Serial.print(
        "% R="
    );

    Serial.print(
        rightAppliedSpeed
    );

    Serial.print(
        "% TURN="
    );

    Serial.print(
        manualTurnPercent
    );

    Serial.println("%");
}


// =====================================================
// MANUAL RIGHT
// =====================================================
//
// FLIPPED TURNING LOGIC:
//
// RIGHT TURN:
//
// LEFT wheel  = reduced speed
// RIGHT wheel = full speed
// =====================================================

void manualRight(
    CommandSource source
)
{
    if (
        driveMode ==
        MODE_MANUAL_REVERSE
    )
    {
        stopLeftMotor();

        stopRightMotor();


        delay(200);
    }


    driveMode =
        MODE_MANUAL_RIGHT;


    activeControlSource =
        source;


    lastManualCommand =
        millis();


    lastDriveStart =
        millis();


    int insideSpeed =
        getManualInsideWheelSpeed();


    // FLIPPED RIGHT TURN

    setLeftForward(
        insideSpeed
    );


    setRightForward(
        manualSpeedPercent
    );


    Serial.print(
        "MANUAL R | L="
    );

    Serial.print(
        leftAppliedSpeed
    );

    Serial.print(
        "% R="
    );

    Serial.print(
        rightAppliedSpeed
    );

    Serial.print(
        "% TURN="
    );

    Serial.print(
        manualTurnPercent
    );

    Serial.println("%");
}


// =====================================================
// MANUAL REVERSE
// =====================================================

void manualReverse(
    CommandSource source
)
{
    // Forward -> reverse requires stop before
    // changing motor electrical direction.

    if (
        driveMode !=
        MODE_MANUAL_REVERSE
    )
    {
        stopLeftMotor();

        stopRightMotor();


        delay(200);
    }


    driveMode =
        MODE_MANUAL_REVERSE;


    activeControlSource =
        source;


    lastManualCommand =
        millis();


    lastDriveStart =
        millis();


    setLeftReverse(
        manualSpeedPercent
    );


    setRightReverse(
        manualSpeedPercent
    );


    Serial.print(
        "MANUAL B | L="
    );

    Serial.print(
        leftAppliedSpeed
    );

    Serial.print(
        "% R="
    );

    Serial.print(
        rightAppliedSpeed
    );

    Serial.println("%");
}


// =====================================================
// REAPPLY CURRENT MANUAL COMMAND
// =====================================================
//
// Used when MSPEED changes while driving.
// =====================================================

void reapplyManualCommand(
    CommandSource source
)
{
    switch (
        driveMode
    )
    {
        case MODE_MANUAL_FORWARD:

            manualForward(
                source
            );

            break;


        case MODE_MANUAL_LEFT:

            manualLeft(
                source
            );

            break;


        case MODE_MANUAL_RIGHT:

            manualRight(
                source
            );

            break;


        case MODE_MANUAL_REVERSE:

            manualReverse(
                source
            );

            break;


        default:

            break;
    }
}


// =====================================================
// AUTOMATIC HALLWAY NAVIGATION
// =====================================================

void calculateAutoNavigation()
{
    if (
        driveMode !=
        MODE_AUTO
    )
    {
        return;
    }


    // =================================================
    // BOTH SENSORS INVALID
    // =================================================

    if (
        leftDistanceCm < 0
        &&
        rightDistanceCm < 0
    )
    {
        stopLeftMotor();

        stopRightMotor();


        currentForwardBase =
            0;


        currentSteeringCorrection =
            0.0;


        return;
    }


    // =================================================
    // ONE SENSOR TEMPORARILY INVALID
    // =================================================
    //
    // Treat missing side as open space.
    // =================================================

    float leftNavDistance =

        leftDistanceCm > 0

        ?

        leftDistanceCm

        :

        200.0;


    float rightNavDistance =

        rightDistanceCm > 0

        ?

        rightDistanceCm

        :

        200.0;


    // =================================================
    // NORMAL AUTO SPEED
    // =================================================

    int forwardBase =
        baseSpeedPercent;


    // =================================================
    // FRONT OBSTACLE DETECTION
    // =================================================

    bool obstacleAhead =

        leftNavDistance <
        frontSlowDistanceCm

        &&

        rightNavDistance <
        frontSlowDistanceCm;


    if (
        obstacleAhead
    )
    {
        float averageDistance =

            (
                leftNavDistance +
                rightNavDistance
            )

            / 2.0;


        // =============================================
        // STOP
        // =============================================

        if (
            averageDistance <=
            frontStopDistanceCm
        )
        {
            stopLeftMotor();

            stopRightMotor();


            currentForwardBase =
                0;


            currentSteeringCorrection =
                0.0;


            return;
        }


        // =============================================
        // GRADUAL FRONT SLOWDOWN
        // =============================================

        float slowdownRatio =

            (
                averageDistance -
                frontStopDistanceCm
            )

            /

            (
                frontSlowDistanceCm -
                frontStopDistanceCm
            );


        slowdownRatio =
            constrain(
                slowdownRatio,
                0.0f,
                1.0f
            );


        forwardBase =

            (int)round(

                baseSpeedPercent *
                slowdownRatio
            );
    }


    currentForwardBase =
        forwardBase;


    // =================================================
    // FLIPPED AUTO TURNING LOGIC
    // =================================================
    //
    // LEFT wall/object closer:
    //
    // leftDistance < rightDistance
    //
    // Error becomes negative.
    //
    // Left wheel slows.
    // Right wheel speeds up.
    //
    // Physical result = RIGHT TURN.
    //
    //
    // RIGHT wall/object closer:
    //
    // Error becomes positive.
    //
    // Left wheel speeds up.
    // Right wheel slows.
    //
    // Physical result = LEFT TURN.
    // =================================================

    float distanceError =

        leftNavDistance -
        rightNavDistance;


    // =================================================
    // CENTER DEADBAND
    // =================================================

    if (
        fabs(
            distanceError
        )
        <=
        centerDeadbandCm
    )
    {
        distanceError =
            0.0;
    }

    else if (
        distanceError > 0
    )
    {
        distanceError -=
            centerDeadbandCm;
    }

    else
    {
        distanceError +=
            centerDeadbandCm;
    }


    // =================================================
    // PROPORTIONAL STEERING
    // =================================================

    float correction =

        distanceError *
        steeringGain;


    correction =
        constrain(
            correction,
            -(float)maxSteeringCorrection,
            (float)maxSteeringCorrection
        );


    // Reduce aggressive steering while slowing
    // for a front obstacle.

    if (
        obstacleAhead
    )
    {
        float allowedCorrection =

            forwardBase *
            0.75;


        correction =
            constrain(
                correction,
                -allowedCorrection,
                allowedCorrection
            );
    }


    currentSteeringCorrection =
        correction;


    // =================================================
    // DIFFERENTIAL MOTOR SPEED
    // =================================================

    int leftSpeed =

        (int)round(

            forwardBase +
            correction
        );


    int rightSpeed =

        (int)round(

            forwardBase -
            correction
        );


    leftSpeed =
        constrain(
            leftSpeed,
            0,
            MAX_NAV_SPEED
        );


    rightSpeed =
        constrain(
            rightSpeed,
            0,
            MAX_NAV_SPEED
        );


    // =================================================
    // APPLY SPEEDS
    // =================================================

    setLeftForward(
        leftSpeed
    );


    setRightForward(
        rightSpeed
    );
}


// =====================================================
// START AUTO MODE
// =====================================================

void startAuto(
    CommandSource source
)
{
    stopLeftMotor();

    stopRightMotor();


    delay(200);


    driveMode =
        MODE_AUTO;


    activeControlSource =
        source;


    lastDriveStart =
        millis();


    readBothSensors();


    calculateAutoNavigation();


    Serial.println(
        "AUTO HALLWAY NAVIGATION STARTED"
    );
}


// =====================================================
// MODE NAME
// =====================================================

const char *modeName()
{
    switch (
        driveMode
    )
    {
        case MODE_AUTO:

            return "AUTO";


        case MODE_MANUAL_FORWARD:

            return "MANUAL FORWARD";


        case MODE_MANUAL_REVERSE:

            return "MANUAL REVERSE";


        case MODE_MANUAL_LEFT:

            return "MANUAL LEFT";


        case MODE_MANUAL_RIGHT:

            return "MANUAL RIGHT";


        default:

            return "STOPPED";
    }
}


// Short BLE mode code.

const char *modeCode()
{
    switch (
        driveMode
    )
    {
        case MODE_AUTO:

            return "AUTO";


        case MODE_MANUAL_FORWARD:

            return "F";


        case MODE_MANUAL_REVERSE:

            return "B";


        case MODE_MANUAL_LEFT:

            return "L";


        case MODE_MANUAL_RIGHT:

            return "R";


        default:

            return "S";
    }
}


// =====================================================
// BLE SEND TEXT
// =====================================================

void sendBleText(
    const String &message
)
{
    if (
        !bleConnected ||
        bleTxCharacteristic ==
        nullptr
    )
    {
        return;
    }


    bleTxCharacteristic->
        setValue(
            (uint8_t *)
            message.c_str(),

            message.length()
        );


    bleTxCharacteristic->
        notify();
}


// =====================================================
// BLE TELEMETRY
// =====================================================
//
// Example:
//
// L72 R68 X15 Y13
//
// L = left distance
// R = right distance
// X = left motor %
// Y = right motor %
// =====================================================

void sendBleTelemetry()
{
    if (
        !bleConnected
    )
    {
        return;
    }


    int leftCm =

        leftDistanceCm < 0

        ?

        -1

        :

        (int)round(
            leftDistanceCm
        );


    int rightCm =

        rightDistanceCm < 0

        ?

        -1

        :

        (int)round(
            rightDistanceCm
        );


    String message =
        "L";


    message +=
        String(
            leftCm
        );


    message +=
        " R";


    message +=
        String(
            rightCm
        );


    message +=
        " X";


    message +=
        String(
            leftAppliedSpeed
        );


    message +=
        " Y";


    message +=
        String(
            rightAppliedSpeed
        );


    sendBleText(
        message
    );
}


// =====================================================
// PRINT DISTANCES
// =====================================================

void printDistances()
{
    readBothSensors();


    Serial.println();

    Serial.println(
        "------ SENSOR DATA ------"
    );


    Serial.print(
        "LEFT:  "
    );


    if (
        leftDistanceCm < 0
    )
    {
        Serial.println(
            "INVALID"
        );
    }

    else
    {
        Serial.print(
            leftDistanceCm,
            1
        );

        Serial.println(
            " cm"
        );
    }


    Serial.print(
        "RIGHT: "
    );


    if (
        rightDistanceCm < 0
    )
    {
        Serial.println(
            "INVALID"
        );
    }

    else
    {
        Serial.print(
            rightDistanceCm,
            1
        );

        Serial.println(
            " cm"
        );
    }


    Serial.println(
        "-------------------------"
    );
}


// =====================================================
// STATUS
// =====================================================

void printStatus()
{
    Serial.println();

    Serial.println(
        "========== STATUS =========="
    );


    Serial.print(
        "MODE: "
    );

    Serial.println(
        modeName()
    );


    Serial.print(
        "BLE: "
    );

    Serial.println(

        bleConnected

        ?

        "CONNECTED"

        :

        "DISCONNECTED"
    );


    Serial.print(
        "MANUAL SPEED: "
    );

    Serial.print(
        manualSpeedPercent
    );

    Serial.println("%");


    Serial.print(
        "MANUAL TURN: "
    );

    Serial.print(
        manualTurnPercent
    );

    Serial.println("%");


    Serial.print(
        "AUTO BASE: "
    );

    Serial.print(
        baseSpeedPercent
    );

    Serial.println("%");


    Serial.print(
        "AUTO TURN CORRECTION: "
    );

    Serial.print(
        currentSteeringCorrection,
        1
    );

    Serial.println("%");


    Serial.print(
        "LEFT DISTANCE: "
    );

    if (
        leftDistanceCm < 0
    )
    {
        Serial.println(
            "INVALID"
        );
    }

    else
    {
        Serial.print(
            leftDistanceCm,
            1
        );

        Serial.println(
            " cm"
        );
    }


    Serial.print(
        "RIGHT DISTANCE: "
    );

    if (
        rightDistanceCm < 0
    )
    {
        Serial.println(
            "INVALID"
        );
    }

    else
    {
        Serial.print(
            rightDistanceCm,
            1
        );

        Serial.println(
            " cm"
        );
    }


    Serial.print(
        "LEFT MOTOR: "
    );

    Serial.print(
        leftAppliedSpeed
    );

    Serial.println("%");


    Serial.print(
        "RIGHT MOTOR: "
    );

    Serial.print(
        rightAppliedSpeed
    );

    Serial.println("%");


    Serial.println(
        "============================"
    );
}


// =====================================================
// HELP
// =====================================================

void printHelp()
{
    Serial.println();

    Serial.println(
        "========== COMMANDS =========="
    );


    Serial.println(
        "MANUAL:"
    );

    Serial.println(
        "F              Forward"
    );

    Serial.println(
        "B              Reverse"
    );

    Serial.println(
        "L              Turn left"
    );

    Serial.println(
        "R              Turn right"
    );

    Serial.println(
        "S              Stop"
    );

    Serial.println(
        "MSPEED 15      Manual speed 0-60%"
    );

    Serial.println(
        "TURN 15        Turn strength 0-100%"
    );

    Serial.println(
        "KEEP           Refresh BLE timeout"
    );


    Serial.println();


    Serial.println(
        "AUTO:"
    );

    Serial.println(
        "AUTO or BF     Start hallway navigation"
    );

    Serial.println(
        "BASE 20        Auto cruise speed"
    );

    Serial.println(
        "GAIN 0.60      Auto steering gain"
    );

    Serial.println(
        "MAXTURN 20     Auto max correction"
    );

    Serial.println(
        "DEADBAND 5     Center tolerance"
    );

    Serial.println(
        "SLOWDIST 45    Begin front slowdown"
    );

    Serial.println(
        "STOPDIST 20    Front stop distance"
    );


    Serial.println();


    Serial.println(
        "DIST           Sensor readings"
    );

    Serial.println(
        "STATUS         Current status"
    );

    Serial.println(
        "HELP           Command list"
    );


    Serial.println(
        "=============================="
    );
}


// =====================================================
// BLE SERVER CALLBACKS
// =====================================================

class HoverboardServerCallbacks
    :
    public BLEServerCallbacks
{
    void onConnect(
        BLEServer *server
    ) override
    {
        bleConnected =
            true;


        Serial.println(
            "BLE CONNECTED"
        );
    }


    void onDisconnect(
        BLEServer *server
    ) override
    {
        bleConnected =
            false;


        bleDisconnectEvent =
            true;


        Serial.println(
            "BLE DISCONNECTED"
        );
    }
};


// =====================================================
// BLE RX CALLBACK
// =====================================================

class HoverboardRxCallbacks
    :
    public BLECharacteristicCallbacks
{
    void onWrite(
        BLECharacteristic *characteristic
    ) override
    {
        std::string value =
            characteristic->
            getValue();


        if (
            value.empty()
        )
        {
            return;
        }


        size_t length =
            value.length();


        if (
            length >
            sizeof(
                blePendingCommand
            ) - 1
        )
        {
            length =
                sizeof(
                    blePendingCommand
                ) - 1;
        }


        portENTER_CRITICAL(
            &bleCommandMux
        );


        memcpy(
            blePendingCommand,
            value.data(),
            length
        );


        blePendingCommand[
            length
        ] =
            '\0';


        bleCommandReady =
            true;


        portEXIT_CRITICAL(
            &bleCommandMux
        );
    }
};


// =====================================================
// BLE SETUP
// =====================================================

void setupBLE()
{
    Serial.println(
        "Starting BLE..."
    );


    BLEDevice::init(
        BLE_DEVICE_NAME
    );


    bleServer =
        BLEDevice::createServer();


    bleServer->setCallbacks(
        new HoverboardServerCallbacks()
    );


    BLEService *service =

        bleServer->createService(
            BLE_SERVICE_UUID
        );


    // =================================================
    // TX
    // ESP32 -> nRF Connect
    // =================================================

    bleTxCharacteristic =

        service->createCharacteristic(

            BLE_TX_UUID,

            BLECharacteristic::PROPERTY_NOTIFY
        );


    bleTxCharacteristic->
        addDescriptor(
            new BLE2902()
        );


    // =================================================
    // RX
    // nRF Connect -> ESP32
    // =================================================

    BLECharacteristic *rxCharacteristic =

        service->createCharacteristic(

            BLE_RX_UUID,

            BLECharacteristic::PROPERTY_WRITE

            |

            BLECharacteristic::PROPERTY_WRITE_NR
        );


    rxCharacteristic->setCallbacks(
        new HoverboardRxCallbacks()
    );


    service->start();


    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();


    advertising->addServiceUUID(
        BLE_SERVICE_UUID
    );


    advertising->setScanResponse(
        true
    );


    advertising->setMinPreferred(
        0x06
    );


    advertising->setMaxPreferred(
        0x12
    );


    BLEDevice::startAdvertising();


    Serial.println(
        "BLE READY: ECE493-Hoverboard"
    );
}


// =====================================================
// COMMAND PROCESSOR
// =====================================================

void processCommand(
    String command,
    CommandSource source
)
{
    command.trim();

    command.toUpperCase();


    if (
        command.length() == 0
    )
    {
        return;
    }


    // =================================================
    // STOP
    // =================================================

    if (
        command == "S" ||
        command == "STOP"
    )
    {
        stopAll();


        sendBleText(
            "ACK S"
        );


        return;
    }


    // =================================================
    // MANUAL FORWARD
    // =================================================

    if (
        command == "F"
    )
    {
        manualForward(
            source
        );


        sendBleText(
            "ACK F"
        );


        return;
    }


    // =================================================
    // MANUAL REVERSE
    // =================================================

    if (
        command == "B" ||
        command == "BR"
    )
    {
        manualReverse(
            source
        );


        sendBleText(
            "ACK B"
        );


        return;
    }


    // =================================================
    // MANUAL LEFT
    // =================================================

    if (
        command == "L"
    )
    {
        manualLeft(
            source
        );


        sendBleText(
            "ACK L"
        );


        return;
    }


    // =================================================
    // MANUAL RIGHT
    // =================================================

    if (
        command == "R"
    )
    {
        manualRight(
            source
        );


        sendBleText(
            "ACK R"
        );


        return;
    }


    // =================================================
    // KEEP MANUAL BLE ACTIVE
    // =================================================

    if (
        command == "KEEP"
    )
    {
        if (
            isAnyManualMode()
        )
        {
            lastManualCommand =
                millis();


            sendBleText(
                "ACK KEEP"
            );
        }


        return;
    }


    // =================================================
    // AUTO MODE
    // =================================================

    if (
        command == "AUTO" ||
        command == "BF"
    )
    {
        startAuto(
            source
        );


        sendBleText(
            "ACK AUTO"
        );


        return;
    }


    // =================================================
    // MANUAL SPEED
    // =================================================

    if (
        command.startsWith(
            "MSPEED "
        )
    )
    {
        int value =

            command
            .substring(7)
            .toInt();


        if (
            value < 0 ||
            value > MAX_NAV_SPEED
        )
        {
            sendBleText(
                "ERR MSPEED"
            );


            return;
        }


        manualSpeedPercent =
            value;


        lastManualCommand =
            millis();


        // If currently driving manually,
        // immediately apply the new speed.

        if (
            isAnyManualMode()
        )
        {
            reapplyManualCommand(
                source
            );
        }


        String response =
            "MSPEED ";


        response +=
            String(
                manualSpeedPercent
            );


        sendBleText(
            response
        );


        Serial.print(
            "MANUAL SPEED = "
        );

        Serial.print(
            manualSpeedPercent
        );

        Serial.println("%");


        return;
    }


    // =================================================
    // MANUAL TURN STRENGTH
    // =================================================

    if (
        command.startsWith(
            "TURN "
        )
    )
    {
        int value =

            command
            .substring(5)
            .toInt();


        if (
            value < 0 ||
            value > 100
        )
        {
            sendBleText(
                "ERR TURN"
            );


            return;
        }


        manualTurnPercent =
            value;


        lastManualCommand =
            millis();


        // Immediately change turn strength
        // if already turning.

        if (
            driveMode ==
            MODE_MANUAL_LEFT
        )
        {
            manualLeft(
                source
            );
        }

        else if (
            driveMode ==
            MODE_MANUAL_RIGHT
        )
        {
            manualRight(
                source
            );
        }


        String response =
            "TURN ";


        response +=
            String(
                manualTurnPercent
            );


        sendBleText(
            response
        );


        Serial.print(
            "MANUAL TURN = "
        );

        Serial.print(
            manualTurnPercent
        );

        Serial.println("%");


        return;
    }


    // =================================================
    // AUTO BASE SPEED
    // =================================================

    if (
        command.startsWith(
            "BASE "
        )
    )
    {
        int value =

            command
            .substring(5)
            .toInt();


        if (
            value < 0 ||
            value > MAX_NAV_SPEED
        )
        {
            sendBleText(
                "ERR BASE"
            );


            return;
        }


        baseSpeedPercent =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        String response =
            "BASE ";


        response +=
            String(
                baseSpeedPercent
            );


        sendBleText(
            response
        );


        return;
    }


    // =================================================
    // PWM ALIAS FOR AUTO BASE
    // =================================================

    if (
        command.startsWith(
            "PWM "
        )
    )
    {
        int value =

            command
            .substring(4)
            .toInt();


        if (
            value < 0 ||
            value > MAX_NAV_SPEED
        )
        {
            sendBleText(
                "ERR PWM"
            );


            return;
        }


        baseSpeedPercent =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // AUTO STEERING GAIN
    // =================================================

    if (
        command.startsWith(
            "GAIN "
        )
    )
    {
        float value =

            command
            .substring(5)
            .toFloat();


        if (
            value < 0.05 ||
            value > 2.0
        )
        {
            sendBleText(
                "ERR GAIN"
            );


            return;
        }


        steeringGain =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // AUTO MAX TURN
    // =================================================

    if (
        command.startsWith(
            "MAXTURN "
        )
    )
    {
        int value =

            command
            .substring(8)
            .toInt();


        if (
            value < 0 ||
            value > 40
        )
        {
            sendBleText(
                "ERR MAXTURN"
            );


            return;
        }


        maxSteeringCorrection =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // AUTO CENTER DEADBAND
    // =================================================

    if (
        command.startsWith(
            "DEADBAND "
        )
    )
    {
        float value =

            command
            .substring(9)
            .toFloat();


        if (
            value < 0 ||
            value > 30
        )
        {
            sendBleText(
                "ERR DEADBAND"
            );


            return;
        }


        centerDeadbandCm =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // FRONT SLOW DISTANCE
    // =================================================

    if (
        command.startsWith(
            "SLOWDIST "
        )
    )
    {
        float value =

            command
            .substring(9)
            .toFloat();


        if (
            value <=
            frontStopDistanceCm

            ||

            value > 300
        )
        {
            sendBleText(
                "ERR SLOW"
            );


            return;
        }


        frontSlowDistanceCm =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // FRONT STOP DISTANCE
    // =================================================

    if (
        command.startsWith(
            "STOPDIST "
        )
    )
    {
        float value =

            command
            .substring(9)
            .toFloat();


        if (
            value < 5

            ||

            value >=
            frontSlowDistanceCm
        )
        {
            sendBleText(
                "ERR STOPD"
            );


            return;
        }


        frontStopDistanceCm =
            value;


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }


        return;
    }


    // =================================================
    // DISTANCE
    // =================================================

    if (
        command == "DIST"
    )
    {
        printDistances();


        sendBleTelemetry();


        return;
    }


    // =================================================
    // STATUS
    // =================================================

    if (
        command == "STATUS"
    )
    {
        printStatus();


        String response =
            "MODE=";


        response +=
            modeCode();


        sendBleText(
            response
        );


        return;
    }


    // =================================================
    // HELP
    // =================================================

    if (
        command == "HELP"
    )
    {
        printHelp();


        sendBleText(
            "ACK HELP"
        );


        return;
    }


    // =================================================
    // UNKNOWN COMMAND
    // =================================================

    sendBleText(
        "ERR CMD"
    );


    Serial.print(
        "Unknown command: "
    );


    Serial.println(
        command
    );
}


// =====================================================
// PROCESS QUEUED BLE COMMAND
// =====================================================

void processPendingBleCommand()
{
    if (
        !bleCommandReady
    )
    {
        return;
    }


    char localCommand[81];


    portENTER_CRITICAL(
        &bleCommandMux
    );


    strncpy(
        localCommand,
        blePendingCommand,
        sizeof(
            localCommand
        )
    );


    localCommand[
        sizeof(
            localCommand
        ) - 1
    ] =
        '\0';


    blePendingCommand[0] =
        '\0';


    bleCommandReady =
        false;


    portEXIT_CRITICAL(
        &bleCommandMux
    );


    String command =
        String(
            localCommand
        );


    command.trim();


    Serial.print(
        "BLE: "
    );


    Serial.println(
        command
    );


    processCommand(
        command,
        SOURCE_BLE
    );
}


// =====================================================
// USB SERIAL COMMANDS
// =====================================================

void readSerialCommands()
{
    while (
        Serial.available() > 0
    )
    {
        char c =
            Serial.read();


        if (
            c == '\r'
        )
        {
            continue;
        }


        if (
            c == '\n'
        )
        {
            processCommand(
                serialCommandBuffer,
                SOURCE_SERIAL
            );


            serialCommandBuffer =
                "";


            continue;
        }


        if (
            serialCommandBuffer.length()
            <
            80
        )
        {
            serialCommandBuffer +=
                c;
        }
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(300);


    // =================================================
    // ULTRASONIC SENSORS
    // =================================================

    pinMode(
        LEFT_TRIG_PIN,
        OUTPUT
    );


    pinMode(
        LEFT_ECHO_PIN,
        INPUT
    );


    pinMode(
        RIGHT_TRIG_PIN,
        OUTPUT
    );


    pinMode(
        RIGHT_ECHO_PIN,
        INPUT
    );


    digitalWrite(
        LEFT_TRIG_PIN,
        LOW
    );


    digitalWrite(
        RIGHT_TRIG_PIN,
        LOW
    );


    // =================================================
    // MOTOR BRAKES
    // =================================================

    pinMode(
        LEFT_BRAKE_PIN,
        OUTPUT
    );


    pinMode(
        RIGHT_BRAKE_PIN,
        OUTPUT
    );


    digitalWrite(
        LEFT_BRAKE_PIN,
        HIGH
    );


    digitalWrite(
        RIGHT_BRAKE_PIN,
        HIGH
    );


    // =================================================
    // MOTOR DIRECTIONS
    // =================================================

    pinMode(
        LEFT_DIR_PIN,
        OUTPUT
    );


    pinMode(
        RIGHT_DIR_PIN,
        OUTPUT
    );


    digitalWrite(
        LEFT_DIR_PIN,
        LEFT_FORWARD_LEVEL
    );


    digitalWrite(
        RIGHT_DIR_PIN,
        RIGHT_FORWARD_LEVEL
    );


    // =================================================
    // LEFT PWM
    // =================================================

    ledcSetup(
        LEFT_PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );


    ledcAttachPin(
        LEFT_PWM_PIN,
        LEFT_PWM_CHANNEL
    );


    // =================================================
    // RIGHT PWM
    // =================================================

    ledcSetup(
        RIGHT_PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );


    ledcAttachPin(
        RIGHT_PWM_PIN,
        RIGHT_PWM_CHANNEL
    );


    // Start stopped.

    stopAll();


    // Start BLE early so it advertises even when
    // running from the hoverboard without Mac USB.

    setupBLE();


    // Initial sensor readings.

    readBothSensors();


    Serial.println();


    Serial.println(
        "ECE493 HOVERBOARD READY"
    );


    Serial.println(
        "REVERSED FRAME DIRECTION ACTIVE"
    );


    Serial.println(
        "AUTO + BLE MANUAL CONTROL"
    );


    Serial.println(
        "F / B / L / R / S = MANUAL"
    );


    Serial.println(
        "TURN n = MANUAL TURN STRENGTH"
    );


    Serial.println(
        "AUTO = HALLWAY NAVIGATION"
    );


    Serial.println(
        "PHYSICAL E-STOP REQUIRED"
    );


    printHelp();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    // =================================================
    // BLE DISCONNECT FAILSAFE
    // =================================================

    if (
        bleDisconnectEvent
    )
    {
        bleDisconnectEvent =
            false;


        if (
            activeControlSource ==
            SOURCE_BLE

            &&

            driveMode !=
            MODE_STOPPED
        )
        {
            stopAll();


            Serial.println(
                "BLE DISCONNECT: MOTORS STOPPED"
            );
        }


        delay(100);


        if (
            bleServer !=
            nullptr
        )
        {
            bleServer->
                startAdvertising();
        }
    }


    // =================================================
    // BLE COMMANDS
    // =================================================

    processPendingBleCommand();


    // =================================================
    // USB SERIAL COMMANDS
    // =================================================

    readSerialCommands();


    // =================================================
    // MANUAL BLE DEADMAN TIMER
    // =================================================

    if (
        isAnyManualMode()

        &&

        activeControlSource ==
        SOURCE_BLE

        &&

        millis() -
        lastManualCommand >=
        MANUAL_TIMEOUT_MS
    )
    {
        stopAll();


        sendBleText(
            "MANUAL TIMEOUT"
        );


        Serial.println(
            "MANUAL BLE TIMEOUT: MOTORS STOPPED"
        );
    }


    // =================================================
    // SENSOR UPDATE
    // =================================================

    if (
        millis() -
        lastSensorUpdate >=
        SENSOR_INTERVAL_MS
    )
    {
        lastSensorUpdate =
            millis();


        readBothSensors();


        if (
            driveMode ==
            MODE_AUTO
        )
        {
            calculateAutoNavigation();
        }
    }


    // =================================================
    // AUTO DEBUG OUTPUT
    // =================================================

    if (
        driveMode ==
        MODE_AUTO

        &&

        millis() -
        lastDebugPrint >=
        DEBUG_INTERVAL_MS
    )
    {
        lastDebugPrint =
            millis();


        Serial.print(
            "L="
        );


        if (
            leftDistanceCm < 0
        )
        {
            Serial.print(
                "INVALID"
            );
        }

        else
        {
            Serial.print(
                leftDistanceCm,
                1
            );
        }


        Serial.print(
            " R="
        );


        if (
            rightDistanceCm < 0
        )
        {
            Serial.print(
                "INVALID"
            );
        }

        else
        {
            Serial.print(
                rightDistanceCm,
                1
            );
        }


        Serial.print(
            " | BASE="
        );


        Serial.print(
            currentForwardBase
        );


        Serial.print(
            "% TURN="
        );


        Serial.print(
            currentSteeringCorrection,
            1
        );


        Serial.print(
            "% | LM="
        );


        Serial.print(
            leftAppliedSpeed
        );


        Serial.print(
            "% RM="
        );


        Serial.print(
            rightAppliedSpeed
        );


        Serial.println("%");
    }


    // =================================================
    // BLE TELEMETRY
    // =================================================

    if (
        bleConnected

        &&

        millis() -
        lastBleTelemetry >=
        BLE_TELEMETRY_INTERVAL_MS
    )
    {
        lastBleTelemetry =
            millis();


        sendBleTelemetry();
    }


    // =================================================
    // 5-MINUTE RUN LIMIT
    // =================================================

    if (
        driveMode !=
        MODE_STOPPED

        &&

        millis() -
        lastDriveStart >=
        RUN_LIMIT_MS
    )
    {
        stopAll();


        sendBleText(
            "RUN TIMEOUT"
        );


        Serial.println(
            "AUTO STOP: 5-minute run limit reached."
        );
    }
}