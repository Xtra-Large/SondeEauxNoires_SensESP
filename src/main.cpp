// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/constant_sensor.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/hysteresis.h"
#include "sensesp/transforms/threshold.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/click_type.h"
#include "sensesp/transforms/time_counter.h"
#include "sensesp_app_builder.h"
#include "sensesp/ui/ui_button.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp/ui/status_page_item.h"


using namespace sensesp;

// Utilsiation du sensor SR04M pour connaitre le niveau de la cuve à eaux noires avec un relais pour le déclenchement de la pompe
// Et Bypass pour déclenchement manuel de la pompe via un interupteur
//Liens capteur : 
//    https://tutorials.probots.co.in/communicating-with-a-waterproof-ultrasonic-sensor-aj-sr04m-jsn-sr04t/
//    https://circuitdigest.com/microcontroller-projects/interfacing-ultrasonic-sensor-module-with-arduino 
//Le capteur est utilisé en mode serial basse consommation
//  2 pins pour le serial : TX=99 et RX=99
//  Le capteur mesure au minimum 20cm de hauteur
// On doit pouvoir configurer 
//    la hauteur min et max du réservoir
//    Le volume total du réservoir
//    Les % de début de vidage et de fin de vidage (https://signalk.org/SensESP/generated/docs/classsensesp_1_1_hysteresis.html)
// Le relai est activé sur le PIND=99
// Le capteur de marche forcée est sur le PIND=99
//    Même en mode forcé, la pompe doit etre arrêtée si le % est inférieur ou egal au % de fin de vidage afin d'eviter que la pompe ne tourne dans le vide
// On doit pouvoir aussi recevoir un ordre de marche forcée du serveur SignalK
// Un bouton activant ou non le mode automatique, ce qui permet de conserver la lecture du taux de remplissage sans pour autant activer la pompe (mode voyage)
//Le tout devant etre associé à un timer de securité permettant à la pompe d'avoir un temps de refroidissement
    //Sensor analogique

// See this link for available tank paths:
// https://signalk.org/specification/1.4.0/doc/vesselsBranch.html#vesselsregexptanks

#define BT_DESACTIVATED

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#elif !defined(BT_DESACTIVATED)
    #include "BluetoothSerial.h"
    #define BT_ACTIVATED
    BluetoothSerial SerialBT;
#endif

#ifdef CORE_DEBUG_LEVEL
    #if CORE_DEBUG_LEVEL > 3
        #define DEBUG_MODE
    #endif
#endif

#define ACTIVATE_SONAR 1
#define ACTIVATE_ANALOG 1

#define MIN_SENSOR_HEIGHT 100 //Minimum 20cm en mm
#define MILLIS_REPAT_SENSOR 1000
#define PIN_SONAR_RX 16
#define PIN_SONAR_TX 17

#define PIN_SONAR_TRIG PIN_SONAR_RX
#define PIN_SONAR_ECHO PIN_SONAR_TX

//1 - Traditional Trigger and Echo Mode (same as the popular HCSR04):
//2 - Low Power Traditional Trigger and Echo Mode
//3 - Automatic Serial Mode
//4 - Low Power Serial Mode
//5 - Computer Printing Mode
int SONAR_MODE = 1; 

#define PIN_RELAY 15
#define PIN_FORCE_START 14
#define PIN_AUTO_START 12
#define PIN_ANALOG 34

#define SEND_SK_SONAR_EXT_INFO true
#define SEND_SK_ANALOG_EXT_INFO true


//Gestion des états d'activation en marche forcée
bool forcePumpStatus = false;

//Gestion des états d'activation en mode automatique
bool autoPumpStatus = false;

//gestion des times de securité
long pumpStartTime = 0;
long pumpStopSecuTime = 0;

bool activateSonarSensor = true;
bool activateAnalogSensor =  true;

//Valeurs à afficher dans la page de statut
StatusPageItem<float> remplissageReservoir{
      "Remplissage (%)", 0, "Eaux noires" , 0};
StatusPageItem<String> pompeActivee{
      "Pompe activéé (oui/non)", "non", "Eaux noires" , 1};

float i = 0;
float sens = 0.5;
bool lastForcePumpStatus = forcePumpStatus;


//Valeurs de configuration
float confVolTotal = 225;
String confVolTotalPath = "/sensors/cuve_eaux_noires/volume_total";
float confPercentStartForced = 80.0F;
String confPercentStartForcedPath = "/sensors/cuve_eaux_noires/percent_start_forced";
float confPercentStart = 50.0F;
String confPercentStartPath = "/sensors/cuve_eaux_noires/percent_start";
float confPercentStop = 25.0F;
String confPercentStopPath = "/sensors/cuve_eaux_noires/percent_stop";

float maxTimeContinius = 5;
String maxTimeContiniusPath = "/sensors/cuve_eaux_noires/pump/max_time";

float pauseTime = 5;
String pauseTimePath = "/sensors/cuve_eaux_noires/pump/pause_time";

#if ACTIVATE_ANALOG == 1
    float confSensorAnalogMaxVal = 3000000;
    String confSensorAnalogMaxValPath = "/sensors/cuve_eaux_noires/analog/max_val";

    float confSensorAnalogMinVal = 1900000;
    String confSensorAnalogMinValPath = "/sensors/cuve_eaux_noires/analog/min_val";

    float confSensorAnalogPercentFullAtMinVal = 15;
    String confSensorAnalogPercentFullAtMinValPath = "/sensors/cuve_eaux_noires/analog/percent_at_min_val";

    StatusPageItem<float> analogValueStatusItem{
      "Valeur de sonde analogique (ohm)", 0.0, "Eaux noires" , 2};
#endif

#if ACTIVATE_SONAR == 1
    float confMaxHeightRead = 80;
    String confMaxHeightReadPath = "/sensors/cuve_eaux_noires/sonar/max_height";

    float confMinHeightRead = 20;
    String confMinHeightReadPath = "/sensors/cuve_eaux_noires/sonar/max_height";
#endif

float fakeReadSonar(){
    if(lastForcePumpStatus != forcePumpStatus){
        if(forcePumpStatus){
            sens = 0.5;
        }else{
            sens = -0.5;
        }
        lastForcePumpStatus = forcePumpStatus;
    }
    
    if(i >= 100){
        sens = -0.5;
    }else if(i <= 0){
        sens = 0.5;
    }
    i += sens;
    return i;
}

float readSonar() { 
    float distance = -1;
    if(activateSonarSensor){
        if(SONAR_MODE == 1 || SONAR_MODE == 2){
            // Clears the trigPin condition
            digitalWrite(PIN_SONAR_TRIG, LOW);  //
            delayMicroseconds(2);
            // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
            digitalWrite(PIN_SONAR_TRIG, HIGH);
            if(SONAR_MODE == 1){
                delayMicroseconds(10);
            }else{
                delayMicroseconds(1500);
            }
            digitalWrite(PIN_SONAR_TRIG, LOW);
            // Reads the echoPin, returns the sound wave travel time in microseconds
            double duration = pulseIn(PIN_SONAR_ECHO, HIGH);
            #ifdef DEBUG_MODE
                debugD("SONAR [MODE %d] duration %f ms", SONAR_MODE, duration);
            #endif
            // Calculating the distance
            //distance = duration / 29.0 / 2.0; // Speed of sound wave divided by 2 (go and back)
            distance = (duration * (340.0*100) / 1000.0) / 2.0; // Speed of sound wave divided by 2 (go and back)
        }else if (SONAR_MODE == 3 || SONAR_MODE == 4 ){
            Serial2.write(0x01);
            delay(50);
            if(Serial2.available()){
                byte startByte, h_data, l_data, sum = 0;
                byte buf[3];
                
                startByte = (byte)Serial2.read();
                if(startByte == 255){
                    Serial2.readBytes(buf, 3);
                    h_data = buf[0];
                    l_data = buf[1];
                    sum = buf[2];
                    distance = (h_data<<8) + l_data;
                    if((( h_data + l_data)&0xFF) != sum){
                        distance = -999;
                    } 
                }
            }
        }else if (SONAR_MODE == 5){
            Serial2.write(0x01);
            delay(10);
            if(Serial2.available()){
                Serial.println(Serial2.readString());
            }
            return 666;
        }
        // Displays the distance on the Serial Monitor
        #ifdef DEBUG_MODE
            debugD("SONAR [MODE %d] Distance %f cm", SONAR_MODE, distance);
        #endif
    }
    return distance;
}

bool fakeSensorRepeat(){
    return true;
}

// The setup function performs one-time application initialization.
void setup() {
  //On commence par mettre le PIN d'activation de la pompe à LOW pour ne pas qu'elle s'allume au démarrage
    //   digitalWrite(PIN_RELAY, LOW);
    //   pinMode(PIN_RELAY, OUTPUT);

    #ifndef SERIAL_DEBUG_DISABLED
    SetupLogging();
    #endif

    #ifdef BT_ACTIVATED
        SerialBT.begin("SondesEauxNoires"); //Bluetooth device name
    #endif
  

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("SondesEauxNoires")
                    ->set_wifi_access_point("SondesEauxNoires", "12345678")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_ota("12345678")
                    ->get_app();

  int order = 0;
  
  /**
   * TODO changer dans le constructeur du NumberConfig
   * pour virer les références aux pointeurs
   */
  NumberConfig* inputConfVolTotal = new NumberConfig(confVolTotal, confVolTotalPath);
  ConfigItem(inputConfVolTotal)
    ->set_title("Cuve")
    ->set_description("Volume Total du Reservoir (L)")
    ->set_sort_order(100 + order++);

  
  NumberConfig* inputConfPercentStartForce = new NumberConfig(confPercentStartForced, confPercentStartForcedPath);
  ConfigItem(inputConfPercentStartForce)
    ->set_title("Cuve")
    ->set_description("Pourcentage de remplissage du réservoir pour le démarrage force de la pompe (0 - 100) [Hors timer de securite]")
    ->set_sort_order(100 + order++);

  NumberConfig* inputConfPercentStart = new NumberConfig(confPercentStart, confPercentStartPath);
  ConfigItem(inputConfPercentStart)
    ->set_title("Cuve")
    ->set_description("Pourcentage de remplissage du réservoir pour le démarrage auto de la pompe (0 - 100)")
    ->set_sort_order(100 + order++);

  NumberConfig* inputConfPercentStop = new NumberConfig(confPercentStop, confPercentStopPath);
  ConfigItem(inputConfPercentStop)
    ->set_title("Cuve")
    ->set_description("Pourcentage de remplissage du réservoir pour l'arret auto de la pompe (0 - 100)")
    ->set_sort_order(100 + order++);

#ifdef DEBUG_MODE
    debugD("Starting ESP waste %dL start at %d and stop at %d and forced at %d", confVolTotal, confPercentStart, confPercentStop, confPercentStartForced);
#endif

#if ACTIVATE_SONAR == 1
  //2 types de capteurs (sonar et analogique)

  CheckboxConfig* chkActivateSonarSensor = new CheckboxConfig(true, "Activation du capteur sonar", "/sensors/cuve_eaux_noires/activate_sonar");
  ConfigItem(chkActivateSonarSensor)
    ->set_title("Sonar")
    ->set_sort_order(200 + order++);

  activateSonarSensor = chkActivateSonarSensor->get_value();
#endif

#if ACTIVATE_ANALOG == 1
  CheckboxConfig* chkActivateAnalogSensor = new CheckboxConfig(true, "Activation du capteur analogique", "/sensors/cuve_eaux_noires/activate_analog");
  ConfigItem(chkActivateAnalogSensor)
    ->set_title("Sonde")
    ->set_sort_order(300 + order++);

  activateAnalogSensor = chkActivateAnalogSensor->get_value();
#endif

  NumberConfig* inputMaxTimeContinius= new NumberConfig(maxTimeContinius, maxTimeContiniusPath);
  ConfigItem(inputMaxTimeContinius)
    ->set_title("Pompe")
    ->set_description("Durée maximale d'activation de la pompe (min)")
    ->set_sort_order(400 + order++);

  NumberConfig* inputPauseTime = new NumberConfig(pauseTime, pauseTimePath);
  ConfigItem(inputPauseTime)
    ->set_title("Pompe")
    ->set_description("Durée de la pause de refroidissement de la pompe après utilisation au temps max (min)")
    ->set_sort_order(400 + order++);

  //Hysteresys de démarrage et d'arret a partir de la sonde analogique
  auto* hysteresisAutoStarterAnalog = new Hysteresis<float, boolean>(confPercentStop, confPercentStart, false, true);
  auto* thresholdForcedStoperAnalog = new FloatThreshold(confPercentStop, 100, true);
  auto* thresholdForcedStarterAnalog = new FloatThreshold(0, confPercentStartForced, false);

#if ACTIVATE_ANALOG == 1
  //Capteur analogique
  AnalogInput* inputAnalogSensor = new AnalogInput(PIN_ANALOG, MILLIS_REPAT_SENSOR);
  if(activateAnalogSensor){
    #ifdef DEBUG_MODE
        debugD("Starting analog sensor on pin %d", PIN_ANALOG);
    #endif
    pinMode(PIN_ANALOG, INPUT);

    NumberConfig* inputConfSensorAnalogMaxVal = new NumberConfig(confSensorAnalogMaxVal, confSensorAnalogMaxValPath);
    ConfigItem(inputConfSensorAnalogMaxVal)
    ->set_title("Sonde")
    ->set_description("The maximum analog input value (i.e. value when sensor is at the high end)")
    ->set_sort_order(300 + order++);

    NumberConfig* inputConfSensorAnalogMinVal = new NumberConfig(confSensorAnalogMinVal, confSensorAnalogMinValPath);
    ConfigItem(inputConfSensorAnalogMinVal)
    ->set_title("Sonde")
    ->set_description("The minimum analog input value (i.e. value when sensor is at the low end)")
    ->set_sort_order(300 + order++);

    NumberConfig* inputConfSensorAnalogPercentFullAtMinVal = new NumberConfig(confSensorAnalogPercentFullAtMinVal, confSensorAnalogPercentFullAtMinValPath);
    ConfigItem(inputConfSensorAnalogPercentFullAtMinVal)
    ->set_title("Sonde")
    ->set_description("If the sensor is shorter than the height of the tank, the percent of full of tank at min val")
    ->set_sort_order(300 + order++);

    SKOutputInt* volumeTotalSKInfo = new SKOutputInt(
                "tanks.blackWater.analog.capacity",          // Signal K path
                "/sensors/cuve_eaux_noires/analog/volume_total",
                new SKMetadata("L",                       // No units for boolean values
                                "Volume total de la cuve")  // Value description
            );
    volumeTotalSKInfo->set(confVolTotal);

    SKOutputString* tankTypeSKInfo = new SKOutputString(
                "tanks.blackWater.analog.type",          // Signal K path
                "/sensors/cuve_eaux_noires/analog/type",
                new SKMetadata("",                       // No units for boolean values
                                "Type de cuve")  // Value description
                );
    tankTypeSKInfo->set("holding");

    // Takes a moving average for every 10 values, with scale factor
    MovingAverage* avgAnalog = new MovingAverage(5);
    inputAnalogSensor->connect_to(avgAnalog);

    auto analogToPercentCallback = [inputAnalogSensor](float input) ->float {    
       float returnVal = (map(input, confSensorAnalogMinVal, confSensorAnalogMaxVal, confSensorAnalogPercentFullAtMinVal, 100));
    
       analogValueStatusItem.set(inputAnalogSensor->get());

       if(returnVal > 100){
        returnVal = 100;
       }else if(returnVal < confSensorAnalogPercentFullAtMinVal){
        returnVal = confSensorAnalogPercentFullAtMinVal;
       }

       #ifdef DEBUG_MODE       
            debugD("Analog raw %f, AVG %f, %fpercent, min %i, max %i, static %i", inputAnalogSensor->get(), input, returnVal, confSensorAnalogMinVal, confSensorAnalogMaxVal, confSensorAnalogPercentFullAtMinVal);
       #endif

       remplissageReservoir.set(returnVal);

       return returnVal;
    };

    const ParamInfo* param_data_analog_to_percent = new ParamInfo[0]{};

    auto* analogValToPercentTransform = new LambdaTransform<float, float>
            (analogToPercentCallback, param_data_analog_to_percent);

    avgAnalog->connect_to(analogValToPercentTransform);

    analogValToPercentTransform->connect_to(new SKOutputFloat(
                "tanks.blackWater.sonar.currentLevel",          // Signal K path
                "/sensors/cuve_eaux_noires/analog/currentLevel",
                new SKMetadata("%",                       // No units for boolean values
                                "Pourcentage de remplissage de la cuve")  // Value description
                ));

    analogValToPercentTransform->connect_to(new Linear(confVolTotal, 0))
                ->connect_to(new SKOutputFloat(
                "tanks.blackWater.sonar.currentVolume",          // Signal K path
                "/sensors/cuve_eaux_noires/analog/currentVolume",
                new SKMetadata("L",                       // No units for boolean values
                                "Nombre de litres dans la cuve")  // Value description
                ));

    analogValToPercentTransform->connect_to(hysteresisAutoStarterAnalog);
    analogValToPercentTransform->connect_to(thresholdForcedStoperAnalog);
    analogValToPercentTransform->connect_to(thresholdForcedStarterAnalog);


  }
#endif

//Hysteresys de démarrage et d'arret a partir du sonar
auto* hysteresisAutoStarterSonar = new Hysteresis<float, boolean>(confPercentStop, confPercentStart, false, true);
auto* thresholdForcedStoperSonar = new FloatThreshold(confPercentStop, 100, true);
auto* thresholdForcedStarterSonar = new FloatThreshold(0, confPercentStartForced, false);
  
#if ACTIVATE_SONAR == 1
  auto* sonarSensor = new RepeatSensor<float>(MILLIS_REPAT_SENSOR, readSonar);
  if(activateSonarSensor){
    if (SONAR_MODE >= 3 ){
        #ifdef DEBUG_MODE
            debugD("Starting sonar in mode %d on RX pin %d and TX pin %d", SONAR_MODE, PIN_SONAR_RX, PIN_SONAR_TX);
        #endif
        Serial2.begin(115200, SERIAL_8N1, PIN_SONAR_RX, PIN_SONAR_TX);    //Hardware Serial of ESP32
    }else{
        #ifdef DEBUG_MODE
            debugD("Starting sonar in mode %d on trigger pin %d and echo pin %d", SONAR_MODE, PIN_SONAR_TRIG, PIN_SONAR_ECHO);
        #endif
        pinMode(PIN_SONAR_TRIG, OUTPUT); // Sets the trigPin as an OUTPUT
        pinMode(PIN_SONAR_ECHO, INPUT); // Sets the echoPin as an INPUT
    }

    //Capteur Sonar
    //Configuration pour le capteur sonar
    NumberConfig* inputConfMaxHeightRead = new NumberConfig(confMaxHeightRead, confMaxHeightReadPath);
    ConfigItem(inputConfMaxHeightRead)
    ->set_title("Sonar")
    ->set_description("Hauteur lue du capteur sonar au niveau bas de la cuve (cm)")
    ->set_sort_order(200 + order++);

    NumberConfig* inputConfMinHeightRead = new NumberConfig(confMinHeightRead, confMinHeightReadPath);
    ConfigItem(inputConfMinHeightRead)
    ->set_title("Sonar")
    ->set_description("Hauteur lue du capteur sonar au niveau haut de la cuve (cm)")
    ->set_sort_order(200 + order++);

    SKOutputInt* volumeTotalSKInfo = new SKOutputInt(
                "tanks.blackWater.sonar.capacity",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/capacity",         // configuration path
                new SKMetadata("L",                       // No units for boolean values
                                "Volume total de la cuve")  // Value description
            );
    volumeTotalSKInfo->set(confVolTotal);

    SKOutputString* tankTypeSKInfo = new SKOutputString(
                "tanks.blackWater.sonar.type",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/type",         // configuration path
                new SKMetadata("",                       // No units for boolean values
                                "Type de cuve")  // Value description
                );
    tankTypeSKInfo->set("holding");
    
    
    // auto* sonarSensor = new RepeatSensor<float>(MILLIS_REPAT_SENSOR, fakeReadSonar);

    //Trasformation de la lecture du reservoir en pourcentage de remplissage
    auto sonarValToPercentCalbback = [](float input, int miniHeight, int maxiHeight) ->float {
        if(input >= maxiHeight){
            #ifdef DEBUG_MODE
                debugD("SUP AU MAXI HEIGHT TANK EMPTY %f>%d", input, maxiHeight);
            #endif
            return 0;
        }else if (input <= miniHeight){
            #ifdef DEBUG_MODE
                debugD("INF AU MINI HEIGHT TANK FULL %f<%d", input, miniHeight);
            #endif
            return 100;
        }else{
            float range = maxiHeight - miniHeight;
            float valNormal = input - miniHeight;
            
            float retVal = (1.0f-(valNormal / range))*100.0f;
            #ifdef DEBUG_MODE
                debugD("PERCENT FULL %f FOR INPUT %f HEIGHT MAXI %f MINI %f", retVal, input, maxiHeight, miniHeight); 
            #endif

            return retVal;
        }
    };

    const ParamInfo* param_data_val_to_percent = new ParamInfo[2]{
        {"miniHeight", "Hauteur minimale possible de lecture du capteur"},
        {"maxiHeight", "Hauteur maximale possible de lecture du capteur"},
    };

    auto* sonarValToPercent = new LambdaTransform<float, int, int, float>
            (sonarValToPercentCalbback, confMinHeightRead, confMaxHeightRead, param_data_val_to_percent);
    
    sonarSensor->connect_to(sonarValToPercent);

    sonarValToPercent->connect_to(new SKOutputInt(
                "tanks.blackWater.sonar.currentLevel",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/currentLevel",
                new SKMetadata("%",                       // No units for boolean values
                                "Pourcentage de remplissage de la cuve")  // Value description
                ));

    sonarValToPercent->connect_to(new Linear(1/100*confVolTotal, 0))
                ->connect_to(new SKOutputInt(
                "tanks.blackWater.sonar.currentVolume",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/currentVolume",
                new SKMetadata("L",                       // No units for boolean values
                                "Nombre de litres dans la cuve")  // Value description
                ));

#if SEND_SK_SONAR_EXT_INFO
    sonarSensor->connect_to(new SKOutputInt(
                "sensors.cuve_eaux_noires.sonar.cm_read",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/cm_read",         // configuration path
                new SKMetadata("cm",                       // No units for boolean values
                                "Hauteur de lecture")  // Value description
                ));

    sonarValToPercent->connect_to(new SKOutputInt(
                "sensors.cuve_eaux_noires.sonar.percent_inside",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/percent_inside",         // configuration path
                new SKMetadata("%",                       // No units for boolean values
                                "Pourcentage de remplissage de la cuve")  // Value description
                ));

    sonarValToPercent->connect_to(new Linear(-1, +100))
                ->connect_to(new Linear(1/100, 0))
                ->connect_to(new SKOutputInt(
                "sensors.cuve_eaux_noires.sonar.percent_free",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/percent_free",
                new SKMetadata("%",                       // No units for boolean values
                                "Pourcentage d'espace libre de la cuve")  // Value description
                ));
    
    sonarValToPercent->connect_to(new Linear(1/100*confVolTotal, 0))
                ->connect_to(new SKOutputInt(
                "sensors.cuve_eaux_noires.sonar.liters_inside",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/liters_inside",
                new SKMetadata("L",                       // No units for boolean values
                                "Nombre de litres dans la cuve")  // Value description
                ));

    sonarValToPercent->connect_to(new Linear(-1, +100))
                ->connect_to(new Linear(1/100*confVolTotal, 0))
                ->connect_to(new SKOutputInt(
                "sensors.cuve_eaux_noires.sonar.liters_free",          // Signal K path
                "/sensors/cuve_eaux_noires/sonar/liters_free",
                new SKMetadata("%",                       // No units for boolean values
                                "Pourcentage d'espace libre de la cuve")  // Value description
                ));
#endif

    sonarValToPercent->connect_to(hysteresisAutoStarterSonar);
    sonarValToPercent->connect_to(thresholdForcedStoperSonar);
    sonarValToPercent->connect_to(thresholdForcedStarterSonar);
    
  }
#endif

  
  // Interrupteur de marche forcée
  auto* digitalMarcheForce = new DigitalInputChange(PIN_FORCE_START, INPUT_PULLUP, CHANGE);
  auto* digitalMarcheForceDebounced = new DebounceBool();
  digitalMarcheForce->connect_to(digitalMarcheForceDebounced);
  digitalMarcheForce->attach([digitalMarcheForce](){
        if(digitalMarcheForce->get() == LOW){
            forcePumpStatus = true;
        }else{
            forcePumpStatus = false;
        }
        #ifdef DEBUG_MODE
            debugD("DIGI IN FORCE PUMP STATUS %d", forcePumpStatus);
        #endif
  });


  auto* digitalMarcheAuto = new DigitalInputChange(PIN_AUTO_START, INPUT_PULLUP, CHANGE);
  auto* digitalMarcheAutoDebounced = new DebounceBool();
  digitalMarcheAuto->connect_to(digitalMarcheAutoDebounced);
  digitalMarcheAutoDebounced->attach([digitalMarcheAutoDebounced](){
        if(digitalMarcheAutoDebounced->get() == LOW){
            autoPumpStatus = true;
        }else{
            autoPumpStatus = false;
        }
        #ifdef DEBUG_MODE
            debugD("DIGI IN AUTO PUMP STATUS %d", autoPumpStatus);
        #endif
  });


  //SignalK marche forcée
  auto* signalKMarcheForce =
      new BoolSKListener("sensors.cuve_eaux_noires.pump.force_on_sk");
    signalKMarcheForce->attach([](){
                forcePumpStatus = !forcePumpStatus;
                #ifdef DEBUG_MODE
                    debugD("SK FORCE PUMP STATUS %d", forcePumpStatus);
                #endif
    });

  //Web UI marche forcee
//   auto* webUIMarcheForceButton = UIButton::add("ForcePump", "Force pompe", false); 
//   webUIMarcheForceButton->attach([](){
//         forcePumpStatus = !forcePumpStatus;
//         #ifdef DEBUG_MODE
//             debugD("WEB UI FORCE PUMP STATUS %d", forcePumpStatus);
//         #endif
//   });


  //Relais d'activation
  auto* relais = new DigitalOutput(PIN_RELAY);
  relais->connect_to(new SKOutputBool(
              "tanks.blackWater.0.pumpState",          // Signal K path
              new SKMetadata("ON/OFF",                       // No units for boolean values
                            "Pompe en fonctionnement")  // Value description
              ));



  // De la valeur de :
  //    hysteresisAutoStarter  -> si true, on active
  //    OU thresholdForcedStoper ET (digitalMarcheForceDebounced OU signalKMarcheForce OU webUIMarcheForce) - > Si true, alors on active
  //Trasformation de la lecture du reservoir en pourcentage de remplissage
  auto isActivateRelais = [sonarSensor, inputAnalogSensor, hysteresisAutoStarterSonar, thresholdForcedStoperSonar, hysteresisAutoStarterAnalog, thresholdForcedStoperAnalog, digitalMarcheForce, thresholdForcedStarterSonar, thresholdForcedStarterAnalog](float input) ->bool {    
    bool pumpActivated = forcePumpStatus;

    #if ACTIVATE_SONAR == 1
    if(activateSonarSensor && thresholdForcedStoperSonar->get() 
            && (autoPumpStatus && hysteresisAutoStarterSonar->get())){
        //Activation en mode SONAR
        #ifdef DEBUG_MODE
            debugD("Sonar read %fcm : Activate Relais autoStart=%d, secuStoper=%d, forcedDigiStarter=%d, forcePumpStatus=%i", sonarSensor->get(),  hysteresisAutoStarterSonar->get(), thresholdForcedStoperSonar->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0);
            #ifdef BT_ACTIVATED
                SerialBT.printf("[DEBUG] Sonar read %fcm : Activate Relais autoStart=%d, secuStoper=%d, forcedDigiStarter=%d, forcePumpStatus=%i\n", sonarSensor->get(),  hysteresisAutoStarterSonar->get(), thresholdForcedStoperSonar->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0);
            #endif
       #endif
        
        pumpActivated = true;
    }
    #endif
    
    #if ACTIVATE_ANALOG == 1
        if(activateAnalogSensor && thresholdForcedStoperAnalog->get() 
                && (autoPumpStatus && hysteresisAutoStarterAnalog->get())){
            //Activation en mode ANALOGIQUE
             #ifdef DEBUG_MODE
                debugD("Analog read %fohm : Activate Relais autoStart=%d, secuStoper=%d, forcedDigiStarter=%d, forcePumpStatus=%i", inputAnalogSensor->get(), hysteresisAutoStarterAnalog->get(), thresholdForcedStoperAnalog->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0);
                
                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] Analog read %fohm : Activate Relais autoStart=%d, secuStoper=%d, forcedDigiStarter=%d, forcePumpStatus=%i", inputAnalogSensor->get(), hysteresisAutoStarterAnalog->get(), thresholdForcedStoperAnalog->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0);
                #endif
            #endif

            pumpActivated = true; 
        }
    #endif

    if(!(thresholdForcedStoperSonar->get() || thresholdForcedStoperAnalog->get() )){
        //La pompe doit etre en mode forcé, mais le niveau est trop faible
         #ifdef DEBUG_MODE
            debugD("Niveau mini atteint Sonar %d analog %d", thresholdForcedStoperSonar->get(), thresholdForcedStoperAnalog->get());

            #ifdef BT_ACTIVATED
                SerialBT.printf("[DEBUG] Niveau mini atteint Sonar %d analog %d", thresholdForcedStoperSonar->get(), thresholdForcedStoperAnalog->get());
            #endif
        #endif

        pumpActivated = false;
    }

    if(! (forcePumpStatus || autoPumpStatus)){
        //On est en mode OFF
         #ifdef DEBUG_MODE
            debugD("MODE OFF Switch Marche force %d Switch auto %d", forcePumpStatus, autoPumpStatus);
            
            #ifdef BT_ACTIVATED
                SerialBT.printf("[DEBUG] MODE OFF Switch Marche force %d Switch auto %d", forcePumpStatus, autoPumpStatus);
            #endif
        #endif
        
        pumpActivated = false;
    }

    if(!pumpActivated){
        #if ACTIVATE_SONAR == 1
        if(activateSonarSensor){
             #ifdef DEBUG_MODE
                debugD("MODE AUTO %d hysteresisAutoStarterSonar %d, thresholdForcedStoperSonar=%d, digitalMarcheForce=%d, forcePumpStatus=%i", autoPumpStatus, hysteresisAutoStarterSonar->get(), thresholdForcedStoperSonar->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0); 
                
                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] MODE AUTO %d hysteresisAutoStarterSonar %d, thresholdForcedStoperSonar=%d, digitalMarcheForce=%d, forcePumpStatus=%i", autoPumpStatus, hysteresisAutoStarterSonar->get(), thresholdForcedStoperSonar->get(), digitalMarcheForce->get() , forcePumpStatus ? 1 : 0);
                #endif
            #endif
        }
        #endif

        #if ACTIVATE_ANALOG == 1
        if(activateAnalogSensor){
             #ifdef DEBUG_MODE
                debugD("MODE AUTO %d hysteresisAutoStarterAnalog %d, thresholdForcedStoperAnalog=%d, digitalMarcheForce=%d, forcePumpStatus=%i", autoPumpStatus, hysteresisAutoStarterAnalog->get(), thresholdForcedStoperAnalog->get() , digitalMarcheForce->get(), forcePumpStatus ? 1 : 0);

                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] MODE AUTO %d hysteresisAutoStarterAnalog %d, thresholdForcedStoperAnalog=%d, digitalMarcheForce=%d, forcePumpStatus=%i", autoPumpStatus, hysteresisAutoStarterAnalog->get(), thresholdForcedStoperAnalog->get() , digitalMarcheForce->get(), forcePumpStatus ? 1 : 0);
                #endif
            #endif
        }
        #endif
    }

    //gestion des timers de sécurité
    if(pumpActivated){
        if(pumpStartTime == 0){
            pumpStartTime =  millis();
        }
        if(thresholdForcedStarterSonar->get() || thresholdForcedStarterAnalog->get()){
            //le niveau est très haut, on force le fonctionnement
             #ifdef DEBUG_MODE
                debugD("NIVEAU TRES HAUT, ON FORCE - SECU TIMER %d ms [< MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60));

                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] NIVEAU TRES HAUT, ON FORCE - SECU TIMER %d ms [< MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60));
                #endif
            #endif

            pompeActivee.set("oui");
            pumpStopSecuTime=0;
            return true;
        }else if((millis() - pumpStartTime) > (maxTimeContinius*1000*60)){
            if(pumpStopSecuTime == 0){
                pumpStopSecuTime =  millis();
            }else if((millis() - pumpStopSecuTime) > (pauseTime*1000*60)){
                pumpStartTime = 0;
                pumpStopSecuTime = 0;
            }

             #ifdef DEBUG_MODE
                debugD("SECU TIMER %d ms [> MAX %d] SECU TIMER %d [MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60), (millis() - pumpStopSecuTime),  (pauseTime*1000*60));
                
                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] SECU TIMER %d ms [> MAX %d] SECU TIMER %d [MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60), (millis() - pumpStopSecuTime),  (pauseTime*1000*60));
                #endif
            #endif
            pompeActivee.set("non");
            return false; //On est en mode durée de sécurité
        }else{
             #ifdef DEBUG_MODE
                debugD("SECU TIMER %d ms [< MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60));

                #ifdef BT_ACTIVATED
                    SerialBT.printf("[DEBUG] SECU TIMER %d ms [< MAX %d]", (millis() - pumpStartTime), (maxTimeContinius*1000*60));
                #endif
            #endif
            pompeActivee.set("oui");
            return true;
        }
    }else{
        pumpStartTime = 0;
        pumpStopSecuTime = 0;
        pompeActivee.set("non");
        return false;
    }

  };

  const ParamInfo* param_data_activate_relais = new ParamInfo[0]{
  };

  auto* relaisActivator = new LambdaTransform<float, bool>
         (isActivateRelais, param_data_activate_relais);

  //Utilisé pour faire une boucle et declencher les evenements
  auto* fakeSensor = new RepeatSensor<bool>(MILLIS_REPAT_SENSOR, fakeSensorRepeat);
  fakeSensor->connect_to(relaisActivator);
  relaisActivator->connect_to(relais);

}

void loop() {
    event_loop()->tick(); 
}
