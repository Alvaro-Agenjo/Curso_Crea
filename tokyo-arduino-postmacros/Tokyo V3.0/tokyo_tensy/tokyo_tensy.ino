#include <math.h>
#include <pt.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>

#include <std_msgs/Byte.h>
#include <std_msgs/String.h>


// -------------------------------------- ROS -------------------------------------- //
bool enableROS = true;

float navVx = 0.0;
float navVth = 0.0;

float currentVelx = 0.0;
float currentVelth = 0.0;

float oldVelx = 0.0;
float oldVelth = 0.0;

byte ordenRecibida;

geometry_msgs::Vector3 moves;

ros::NodeHandle nh;
std_msgs::Int32 battery;
std_msgs::String runMsg;

ros::Publisher movement("movement_arduino", &moves);
ros::Publisher runscriptPublisher("runscript", &runMsg);
ros::Publisher batteryPublisher("battery", &battery);

void navCb(const std_msgs::String& value)
{
  String data = value.data;
  if (data == "nav")
  {
    navVx = 0.0;
    navVth = 0.0;

  }
  else {
    navVx = 0.0;
    navVth = -1.0;

  }
}

void eyesCb(const std_msgs::String& value)
{
  Serial5.print(value.data);
}


void messageCb(const geometry_msgs::Point& velocity)
{
  navVx = velocity.x;
  navVth = velocity.z;

}

ros::Subscriber<geometry_msgs::Point> sub("ard_cmd_vel", messageCb );
ros::Subscriber<std_msgs::String> eyes_sub("tokyo_eyes", eyesCb );
ros::Subscriber<std_msgs::String> communication("nav_state", navCb );



int tROS = 20;
long lastTimeROSE;//Envío
long lastTimeROSR;//Recibo

// ---------------------------------------- SENSORES INFRARROJOS ---------------------------------------- //

bool activar_sensores = false;
int pinIR_DER = 25 ;
int pinIR_IZQ = 24 ;
int margen = -1;

//------------------------------------- MEDIDOR BATERÍA ---------------------------------------------- //
String inString = "";      // string to hold input

int incomingByte, BalanceCode, Length, highbyte, lowbyte;
byte BatteryConfigH, BatteryConfigL, bcl, bcln, bch, Checksum, switche;
uint8_t BYTE1, BYTE2, BYTE3, BYTE4, BYTE5, BYTE6, BYTE7, BYTE8, BYTE9, BYTE10;
uint8_t inInts[40], data[9];   // an array to hold incoming data, not seen any longer than 34 bytes, or 9
uint16_t a16bitvar;
float  eresultf; //Cellv1, Cellv2, Cellv3, Cellv4, Cellv5, Cellv6, Cellv7, Cellv8;
bool lowBattery = false;
float CellMin = 5, CellMax = 0, Cellsum = 0;

//---------------------------------------------------  MOTORES ---------------------------------------//


//Variables CONEXIONES DRIVER
int enable_DER = 2;
int PUL_DER = 3;//pul-
int DIR_DER = 4;//dir- //PUL+ Y DIR+ a

int enable_IZQ = 5;
int PUL_IZQ = 6;//pul-
int DIR_IZQ = 7;//dir-

volatile long ISRCounterDER = 0;
volatile long ISRCounterIZQ = 0;
volatile long pulsosDER = 0;
volatile long pulsosIZQ = 0;

//ENCODERS
int encoderDERBn = 9;
int encoderDERAp = 10;
int encoderDERBp = 11;
int encoderDERAn = 12;

int encoderIZQAp = 14;
int encoderIZQBp = 15;
int encoderIZQAn = 18;
int encoderIZQBn = 19;

float velDER, velIZQ;

long lastencoderDerecho;
long lastencoderIzquierdo;

//ESTADOS MOVIMIENTO
enum Estado_NAV {
  PAUSA,
  CURVO,
  GIRO_DER,
  GIRO_IZQ,
  RETROCEDIENDO
};
Estado_NAV estado_NAV = PAUSA;
Estado_NAV last_estado_NAV = PAUSA;

//Variables para el tren de pulsos
bool accelerado = false;
int periodoDer;
int periodoIzq;
float frec_der;
float frec_izq;

float radio = 0.0795;//metros
float distRuedas = 0.3445;
int micropaso = 200;
float pi = 3.1416;
float resolucionEncoder = 0.000125;//Perimetro/pulsosEncoder 500/2000 mm/pulsos

//Rampa aceleracion
long lastRight = 0;
long lastLeft = 0;
float velDer, velIzq;
float lastVelDer, lastVelIzq;

//-------------------------------------------------------------------------------

float velStep = 0.29;//7.0

bool instruccionNavVx = false;
bool instruccionNavVth = false;
String vxLineal =  "";
String vthAngular = "";


static struct pt pt1, pt2, ptREAD, ptSEND, ptBATTERY, ptCABEZA; // each protothread needs one of these
bool isMoving = false;

bool breaking;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// FUNCIONES ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// PROGRAMA PRINCIPAL //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ------------------------- CONTROL MOTORES -------------------------------- //

void recibeNav()
{
  moves.x = currentVelx;
  moves.z = currentVelth;
  movement.publish(&moves);
}

//-----------------------PROGRAMA INCIIAL ------------------------/

void setup()
{
  nh.initNode();
  nh.advertise(movement);
  nh.subscribe(eyes_sub);
  nh.subscribe(communication);
  nh.subscribe(sub);
  nh.advertise(runscriptPublisher);
  nh.advertise(batteryPublisher);


  Serial5.begin(57600);  //MegaCabeza
  Serial4.begin(9600);
  // Serial.begin(115200);


  // ----- DESCOMENTAR SOLO UNA VEZ CUANDO EN LA EEPROM NO HAYA NADA ----- //

  // ----------------------------MOTORES----------------------------------------- //
  PT_INIT(&pt1);  // initialise the two
  PT_INIT(&pt2);  // protothread variables
  PT_INIT(&ptREAD);
  PT_INIT(&ptSEND);
  PT_INIT(&ptBATTERY);
  PT_INIT(&ptCABEZA);

  pinMode(PUL_DER, OUTPUT);
  pinMode(DIR_DER, OUTPUT);
  pinMode(PUL_IZQ, OUTPUT);
  pinMode(DIR_IZQ, OUTPUT);
  pinMode(enable_DER, OUTPUT);
  pinMode(enable_IZQ, OUTPUT);

  pinMode(encoderDERAp, INPUT);
  pinMode(encoderDERAn, INPUT);
  pinMode(encoderDERBp, INPUT);
  pinMode(encoderDERBn, INPUT);
  pinMode(encoderIZQAp, INPUT);
  pinMode(encoderIZQAn, INPUT);
  pinMode(encoderIZQBp, INPUT);
  pinMode(encoderIZQBn, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderDERAn), updateEncodersRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderIZQAn), updateEncodersLeft, CHANGE);

  digitalWrite(enable_DER, LOW);
  digitalWrite(enable_IZQ, LOW);
  pinMode(pinIR_DER, INPUT);
  pinMode(pinIR_IZQ, INPUT);
}

void loop() {
  // ----------------------------- ROS ------------------------------------------ //
  if (enableROS) {
    long nowTimer = millis();
    if (nowTimer > lastTimeROSR + 30) {
      nh.spinOnce();
      lastTimeROSR = nowTimer;

      recibeNav();
    }
  }

  //--------Movimiento (estados, dirección, velocidad)
  threadDer(&pt1);
  threadIzq(&pt2);
  //--------Lectura datos nav ROS
  readNucROS(&ptREAD);
  //--------Envía movimiento del robot
  writeNucROS(&ptSEND);
  //--------Envío de datos MEGA cabeza
  readBattery(&ptBATTERY);
  //--------Lectura batería
  readCabeza(&ptCABEZA);
}

static int readBattery(struct pt *pt) {

  PT_BEGIN(pt);
  while (1) {

    static unsigned long timestamp = millis();
    static unsigned long tBattery = 10000;
    PT_YIELD_UNTIL(pt, millis() >= tBattery + timestamp);

    int incomByte = Serial4.read();

    uint8_t data[5][9] = {
      {221, 90, 0, 2, 86, 120, 255,   48, 119},
      {221, 90, 1, 2, 0,    0, 255,  253, 119},
      {221, 90, 0, 2, 86, 120, 255,   48, 119},
      {221, 90, 1, 2, 0,    0, 255,  253, 119},
      {221, 90, 0, 2, 86, 120, 255,   48, 119}
    };

    static int i = 0;

    for (i = 0; i < 5; i++) {
      /////FLUSH
      timestamp = millis();
      PT_YIELD_UNTIL(pt, millis() >= 100 + timestamp);

      while (Serial4.available() > 0) {
        Serial4.read();
      }
      /////////

      ////////
      timestamp = millis(); // take a new timestamp

      PT_YIELD_UNTIL(pt, millis() >= 50 + timestamp);

      Serial4.write(data[i], 2);
      ///////
    }

    // -------------------      EPROM READS END    -----------------------

    //-------------------- EPROM WRITES START ----------------------------------

    //-------------------- EPROM WRITES END ----------------------------------

    //CCCCCCCCCCCCCCCCCCCCCCC  CELLS VOLTAGE  CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC

    //USING BASIC INFO 03 get
    /////FLUSH
    timestamp = millis(); // take a new timestamp

    PT_YIELD_UNTIL(pt, millis() >= 200 + timestamp);

    while (Serial4.available() > 0) {
      Serial4.read();
    }

    //  USING BASIC INFO
    /////FLUSH
    timestamp = millis(); // take a new timestamp

    PT_YIELD_UNTIL(pt, millis() >= 100 + timestamp);

    while (Serial4.available() > 0) {
      Serial4.read();
    }
    /////////
    call_Basic_info();      // requests basic info.

    timestamp = millis(); // take a new timestamp

    PT_YIELD_UNTIL(pt, millis() >= 100 + timestamp);

    get_bms_feedback();   // get that data, used to get BALANCE STATE byte 17 less 4, decimal=byte 13

    BalanceCode = inInts[13]; //  the 13th byte
    BalanceCode = Bit_Reverse( BalanceCode ) ; // reverse the bits, so they are in same order as cells
    print_binary(BalanceCode, 8);// print balance state as binary, cell 1 on the right, cell 8 on left
    //                                    Reversed this. 1 on left, 8 on right


    // PACK VOLTAGE,, bytes 0 and 1, its 16 bit, high and low
    highbyte = (inInts[0]); // bytes 0 and 1
    lowbyte = (inInts[1]);
    uint16_t PackVoltage = two_ints_into16(highbyte, lowbyte);
    float PackVoltagef = PackVoltage / 100.0f; // convert to float and leave at 2 dec places
    //Serial.print(" PackVoltagef:");
    //Serial.print(PackVoltagef);

    int RSOC;

    if (PackVoltagef > 28.7) {
      RSOC = 100;
    }
    else if (PackVoltagef <= 28.7 && PackVoltagef > 28.0) {
      RSOC = fmap(PackVoltagef, 28.0, 28.7, 90, 100);
    }
    else if (PackVoltagef <= 28.0 && PackVoltagef > 27.4) {
      RSOC = fmap(PackVoltagef, 27.4, 28.0, 80, 90);
    }
    else if (PackVoltagef <= 27.4 && PackVoltagef > 26.7) {
      RSOC = fmap(PackVoltagef, 26.7, 27.4, 70, 80);
    }
    else if (PackVoltagef <= 26.7 && PackVoltagef > 26.0) {
      RSOC = fmap(PackVoltagef, 26.0, 26.7, 60, 70);
    }
    else if (PackVoltagef <= 26.0 && PackVoltagef > 25.6) {
      RSOC = fmap(PackVoltagef, 25.6, 26.0, 50, 60);
    }
    else if (PackVoltagef <= 25.6 && PackVoltagef > 24.6) {
      RSOC = fmap(PackVoltagef, 24.6, 25.6, 35, 50);
    }
    else if (PackVoltagef <= 24.6 && PackVoltagef > 24.0) {
      RSOC = fmap(PackVoltagef, 24.0, 24.6, 20, 35);
    }
    else if (PackVoltagef <= 24.0 && PackVoltagef > 23.8) {
      RSOC = fmap(PackVoltagef, 23.8 , 24.0 , 5, 20);
    }
    else if (PackVoltagef <= 23.8) {
      RSOC = 0;
    }
    //  Serial.print("RSOC: ");
    //  Serial.println(RSOC);

    // CURRENT
    float PackCurrentf;
    highbyte = (inInts[2]); // bytes 2 and 3
    lowbyte = (inInts[3]);
    int PackCurrent = two_ints_into16(highbyte, lowbyte);
    if (PackCurrent > 32768) {
      PackCurrent = 65536 - PackCurrent;
      PackCurrentf = PackCurrent / (-100.0f); // convert to float and leave at 2 dec places
    }
    else {//Cargando
      PackCurrentf = PackCurrent / 100.0f; // convert to float and leave at 2 dec places
    }
    if (PackCurrentf > 0.1) {
      if (PackVoltagef >= 29.05) {
        RSOC = 100;
      }
      else {
        RSOC = 101;
      }
      //Serial.println("Cargando");
    }

    //  Serial.print(" PackCurrentf:");
    //  Serial.print(PackCurrentf);
    battery.data = RSOC;
    batteryPublisher.publish(&battery);


    if ((PackVoltagef < 24.1) && (lowBattery == false)) {
      Serial5.print("fs");
      lowBattery = true;
    }
    else if ((PackVoltagef > 24.2) && (lowBattery == true)) {
      Serial5.print("fa");
      lowBattery = false;
    }
    if ( (PackVoltagef < 23.5) ) {
      Serial5.print("apa");
      delay(5);
      runMsg.data = "shutdown";
      runscriptPublisher.publish(&runMsg);
    }

    write_request_end();
  } PT_END(pt);
}

static int readCabeza(struct pt * pt) {

  PT_BEGIN(pt);

  if (Serial5.available() > 0)
  {
    char charByte = Serial5.read();
    if ( charByte == 'g')
    { // Apagar
      runMsg.data = "shutdown";
      runscriptPublisher.publish(&runMsg);
    }
  }
  PT_END(pt);
}

static int threadDer(struct pt * pt) {
  static unsigned long timeStamp = 0;

  static byte current = 0;
  static bool triggered = false;

  PT_BEGIN(pt);

  if (isMoving) {

    if (current == 0) {
      if (!triggered)
      {
        calculoCinematicaRight();
        triggered = true;
        digitalWrite(PUL_DER, HIGH);
      }
      // runMsg.data = frec_der;
      // runscriptPublisher.publish(&runMsg);
      PT_YIELD_UNTIL(pt, micros() - timeStamp > frec_der / 2);
      triggered = false;

      timeStamp = micros();

      current = 1;
    }
    else if (current == 1) {
      if (!triggered)
      {
        triggered = true;
        digitalWrite(PUL_DER, LOW);
      }

      PT_YIELD_UNTIL(pt, micros() - timeStamp > frec_der / 2);

      timeStamp = micros();

      triggered = false;

      current = 0;
      pulsosDER++;
    }
  }

  PT_END(pt);
}

static int threadIzq(struct pt * pt) {
  static unsigned long timeStamp = 0;
  static byte current = 0;
  static bool triggered = false;
  //frec_izq=12000;

  PT_BEGIN(pt);

  if (isMoving) {
    if (current == 0) {

      if (!triggered)
      {
        calculoCinematicaLeft();

        triggered = true;
        digitalWrite(PUL_IZQ, HIGH);
      }

      PT_YIELD_UNTIL(pt, micros() - timeStamp > frec_izq / 2);
      timeStamp = micros();
      current = 1;
      triggered = false;
    }
    else if (current == 1) {
      if (!triggered)
      {
        triggered = true;
        digitalWrite(PUL_IZQ, LOW);
      }
      PT_YIELD_UNTIL(pt, micros() - timeStamp > frec_izq / 2);
      timeStamp = micros();
      current = 0;
      triggered = false;
      pulsosIZQ++;
    }
  }
  PT_END(pt);
}



static int readNucROS(struct pt * pt) {
  static unsigned long timestamp = 0;

  PT_BEGIN(pt);

  timestamp = millis(); // take a new timestamp

  navVth = constrain(navVth, -1, 1);
  vxLineal = "";
  vthAngular = "";

  float absVth = abs(navVth);
  float absVx = abs(navVx);

  float offset = 4.2;//1.8

  if (!breaking) {
    if (estado_NAV == PAUSA) {
      if (absVth > 0.0) {
        if (absVx * offset < absVth)//absVx < 0.35
        {
          if (navVth > 0.2)
          {
            estado_NAV = GIRO_DER;
            navVx = 0.0;
            digitalWrite(enable_DER, HIGH);
            digitalWrite(enable_IZQ, HIGH);
            navegacion();

          }
          else if (navVth < -0.2)
          {
            estado_NAV = GIRO_IZQ;
            navVx = 0.0;
            digitalWrite(enable_DER, HIGH);
            digitalWrite(enable_IZQ, HIGH);
            navegacion();

          }
        }
        else if (navVx > 0.09) {
          estado_NAV = CURVO;
          digitalWrite(enable_DER, HIGH);
          digitalWrite(enable_IZQ, HIGH);
          navegacion();
        }
        else if (navVx < -0.09) {
          estado_NAV = RETROCEDIENDO;
          digitalWrite(enable_DER, HIGH);
          digitalWrite(enable_IZQ, HIGH);
          navegacion();
        }
      }
      else {
        if (navVx > 0.09)
        {
          estado_NAV = CURVO;
          digitalWrite(enable_DER, HIGH);
          digitalWrite(enable_IZQ, HIGH);
          navegacion();

        }
        else if (navVx < -0.09) {
          estado_NAV = RETROCEDIENDO;
          digitalWrite(enable_DER, HIGH);
          digitalWrite(enable_IZQ, HIGH);

          navegacion();

        }
      }
    }
    else if (estado_NAV == GIRO_IZQ) {
      if (navVth > -0.02 || absVx * offset >= absVth) {//absVx >= 0.40
        estado_NAV = PAUSA;
        navegacion();
      }
      else {
        navVx = 0.0;
      }
    }
    else if (estado_NAV == GIRO_DER) {
      if (navVth < 0.02 || absVx * offset >= absVth) {//absVx >= 0.40
        estado_NAV = PAUSA;
        navegacion();
      }
      else {
        navVx = 0;
      }
    }
    else if (estado_NAV == CURVO) {
      if (absVx < 0.09)
      {
        estado_NAV = PAUSA;
        navegacion();

      }
    }
    else if (estado_NAV == RETROCEDIENDO) {
      if (absVx < 0.09) {
        estado_NAV = PAUSA;
        navegacion();
      }
    }
  }

  PT_END(pt);
}

static int writeNucROS(struct pt * pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);

  char Vx_char[5];//2 decimales
  char Vth_char[5];
  PT_YIELD_UNTIL(pt, millis() >= tROS + timestamp);
  timestamp = millis(); // take a new timestamp

  calculoVel();

  itoa(int(currentVelx * 100), Vx_char, 10);
  itoa(int(currentVelth * 100), Vth_char, 10);

  if (currentVelx == 0 && currentVelth == 0) {
    lastencoderDerecho = 0;
    lastencoderIzquierdo = 0;

    frec_der = 0;
    frec_izq = 0;
    resetEncoders();
  }

  PT_END(pt);
}

void resetEncoders() {
  ISRCounterIZQ = 0;
  ISRCounterDER = 0;
  lastencoderDerecho = 0;
  lastencoderIzquierdo = 0;
}

void updateEncodersRight() {
  if (digitalRead(encoderDERBp) == HIGH) {
    if (digitalRead(encoderDERAp) == LOW) {
      ISRCounterDER++;
    } else {
      ISRCounterDER--;
    }
  } else {
    if (digitalRead(encoderDERAp) == LOW) {
      ISRCounterDER--;
    } else {
      ISRCounterDER++;
    }
  }
}

void updateEncodersLeft() {
  if (digitalRead(encoderIZQAp) == HIGH) {
    if (digitalRead(encoderIZQBp) == LOW) {
      ISRCounterIZQ++;
    } else {
      ISRCounterIZQ--;
    }
  } else {
    if (digitalRead(encoderIZQBp) == LOW) {
      ISRCounterIZQ--;
    } else {
      ISRCounterIZQ++;
    }
  }
}
double readEncodersRight() {
  return ISRCounterDER;
}

double readEncodersLeft() {
  return ISRCounterIZQ;
}

void calculoVel() {
  long encoderDerecho = readEncodersRight();
  long encoderIzquierdo = readEncodersLeft();
  if (readEncodersRight() == 0 && readEncodersLeft() == 0) {
    lastencoderDerecho = 0;
    lastencoderIzquierdo = 0;
  }

  velDer = -((lastencoderDerecho - encoderDerecho) * 0.00025) / (tROS * 0.001);
  velIzq = ((lastencoderIzquierdo - encoderIzquierdo) * 0.00025) / (tROS * 0.001);


  oldVelx = currentVelx;
  oldVelth = currentVelth;
  currentVelx = (velDer + velIzq) / 2;
  currentVelth = (velIzq - velDer) / distRuedas;

  lastencoderDerecho = encoderDerecho;
  lastencoderIzquierdo = encoderIzquierdo;
}

float stepRight;
float stepLeft;

float lastSpeed;

float RampRight(int movido, double tiempo) {
  stepRight = RampCalculation(stepRight, tiempo);
  float vel = abs(navVx - (navVth * distRuedas) / 2);
  vel = lerp(0.00, vel, stepRight);

  return vel;
}

float RampLeft(int movido, double tiempo) {
  stepLeft = RampCalculation(stepLeft, tiempo);
  float vel = abs(navVx + (navVth * distRuedas) / 2);
  vel = lerp(0.00, vel, stepLeft);

  return vel;
}

float RampCalculation(float currentStep, double tiempo) {

  if (currentStep < 0.1) {
    currentStep = 0.1;
  }

  if (currentStep > 0.15) {
    detectIsMoving();
  }
  if (currentStep >= 1) {
    currentStep = 1;
    accelerado = true;
  }
  return currentStep += currentStep * tiempo * 1.2;
  /*
    if (stepLeft < 0.3) {
      stepLeft += 0.15 * tiempo;
    } else if (stepLeft < 0.6) {
      stepLeft += 0.3 * tiempo;
    } else {
      stepLeft += 0.5 * tiempo;
    }
  */
}

float BreakRight(double tiempo) {

  stepRight -= 4.0 * tiempo;

  float currentVel = lerp(0.01, abs(velDER), stepRight);

  if (currentVel <= 0.01) {
    breaking = false;
    Stop();
  }

  return currentVel;
}

float BreakLeft(double tiempo) {

  stepLeft -= 4.0 * tiempo;

  float currentVel = lerp(0.01, abs(velIZQ), stepLeft);

  if (currentVel <= 0.01) {

    breaking = false;
    Stop();
  }

  return currentVel;
}
float currentVelDer;
float currentVelIzq;


void calculoCinematicaRight() {

  double current = millis();
  double tiempo = (current - lastRight) / 1000;

  if (breaking) {
    currentVelDer  = BreakRight(tiempo);
  }
  else if (!accelerado) {
    int movido = abs(readEncodersLeft()) + abs(readEncodersRight());

    velDER = RampRight(movido, tiempo);
    currentVelDer = velDER;
  }
  else {
    if (detectIsMoving()) {
      float newVel = navVx - (navVth * distRuedas) / 2;
      velDER = MoveTowardsSpeed(newVel, velDER, tiempo);

      currentVelDer = velDER;
    }
  }
  currentVelDer = constrain(currentVelDer, -0.8, 0.8);

  frec_der = 1000000 / ((abs(currentVelDer) * 1600) / 0.5);
  lastRight = current;

}

void calculoCinematicaLeft() {


  double current = millis();
  double tiempo = (current - lastLeft) / 1000;

  if (breaking) {
    currentVelIzq  = BreakLeft(tiempo);
  }
  else if (!accelerado) {

    int movido = abs(readEncodersLeft()) + abs(readEncodersRight());

    velIZQ = RampLeft(movido, tiempo);
    currentVelIzq = velIZQ;
  }
  else {
    if (detectIsMoving()) {
      float newVel = navVx + (navVth * distRuedas) / 2;
      velIZQ = MoveTowardsSpeed(newVel, velIZQ, tiempo);
      currentVelIzq = velIZQ;
    }
  }
  currentVelIzq = constrain(currentVelIzq, -0.8, 0.8);

  frec_izq = 1000000 / ((abs(currentVelIzq) * 1600) / 0.5);
  lastLeft = current;
}

float MoveTowardsSpeed(float newVel, float oldVel, double tiempo) {
  if (abs(newVel) < abs(oldVel)) {
    if (abs(newVel) + (velStep * tiempo) < abs(oldVel)) {
      return abs(oldVel) - (velStep * tiempo);
    }
  }
  else if (abs(newVel) > abs(oldVel)) {
    if (abs(newVel) - (velStep * tiempo) > abs(oldVel)) {
      return abs(oldVel) + (velStep * tiempo);
    }
  }
  return abs(newVel);
}

bool detectIsMoving() {
  if (estado_NAV == GIRO_DER || estado_NAV == GIRO_IZQ) {
    float val = abs(abs(currentVelth) - abs((currentVelIzq + currentVelDer) / distRuedas));
    if (val > 0.25) {
      navVx = 0;
      navVth = 0;
      val += 1;
      return false;
    }
  }
  else {
    float val = abs(abs(currentVelx) - abs((currentVelIzq + currentVelDer) / 2));
    if (val > 0.071) {
      navVx = 0;
      navVth = 0;
      dtostrf(val, 6, 2, runMsg.data);
      runscriptPublisher.publish(&runMsg);
      return false;
    }
  }
  return true;
}

void Stop() {
  isMoving = false;

  // digitalWrite(enable_DER, LOW);
  // digitalWrite(enable_IZQ, LOW);

  stepLeft = 0.1;
  stepRight = 0.1;

  velIZQ = 0;
  velDER = 0;

  frec_der = 0;
  frec_izq = 0;
  resetEncoders();


  pulsosDER = 0;
  pulsosIZQ = 0;
  accelerado = false;

  // delay(100);
}

void navegacion() {
  lastRight = millis();
  lastLeft = millis();
  if (estado_NAV == PAUSA) {
    breaking = true;
    stepLeft = 1;
    stepRight = 1;
  }
  else {

    isMoving = true;

    if (estado_NAV == CURVO) {//RECTO-CURVO
      digitalWrite(DIR_DER, HIGH);
      digitalWrite(DIR_IZQ, LOW);
    }
    else  if (estado_NAV == RETROCEDIENDO) {//Retrocediendo
      digitalWrite(DIR_DER, LOW);
      digitalWrite(DIR_IZQ, HIGH);
    }
    else  if (estado_NAV == GIRO_DER) {//Giro IZQUIERDA
      digitalWrite(DIR_DER, LOW);
      digitalWrite(DIR_IZQ, LOW);
    }
    else  if (estado_NAV == GIRO_IZQ) {//Giro DERECHA
      digitalWrite(DIR_DER, HIGH);
      digitalWrite(DIR_IZQ, HIGH);
    }
  }
}

float lerp(float a, float b, float x)
{
  return a + x * (b - a);
}
//-------------------------------------------------------------------------------
uint8_t call_read_eprom()
{
  // BYTES 3 and 6 need to be set first
  uint8_t data1[7] = {221, 165, BYTE3, 0, 255, BYTE6, 119};
  Serial4.write(data1, 7);
  get_bms_feedback(); // get the data reply
  highbyte = (inInts[0]); // bytes 5 and 6, is where the actual data is
  lowbyte = (inInts[1]);
  uint16_t eresult = two_ints_into16(highbyte, lowbyte); // TURN THEM INTO ONE LONG INTEGER
  eresultf = eresult / 100.0f; // convert to float
}
//------------------------------------------------------------------------------------------

uint16_t two_ints_into16(int highbyte, int lowbyte) // turns two bytes into a single long integer
{
  a16bitvar = (highbyte);
  a16bitvar <<= 8; //Left shift 8 bits,
  a16bitvar = (a16bitvar | lowbyte); //OR operation, merge the two
  return a16bitvar;
}
// ----------------------------------------------------------------------------------------------------
void call_Basic_info()
// total voltage, current, Residual capacity, Balanced state, MOSFET control status
{

  //  DD  A5 03 00  FF  FD  77
  // 221 165  3  0 255 253 119
  uint8_t data[7] = {221, 165, 3, 0, 255, 253, 119};
  Serial4.write(data, 7);
}

//----------------------------------------------------------------------------
void write_request_end()
{
  uint8_t data[9] = {221, 90, 1, 2, 0, 0, 255, 253, 119};
  Serial4.write(data, 9);
}

//--------------------------------------------------------------------------
void get_bms_feedback()  // returns with up to date, inString= chars, inInts= numbers, chksum in last 2 bytes
//                          Length
//                          Data only, exclude first 3 bytes
{
  if (Serial4.available() > 0) {
    {
      for (int i = 0; i < 4; i++)               // just get first 4 bytes
      {
        incomingByte = Serial4.read();
        if (i == 3)
        { // could look at 3rd byte, it's the ok signal
          Length = (incomingByte); // The fourth byte holds the length of data, excluding last 3 bytes checksum etc
          // Serial.print(" inc ");
          //Serial.print(incomingByte);
        }
        if (Length == 0) {
          Length = 1; // in some responses, length=0, dont want that, so, make Length=1
        }
      }
      //  Length = Length + 2; // want to get the checksum too, for writing back, saves calculating it later
      for (int i = 0; i < Length + 2; i++) { // get the checksum in last two bytes, just in case need later
        incomingByte = Serial4.read(); // get the rest of the data, how long it might be.
        inString += (char)incomingByte; // convert the incoming byte to a char and add it to the string
        inInts[i] = incomingByte;       // save incoming byte to array as int
      }
    }
  }
}

//-----------------------------------------------------------------------------------------------------
void print_binary(int v, int num_places) // prints integer in binary format, nibbles, with leading zeros
// altered a bit, but got from here,  https://phanderson.com/arduino/arduino_display.html
{

  int mask = 0, n;
  for (n = 1; n <= num_places; n++)
  {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while (num_places)
  {
    --num_places;
  }
}
//-----------------------------------------------------------------------------------------------------
byte Bit_Reverse( byte x )
// http://www.nrtm.org/index.php/2013/07/25/reverse-bits-in-a-byte/
{
  //          01010101  |         10101010
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  //          00110011  |         11001100
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  //          00001111  |         11110000
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
