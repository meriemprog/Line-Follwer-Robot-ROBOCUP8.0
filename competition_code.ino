'"#include<vector>
  //temps de calibrage =13s
  using namespace std;
  const int ML_F = 4;  // PWM pin for Motor A direction 1
  const int ML_B = 16;  // PWM pin for Motor A direction 2  
  const int MR_F = 17;  // PWM pin for Motor B direction 1
  const int MR_B = 18; // PWM pin for Motor B direction 2
  #define LED 2 // internal led 
  const int btnpin=34;
  const int sensorPins[6] = {32,33,25,26,27,13}; 
  float Kp = 43; 
  float Ki = 0.0005; 
  float Kd = 8 ;   
  int error = 0;                  
  int previousError = 0;  
  float integralTerm=0; 
  int derivative = 0; 
  int pidOutput =0;
  bool cerclemode=false;
  bool flechemode=false;
  // Motor Speed Parameters
  int baseSpeed =150;     // Base speed when robot is on line (0-255)
  int maxSpeed = 255;      // Maximum allowed speed
  int minSpeed = 30;       // Minimum speed to overcome friction
  // Sensor Values and Calibration
  int sensorValues[6];     // Raw sensor readings
  int sensorCalibrated[6]; // Calibrated sensor values
  int valblanc[6];        // Minimum values during calibration
  int valnoir[6];        // Maximum values during calibration
  int sensorMin[6]={4095,4095,4095,4095,4095,4095};
  int sensorMax[6]={0,0,0,0,0,0};
  unsigned int t_lekhra ; // instance de khrouj ml couleur inverséée
  bool test_tri9_twil = false;
  unsigned int t0;
  int k=0;
  int k1=0;
  int k2=0;
  int activeCountBefore=0;
  bool test1=false;
  int khrouj =0;



  //Line Position (0 = centered, negative = left, positive = right)
  int linePosition = 0;
  vector<double> weights = {5, 3, 1, -1, -3, -5};
  vector<int>weightscercle={9,7,1,-1,-3,-5};
  void calibrateSensors() {
    int valBlanc[6] = {0, 0, 0, 0, 0, 0};
    int valNoir[6]  = {0, 0, 0, 0, 0, 0};



    Serial.println("=== Calibration Start ===");
    Serial.println("Step 1: Place robot on WHITE area");

    delay(2000); // Time to place it manually
    digitalWrite(LED,HIGH);

    // Measure white surface
    for (int j = 0; j < 50; j++) {
      for (int i = 0; i < 6; i++) {
        valBlanc[i] += abs(analogRead(sensorPins[i]));
      }
      delay(50);
    }
    digitalWrite(LED,LOW);

    delay(2000);
    Serial.println("Step 2: Place robot on BLACK line");
    delay(3000);

    digitalWrite(LED,HIGH);

    // Measure black surface
    for (int j = 0; j < 50; j++) {
      for (int i = 0; i < 6; i++) {
        valNoir[i] +=abs(analogRead(sensorPins[i]));
      }
      delay(50);
    }
    digitalWrite(LED,LOW);

    // Compute average values and save them as min/max calibration
    for (int i = 0; i < 6; i++) {
      sensorMin[i] = abs(valBlanc[i]) / 50;  // White = min reflectivity
      sensorMax[i] = abs(valNoir[i]) / 50;   // Black = max reflectivity
      if (sensorMax[i] < sensorMin[i]) {  // swap if reversed
        int temp = sensorMax[i];
        sensorMax[i] = sensorMin[i];
        sensorMin[i] = temp;
      }
    }


    Serial.println("Calibration complete!");
    for (int i = 0; i < 6; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" -> Min: ");
      Serial.print(sensorMin[i]);
      Serial.print("  Max: ");
      Serial.println(sensorMax[i]);
    }
    Serial.println("=========================");
  }
  // MIN of an array 
  int Min(int* T) {
    int min=T[0];
    for(int i=0;i<6;i++){
      if(T[i]<=min){
        min=T[i];
      }
    }
    return min;
  }
  int Max(int* T) {
    int max=T[0];
    for(int i=0;i<6;i++){
      if(T[i]>=max){
        max=T[i];
      }
    }
    return max;
  }
  int stop = 0;

  void readSensors() {
    /*
    * Reads all 6 sensors and converts to binary values
    * Applies calibration to get consistent readings
    */
    for (int i = 0; i < 6; i++) {
      int rawValue = analogRead(sensorPins[i]);
      
      // Apply calibration: map to 0-1000 range
      int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], 0, 4095);
      
      // Constrain values and convert to binary (0 or 1)
      //calibratedValue = constrain(calibratedValue, 0, 1000);
      sensorCalibrated[i] = (calibratedValue > (Max(sensorMin)+Min(sensorMax))/2) ? 1 : 0; 
    }
  }
  void calculateError() {
    /*
    * Calculates position error based on sensor readings
    * Uses weighted position method for better accuracy
    * Sensor positions: [0, 1, 2, 3, 4, 5] from left to right
    * Weights: [-5, -3, -1, 1, 3, 5] for 6 sensors
    */
    int weightedSum = 0;
    int sensorSum = 0;
    
    // Define weights for each sensor position 
    for (int i = 0; i < 6; i++) {
      weightedSum += sensorCalibrated[i] * weights[i];
      sensorSum += sensorCalibrated[i];
    }
    
    // Calculate error (0 if no sensors detect line)
    if (sensorSum != 0) {
      error = weightedSum;
      linePosition = weightedSum;
    } else {
      // No line detected - use previous error for recovery
      error = previousError > 0 ? 10 : -10; // Continue in last known direction
    }
  }
  void calculatePID() {
    float proportional = Kp * error;
    // Prevent integral windup (limit integral term)
    if (pidOutput>255){
      Ki=0;
    }
    integralTerm += Ki * error;

    derivative = error - previousError;
    float derivativeTerm = Kd * derivative;
    
    previousError = error;
    
    pidOutput = proportional + integralTerm + derivativeTerm;
    
    applyPIDToMotors(pidOutput);
  }
  void applyPIDToMotors(int pidOutput) {
    /*
    * Applies PID output to adjust motor speeds
    * Positive output = turn right, Negative output = turn left
    */
    int leftMotorSpeed = (baseSpeed - abs(pidOutput)) - pidOutput;
    int rightMotorSpeed = (baseSpeed - abs(pidOutput)) + pidOutput;
    // Constrain speeds to valid PWM range
    leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);
    if (abs(error)>=5){
      if(rightMotorSpeed<leftMotorSpeed)
        rightMotorSpeed=0;
      else
        leftMotorSpeed=0;
    }
    // Set motor speeds
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  }
  void setMotorSpeed(int leftSpeed, int rightSpeed) {
    /*
    * Controls 4 PWM motors based on calculated speeds
    * Assumes motor configuration:
    * - Motor A: Left side (pins A1, A2)
    * - Motor B: Right side (pins B1, B2)
    */
    
    // Left Motor Control
    if (leftSpeed > 0) {
      // Forward motion
      analogWrite(ML_F, leftSpeed);
      analogWrite(ML_B, 0);
    } else {
      // Backward motion (if needed for sharp turns)
      analogWrite(ML_F, 0);
      analogWrite(ML_B, abs(leftSpeed));
    }
    
    // Right Motor Control
    if (rightSpeed > 0) {
      // Forward motion
      analogWrite(MR_F, rightSpeed);
      analogWrite(MR_B, 0);
    } else {
      // Backward motion (if needed for sharp turns)
      analogWrite(MR_F, 0);
      analogWrite(MR_B, abs(rightSpeed));
    }
  }
  int activeCount() {
    readSensors();
    int count = 0;
    for (int i = 0; i < 6; i++) {
      if (sensorCalibrated[i] == 1) {
        count++;
      }
    }
    return count;
  }
  void setup() {
    // Initialize motor pins as OUTPUT
    pinMode(ML_F, OUTPUT);
    pinMode(ML_B, OUTPUT);
    pinMode(MR_F, OUTPUT);
    pinMode(MR_B, OUTPUT);
    pinMode(LED,OUTPUT);
    pinMode(btnpin, INPUT);
    // Initialize serial communication for debugging
    Serial.begin(115200);
    while (!Serial) {delay(10);}   
    for (int i = 0; i < 6; i++) {
      pinMode(sensorPins[i], INPUT);
    }
    // Calibrate sensors at startup
    calibrateSensors(); 
    Serial.println("Line Follower Robot Ready!");
    delay(1000);
    while(digitalRead(btnpin)==1){delay(10);}
    t0=millis();
  }
  void stopMotors() {
    analogWrite(ML_F, 0);
    analogWrite(ML_B, 0);
    analogWrite(MR_F, 0);
    analogWrite(MR_B, 0);
  }
  void readSensorsinv(){
    /*
    * Reads all 6 sensors and converts to binary values
    * Applies calibration to get consistent readings
    */
    for (int i = 0; i < 6; i++) {
      int rawValue = analogRead(sensorPins[i]);
      
      // Apply calibration: map to 0-1000 range
      int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], 0, 4095);
      
      // Constrain values and convert to binary (0 or 1)
      //calibratedValue = constrain(calibratedValue, 0, 1000);
      sensorCalibrated[i] = (calibratedValue > (Max(sensorMin)+Min(sensorMax))/2) ? 0 : 1; 
    }
  }
  void loop() {
    // Main control loop
    readSensors();           // Read all 6 sensor values
    unsigned int t1=millis();
    while(millis() - t0 >=200 && millis() - t0 <= 910){
      readSensors();
      baseSpeed=240;
      Kp=47;
      weights = {0, 0, 2, -2, 0, 0};
      k++;
      calculateError();
      calculatePID();
      delay(5);
    }
    baseSpeed=130;
    Kp=43;
    while(k!=0&& millis()-t0>=1500&&millis()-t0<=4800){
      readSensors();
      weights = {1, 1, 1, -1, -2,-3};
      calculateError();
      calculatePID();
      delay(5);
    }
    while(millis()-t0>4800&&millis()-t0<=8500){
      readSensors();
      weights = {10, 3, 1, -1, -3, -10};
      baseSpeed=200;
      Kp=45;
      calculateError();
      calculatePID();
      delay(5);
      k1++;
    }
    
    // partie des couleur inversée
    while(sensorCalibrated[0]==1&&sensorCalibrated[1]==1&&sensorCalibrated[4]==1&&sensorCalibrated[5]==1&&(sensorCalibrated[2]==0||sensorCalibrated[3]==0)){
      readSensorsinv();
      while(!(sensorCalibrated[0]==1&&sensorCalibrated[1]==1&&sensorCalibrated[4]==1&&sensorCalibrated[5]==1&&(sensorCalibrated[2]==0||sensorCalibrated[3]==0))){
        readSensorsinv();
        weights = {5, 7, 1, -1, -3, -5};
        calculateError();
        calculatePID();
        delay(5);
      }
      if (stop==0){
        stop = 1;
      }
      readSensors();
    }
    if (stop>=1 && khrouj==0){
      
      weights = {5, 3, 1, -1, -3, -5};
      khrouj++;
      t_lekhra = millis();
    }
    while(millis()-t_lekhra <= 800 && khrouj!=0){
      readSensors();
      baseSpeed = 230;
      calculateError();
      calculatePID();
      delay(5);
      if (test_tri9_twil == false){
        test_tri9_twil = true;
      }
    }
    while(test_tri9_twil==true){
      readSensors();
      baseSpeed = 135;
      calculateError();
      calculatePID();
      delay(5);
      if (stop==1 && activeCount()==6){
        delay(300);
        stopMotors();
        while(true){};
      }
      
      }
    digitalWrite(LED,LOW);
    weights = {5, 3, 1, -1, -3, -5};
    Serial.println(); // nouvelle ligne*/
    calculateError();
    calculatePID();
    delay(5); // Small delay for stability
  }