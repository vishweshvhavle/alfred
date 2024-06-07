#define LPWM1 2
#define RPWM1 3
#define LPWM2 4
#define RPWM2 5
#define LPWM3 10
#define RPWM3 11
#define LPWM4 12
#define RPWM4 13

int myArray[9]; //this value is the upgradable data
byte* ddata = reinterpret_cast<byte*>(&myArray); // pointer for transferData()
size_t pcDataLen = sizeof(myArray);
bool newData=false;

void bot_stop() {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
    analogWrite(RPWM4, 0);
    analogWrite(LPWM4, 0);

  Serial.println("Alfred has stopped");
}

void run_bot(int pwmVals[9]) {
    analogWrite(RPWM1, constrain(pwmVals[0], 0, 255));
    analogWrite(LPWM1, constrain(pwmVals[1], 0, 255));
    analogWrite(RPWM2, constrain(pwmVals[2], 0, 255));
    analogWrite(LPWM2, constrain(pwmVals[3], 0, 255));
    analogWrite(RPWM3, constrain(pwmVals[4], 0, 255));
    analogWrite(LPWM3, constrain(pwmVals[5], 0, 255));
    analogWrite(RPWM4, constrain(pwmVals[6], 0, 255));
    analogWrite(LPWM4, constrain(pwmVals[7], 0, 255));
  
    Serial.println("Alfred is in motion");
}

void checkForNewData () {
    if (Serial.available() >= pcDataLen && newData == false) {
        byte inByte;
        for (byte n = 0; n < pcDataLen; n++) {
            ddata[n] = Serial.read();
        }
        while (Serial.available() > 0) { // now make sure there is no other data in the buffer
             byte dumpByte =  Serial.read();
            //  Serial.println(dumpByte);
        }
        newData = true;
    }
}

void setup() {
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);

    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);

    pinMode(RPWM3, OUTPUT);
    pinMode(LPWM3, OUTPUT);

    pinMode(RPWM4, OUTPUT);
    pinMode(LPWM4, OUTPUT);

    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);

    Serial.begin(9600);
    Serial.println("Alfred in Operation");
}

void loop() {
    checkForNewData();
    if (newData == true) {
        newData = false;
        int checksum = myArray[8];
    if(checksum == 100) {
          Serial.println("Alfred read data");
          run_bot(myArray);
        } else {
          Serial.println("Alfred detected error in checksum");
            bot_stop();
        }
    } 
}
