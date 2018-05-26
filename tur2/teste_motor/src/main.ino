
void setup(){
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
}

void loop(){
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    analogWrite(3, 50);
}
