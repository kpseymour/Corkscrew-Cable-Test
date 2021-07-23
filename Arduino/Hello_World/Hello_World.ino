int i = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(i==1){
    Serial.println("Hello World");
    i++;
  }
}
