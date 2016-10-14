float t1 = 0;
float t2 = 0;
float t3 = 0;
float t4 = 0;

void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);  
  Serial.println("--- Print Torque Commands ---");
  Serial.println(); 
}
//--(end setup )---

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  if (Serial.available() > 0){

        t1 = Serial.parseFloat();
        t2 = Serial.parseFloat();
        t3 = Serial.parseFloat();
        t4 = Serial.parseFloat();

        Serial.print(t1);
        Serial.print(' '); 
        Serial.print(t2);
        Serial.print(' '); 
        Serial.print(t3);
        Serial.print(' ');
        Serial.println(t4); 
    }

  // END Serial Available
}
