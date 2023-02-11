#include <Arduino.h>
#include <ESP32Servo.h>
#include <SerialCommands.h>

const int serial_baudrate=115200;

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
#if defined(ARDUINO_ESP32S2_DEV)
const int servoPin = 17;
const int servoFBPin = 10;        // GPIO pin used to connect the potentiometer (analog in)
#else
const int servoPin = 18;
const int servoFBPin = 34;        // GPIO pin used to connect the potentiometer (analog in)
#endif

const int ADC_Max = 4096;     // This is the default ADC max value on the ESP32 (12 bit ADC width);
                              // this width can be set (in low-level oode) from 9-12 bits, for a
                              // a range of max values of 512-4096

const int servo_min_pos=0;
const int servo_max_pos=180;
const int servo_pwm_period=50;
const int servo_pwm_period_min=500;
const int servo_pwm_period_max=2500;

int servo_min_pos_fb=0;
int servo_max_pos_fb=0;
int sweep_active=0;
int sweep_pos=0;
int sweep_dir=0;
int sweep_pos1=0;
int sweep_pos2=0;
int sweep_step=0;
int sweep_period=0;
unsigned long int sweep_millis = 0;


Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 0;    // variable to store the servo position

int isNumber(char s[])
{
    const char * c=&s[0];
	while (*c)
        if (!isdigit(*c++))
              return 0;
    return 1;
}

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	sweep_active=0;
	sender->GetSerial()->print("NOK Command unknown [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}



// POS <pos>, where pos >= servo_min_pos and pos <= servo_max_pos
// e.g. POS 10, POS 170
void cmd_servo_position(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter
	char* pwm_str = 0;
	sweep_active=0;
	pwm_str = sender->Next();
	if (pwm_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing Position");
		return;
	}	
	do 
	{
		if (!isNumber(pwm_str))
		{
			sender->GetSerial()->println("NOK pos arg not a number or a neg number");
			return;
		}
		int new_pos = atoi(pwm_str);
		if ( new_pos < servo_min_pos or new_pos > servo_max_pos) 
		{
			sender->GetSerial()->println("NOK requested pos outside min/max pos");
			return;
		}
		myservo.write(new_pos);
		sender->GetSerial()->print("OK servo pos commanded to ");
		sender->GetSerial()->println(new_pos);
	}
	while ((pwm_str = sender->Next()) != NULL);	
}



// SWEEP <pos1> <pos2> <step> <period> where <pos1> and <pos2> >= servo_min_pos and <= servo_max_pos, <step> in degrees, <period> the period of the sweep 
// e.g.
// SWEEP 40 60 1 300
// SWEEP 40 47 7 300
void cmd_servo_sweep(SerialCommands* sender)
{	
	//Note: Every call to Next moves the pointer to next parameter
	char* arg_str = sender->Next();
	if (arg_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing pos1 arg");
		return;
	}
	if (!isNumber(arg_str))
	{
		sender->GetSerial()->println("NOK pos1 arg not a number or a neg number");
		return;
	}
	sweep_pos1 = atoi(arg_str);
	if ( sweep_pos1 < servo_min_pos or sweep_pos1 > servo_max_pos) 
	{
		sender->GetSerial()->println("NOK requested pos1 outside min/max pos");
		return;
	}

	arg_str = sender->Next();
	if (arg_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing pos2 arg");
		return;
	}
	if (!isNumber(arg_str))
	{
		sender->GetSerial()->println("NOK pos2 arg not a number or a neg number");
		return;
	}
	sweep_pos2 = atoi(arg_str);
	if ( sweep_pos2 < servo_min_pos or sweep_pos2 > servo_max_pos) 
	{
		sender->GetSerial()->println("NOK requested pos2 outside min/max pos");
		return;
	}

	if (sweep_pos1 >= sweep_pos2)
	{
		sender->GetSerial()->println("NOK requested pos1 bigger or equal to pos2");
		return;
	}

	arg_str = sender->Next();
	if (arg_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing step arg");
		return;
	}
	if (!isNumber(arg_str))
	{
		sender->GetSerial()->println("NOK step arg not a number or a neg number");
		return;
	}
	sweep_step = atoi(arg_str);
	if ( sweep_step < 0 or sweep_step > abs(sweep_pos2-sweep_pos1)) 
	{
		sender->GetSerial()->println("NOK requested step less than zero or bigger than abs(pos2-pos1)");
		return;
	}	

	arg_str = sender->Next();
	if (arg_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing period arg");
		return;
	}
	if (!isNumber(arg_str))
	{
		sender->GetSerial()->println("NOK period arg not a number or a neg number");
		return;
	}
	sweep_period = atoi(arg_str);
	if ( sweep_period < 0 or sweep_period > 5000) 
	{
		sender->GetSerial()->println("NOK requested period less than zero or bigger than 5000");
		return;
	}

	sender->GetSerial()->print("OK activating SWEEP ");	
	sender->GetSerial()->print(sweep_pos1);
	sender->GetSerial()->print(", ");
	sender->GetSerial()->print(sweep_pos2);
	sender->GetSerial()->print(", ");
	sender->GetSerial()->print(sweep_step);
	sender->GetSerial()->print(", ");
	sender->GetSerial()->println(sweep_period);

	sweep_active=1;		
	myservo.write(sweep_pos1);
	sweep_pos = sweep_pos1;
	sweep_dir=sweep_pos2>sweep_pos1?1:-1;
	myservo.write(sweep_pos);
	sweep_millis=millis();
}

void update_servo_sweep() 
{
	if (sweep_active and (millis()-sweep_millis)>sweep_period)
  	{
		sweep_millis=millis();
		if (sweep_dir>0)
		{
		  if (sweep_pos<sweep_pos2)
		  {
		    sweep_pos+=sweep_step*sweep_dir;
			if (sweep_pos>sweep_pos2)
				sweep_pos=sweep_pos2;
		  }
		  else 
		  {
		    sweep_dir=-sweep_dir;
		  }
		}

		if (sweep_dir<0)
		{
		  if (sweep_pos>sweep_pos1)
		  {
		    sweep_pos+=sweep_step*sweep_dir;
			if (sweep_pos<sweep_pos1)
				sweep_pos=sweep_pos1;
		  }
		  else 
		  {
		    sweep_dir=-sweep_dir;
			sweep_pos+=sweep_step*sweep_dir;
			if (sweep_pos>sweep_pos2)
				sweep_pos=sweep_pos2;
		  }
		}
		myservo.write(sweep_pos);
		//Serial.print(sweep_pos);
		//Serial.print(" ");
  	}
}

// CALIBRATE <wait_ms>, where <wait_ms> = wait milisecs for servo to reach each position
// e.g. CALIBRATE 2000
void cmd_servo_feedback_calibrate(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter
	char* arg_str=0;
	sweep_active=0;
	arg_str = sender->Next();	
	if (arg_str == NULL)
	{
		sender->GetSerial()->println("NOK Missing arg");
		return;
	} 
	else 

	{
		if (!isNumber(arg_str))
		{
			sender->GetSerial()->println("NOK wait_ms arg not a number or a neg number");
			return;
		}
		int wait_ms = atoi(arg_str);
		if ( wait_ms < 0 or wait_ms > 10000) 
		{
			sender->GetSerial()->println("NOK wait_ms arg less than zero or greater than 10000");
			return;
		}

		myservo.write(servo_min_pos);
		delay(wait_ms);                          // wait for the servo to get there
		servo_min_pos_fb = analogRead(servoFBPin);            // read the value of the potentiometer (value between 0 and 1023)
		servo_min_pos_fb = map(servo_min_pos_fb, 0, ADC_Max, servo_min_pos, servo_max_pos);     // scale it to use it with the servo (value between 0 and 180)
		
		myservo.write(servo_max_pos);
		delay(wait_ms);                          // wait for the servo to get there
		servo_max_pos_fb = analogRead(servoFBPin);            // read the value of the potentiometer (value between 0 and 1023)
		servo_max_pos_fb = map(servo_max_pos_fb, 0, ADC_Max, servo_min_pos, servo_max_pos);     // scale it to use it with the servo (value between 0 and 180) 

		if (servo_min_pos_fb >= servo_max_pos_fb) 
		{
			sender->GetSerial()->println("NOK min_pos_fb bigger or equal to max_pos_fb");
			return;
		}
		sender->GetSerial()->print("OK min:");
		sender->GetSerial()->print(servo_min_pos_fb);
		sender->GetSerial()->print(", max:");
		sender->GetSerial()->println(servo_max_pos_fb);
	}
	arg_str = sender->Next();
	if (arg_str != NULL)
	{
		sender->GetSerial()->println("NOK Too many args");
		return;
	} 
}


char serial_command_buffer_[128];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");
SerialCommand  cmd_servo_position_("POS", cmd_servo_position);
SerialCommand  cmd_servo_feedback_calibrate_("CALIBRATE", cmd_servo_feedback_calibrate);
SerialCommand  cmd_servo_sweep_("SWEEP", cmd_servo_sweep);



void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(servo_pwm_period);    // standard 250 hz servo
  myservo.attach(servoPin, servo_pwm_period_min, servo_pwm_period_max ); // attaches the servo on pin 18 to the servo object  

  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_servo_position_); 
  serial_commands_.AddCommand(&cmd_servo_feedback_calibrate_);
  serial_commands_.AddCommand(&cmd_servo_sweep_);

  Serial.begin(serial_baudrate);
  Serial.println("READY");
}

void loop() {
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);

  serial_commands_.ReadSerial();
  update_servo_sweep();
  delay(1); 
}