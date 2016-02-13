// https://github.com/mathertel/DMXSerial
#include <DMXSerial.h>

// Consts
const int CLOSED = 0;
const int OPEN = 1;
const int ERROR_LED_PIN = 13;
const int NUMBER_OF_POOFERS = 16;
const int POOFER_START_CHANNEL = 8;
const unsigned long MAX_POOF_DURATION_MS = 256;
const unsigned long MAX_REST_DURATION_MS = 1536;
const unsigned long MIN_POOF_DURATION_MS = 30;
const unsigned long MIN_REST_DURATION_MS = 45;
const int           DMX_RESET_PIN = 9;
const unsigned long DMX_RESET_TIMEOUT_MS = 30000;
const unsigned long DMX_TIMEOUT_MS = 1000;

unsigned long dmx_last_packet;

// Controller modes
const int MODE_OFF = 0;
const int MODE_CHASE_UP_1_8 = 2;
const int MODE_CHASE_DOWN_1_8 = 4;
const int MODE_CHASE_UP_DOWN_1_8 = 6;
const int MODE_CHASE_DOWN_UP_1_8 = 8;
const int MODE_CHASE_IN_OUT_1_8 = 10;
const int MODE_CHASE_OUT_IN_1_8 = 12;
const int MODE_CHASE_IN_1_8 = 14;
const int MODE_CHASE_OUT_1_8 = 16;
const int MODE_ALTERNATE = 18;
const int MODE_FIRE_ALL = 200;
const int MODE_RAW_DMX = 254;

// Poofer Controller Channels
const int CHANNEL_ARMED = 1;
const int CHANNEL_POOF_MS = 2;
const int CHANNEL_REST_MS = 3;
const int CHANNEL_REPEAT_PATTERNS = 4;
const int CHANNEL_MODE = 5;
const int CHANNEL_DATA_VALUE_1 = 6;
const int CHANNEL_DATA_VALUE_2 = 7;
const int CHANNEL_FIRE_1 = 8;
const int CHANNEL_FIRE_2 = 9;
const int CHANNEL_FIRE_3 = 10;
const int CHANNEL_FIRE_4 = 11;
const int CHANNEL_FIRE_5 = 12;
const int CHANNEL_FIRE_6 = 13;
const int CHANNEL_FIRE_7 = 14;
const int CHANNEL_FIRE_8 = 15;
const int CHANNEL_FIRE_9 = 16;
const int CHANNEL_FIRE_10 = 17;
const int CHANNEL_FIRE_11 = 18;
const int CHANNEL_FIRE_12 = 19;
const int CHANNEL_FIRE_13 = 20;
const int CHANNEL_FIRE_14 = 21;
const int CHANNEL_FIRE_15 = 22;
const int CHANNEL_FIRE_16 = 23;

// GLOBAL VARIABLES
int armed = 0;
unsigned long poof_ms = 0;
unsigned long rest_ms = 0;
boolean repeat_patterns = false;
int mode = 0;
int data_1 = 0;
int data_2 = 0;

// Stores the ending timestamp of each poof event
unsigned long poofs[] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

// Stores the ending timestamp of each rest event
unsigned long rests[] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

void setup() { 

  // Turn off the power to the DMX shield
  pinMode(DMX_RESET_PIN, OUTPUT);
  digitalWrite(DMX_RESET_PIN, LOW);

  // Status LED (FAST BLINK = ERROR, SLOW BLINK = NO ERROR)
  pinMode(ERROR_LED_PIN, OUTPUT);

  // Startup Status LED: SOLID RED
  digitalWrite(ERROR_LED_PIN, LOW); 

  // 16 RELAYS, 1-16:
  //
  // PORTA = 1-8
  // PORTC = 9-16
  //
  // All pins are outputs
  DDRA = 0b11111111;
  DDRC = 0b11111111;

  // Everything off to start
  // 1/HIGH = SOLENOID CLOSED
  // 0/LOW  = SOLENOID OPEN
  PORTA = 0b11111111;
  PORTC = 0b11111111;

  // Setup max DMX channels
  DMXSerial.maxChannel(24);

  // Set up as a DMX receiver  
  DMXSerial.init(DMXReceiver);

  // Turn on the DMX Shield
  delay(1000);
  digitalWrite(DMX_RESET_PIN, HIGH);

}

// function to reset the device
void(* ResetDevice) (void) = 0;

// Main loop 
void loop() { 

  // Calculate how long no data packet was received
  dmx_last_packet = DMXSerial.noDataSince();

  // Read the current mode's value from DMX
  armed = constrain(DMXSerial.read(CHANNEL_ARMED), 0, 255);

  // If DMX down or system isn't armed, turn off the poofers
  if (armed < 254 || dmx_last_packet >= DMX_TIMEOUT_MS) {

    // DMX is down or the system isn't armed
    // Turn everything off by default
    PORTA = 0b11111111;
    PORTC = 0b11111111;

    // Status RED LED fast blink
    if ((millis() / 100) % 2) digitalWrite(ERROR_LED_PIN, HIGH); 
    else digitalWrite(ERROR_LED_PIN, LOW); 

    // Reset the device if DMX hasn't worked in reset_timeout ms  
    if (dmx_last_packet > DMX_RESET_TIMEOUT_MS) {

      // Try to restart DMX
      ResetDevice();

    }

    // No need to keep going if DMX is down
    return;

  } // if  

  // DMX is working let's do things


  // Read Global DMX values
  int new_mode = constrain(DMXSerial.read(CHANNEL_MODE), 0, 255);
  poof_ms = map(constrain(DMXSerial.read(CHANNEL_POOF_MS), 0, 255), 0, 255, MIN_POOF_DURATION_MS, MAX_POOF_DURATION_MS);
  rest_ms = map(constrain(DMXSerial.read(CHANNEL_REST_MS), 0, 255), 0, 255, MIN_POOF_DURATION_MS, MAX_REST_DURATION_MS);
  data_1 = constrain(DMXSerial.read(CHANNEL_DATA_VALUE_1), 0, 255);
  data_2 = constrain(DMXSerial.read(CHANNEL_DATA_VALUE_2), 0, 255);
  if (constrain(DMXSerial.read(CHANNEL_REPEAT_PATTERNS), 0, 255) <= 127) repeat_patterns = false;
  else repeat_patterns = true;
    
  // Are we switching modes?
  if (new_mode != mode) {

    // Yes!
    // Turn off the poofers so we can start fresh
    ResetPoofsRests();

    // Setup mode
    switch (new_mode) {
  
      // Chase Up
      case MODE_CHASE_UP_1_8:
      case MODE_CHASE_UP_DOWN_1_8:
        ChaseUpInit();
        break;    
  
      // Chase down
      case MODE_CHASE_DOWN_1_8:
      case MODE_CHASE_DOWN_UP_1_8:
        ChaseDownInit();
        break;    

      // Alternate
      case MODE_ALTERNATE: 
        AlternateInit();
        break;

      // Chase in
      case MODE_CHASE_IN_1_8:
      case MODE_CHASE_IN_OUT_1_8:
        ChaseInInit();
        break;

      // Chase out
      case MODE_CHASE_OUT_1_8:
      case MODE_CHASE_OUT_IN_1_8:
        ChaseOutInit();
        break;

    }

    // Update mode variable    
    mode = new_mode;

  }

  // Perform the actions for the mode
  switch (mode) {

    case MODE_CHASE_UP_1_8: 
      ChaseUp_1_8(); 
      break;

    case MODE_CHASE_DOWN_1_8: 
      ChaseDown_1_8(); 
      break;

    case MODE_CHASE_UP_DOWN_1_8: 
      ChaseUpDown_1_8(); 
      break;

    case MODE_CHASE_DOWN_UP_1_8: 
      ChaseDownUp_1_8(); 
      break;

    case MODE_CHASE_IN_1_8:
      ChaseIn_1_8();
      break;

    case MODE_CHASE_OUT_1_8:
      ChaseOut_1_8();
      break;

    case MODE_CHASE_IN_OUT_1_8:
      ChaseInOut_1_8();
      break;

    case MODE_CHASE_OUT_IN_1_8:
      ChaseOutIn_1_8();
      break;

    case MODE_ALTERNATE: 
      Alternate(); 
      break;

    case MODE_FIRE_ALL: 
      PoofAll(); 
      break;

    case MODE_RAW_DMX:
      RawDMX();
      break;

    case MODE_OFF:
    default: 
      PORTA = 0b11111111;
      PORTC = 0b11111111;
      return;
      break;

  }

  // Display status
  digitalWrite(ERROR_LED_PIN, HIGH); 

}

// MODES

int chase_index = 0;
int chase_direction = 1;

void ChaseInOut_1_8() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if ((chase_index == 3 && chase_direction == 1) ||
            (chase_index == 0 && chase_direction == -1)) {
          
          // if repeat is off and the mode is the same, we "rest"
          if (chase_index == 0 && chase_direction == -1 && repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;
  
          chase_direction *= -1;
  
        }
        
        chase_index += chase_direction;

      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  
  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseOutIn_1_8() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if ((chase_index == 3 && chase_direction == 1) ||
            (chase_index == 0 && chase_direction == -1)) {
          
          // if repeat is off and the mode is the same, we "rest"
          if (chase_index == 3 && chase_direction == 1 && repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;
  
          chase_direction *= -1;
  
        }
        
        chase_index += chase_direction;

      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  
  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseIn_1_8() {
  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if (chase_index == 3) {

          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
          
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;

          chase_index = 0;
          
        } else {

          chase_index += chase_direction;

        }
        
      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseOut_1_8() {
  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if (chase_index == 0) {

          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;

          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;

          chase_index = 3;

        } else {
          
          chase_index -= 1;
        
        }
        
      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);
  
}

void ChaseInInit() {
  chase_index = 0;
  chase_direction = 1;
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  if (DMXSerial.read(POOFER_START_CHANNEL + chase_index) >= 254) Solenoid(chase_index, OPEN);
}

void ChaseOutInit() {
  chase_index = 3;
  chase_direction = -1;
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  if (DMXSerial.read(POOFER_START_CHANNEL + chase_index) >= 254) Solenoid(chase_index, OPEN);
}

void ChaseUpInit() {
  chase_index = 0;
  chase_direction = 1;
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  if (DMXSerial.read(POOFER_START_CHANNEL + chase_index) >= 254) Solenoid(chase_index, OPEN);
}

void ChaseDownInit() {
  chase_index = 7;
  chase_direction = -1;
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;
  if (DMXSerial.read(POOFER_START_CHANNEL + chase_index) >= 254) Solenoid(chase_index, OPEN);
}

void ChaseUp_1_8() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if (chase_index == 7) {
          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;

          chase_index = 0;
          
        } else {
          
          chase_index += chase_direction;
        
        }
        
      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseUpDown_1_8() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if ((chase_index == 7 && chase_direction == 1) ||
            (chase_index == 0 && chase_direction == -1)) {
 
          // if repeat is off and the mode is the same, we "rest"
          if (chase_index == 0 && chase_direction == -1 && repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;
  
          chase_direction *= -1;
  
        }
        
        chase_index += chase_direction;

      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseDown_1_8() {
  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if (chase_index == 0) {
          
          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
            // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;

          chase_index = 7;

        } else {

          chase_index += chase_direction;

        }
        
      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

void ChaseDownUp_1_8() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[chase_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[chase_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(chase_index, CLOSED);

      // Is the device resting?
      if (rests[chase_index] < current_time) {

        if ((chase_index == 7 && chase_direction == 1) ||
            (chase_index == 0 && chase_direction == -1)) {

          // if repeat is off and the mode is the same, we "rest"
          if (chase_index == 7 && chase_direction == 1 && repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[chase_index] = 0;
          rests[chase_index] = 0;
  
          chase_direction *= -1;
  
        }
        
        chase_index += chase_direction;

      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + chase_index;

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[chase_index] = millis() + poof_ms;
  rests[chase_index] = poofs[chase_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(chase_index, OPEN);

}

int alternate_index = 0;
void AlternateInit() {
  alternate_index = 0;
}

void Alternate() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // Are we in the middle of a poof/rest event?
  if (poofs[alternate_index] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[alternate_index] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(alternate_index, CLOSED);
      Solenoid(alternate_index + 2, CLOSED);
      Solenoid(alternate_index + 4, CLOSED);
      Solenoid(alternate_index + 6, CLOSED);
      Solenoid(alternate_index + 8, CLOSED);
      Solenoid(alternate_index + 10, CLOSED);
      Solenoid(alternate_index + 12, CLOSED);
      Solenoid(alternate_index + 14, CLOSED);


      // Is the device resting?
      if (rests[alternate_index] < current_time) {

        if (alternate_index == 0) {

          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[alternate_index] = 0;
          rests[alternate_index] = 0;
  
          alternate_index = 1;
          
        } else {

          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;
  
          alternate_index = 0; 

        }
        
      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0
  
  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL + (alternate_index * 2);

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[alternate_index] = millis() + poof_ms;
  rests[alternate_index] = poofs[alternate_index] + rest_ms;

  // open if enabled
  if (DMXSerial.read(channel) >= 254) Solenoid(alternate_index, OPEN);
  if (DMXSerial.read(channel + 2) >= 254) Solenoid(alternate_index + 2, OPEN);
  if (DMXSerial.read(channel + 4) >= 254) Solenoid(alternate_index + 4, OPEN);
  if (DMXSerial.read(channel + 6) >= 254) Solenoid(alternate_index + 6, OPEN);
  if (DMXSerial.read(channel + 8) >= 254) Solenoid(alternate_index + 8, OPEN);
  if (DMXSerial.read(channel + 10) >= 254) Solenoid(alternate_index + 10, OPEN);
  if (DMXSerial.read(channel + 12) >= 254) Solenoid(alternate_index + 12, OPEN);
  if (DMXSerial.read(channel + 14) >= 254) Solenoid(alternate_index + 14, OPEN);

}

void PoofAll() {

  int channel;

  // Get current time
  unsigned long current_time = millis();

  // The STARTING DMX channel of the device
  channel = POOFER_START_CHANNEL;

  // Are we in the middle of a poof/rest event?
  if (poofs[0] != 0) {

    // This device is in a poof event

    // Are we done with the poof?      
    if (poofs[0] < current_time) {

      // Yes, we're done with the poof

      // CLOSE SOLENOID
      Solenoid(0, CLOSED);
      Solenoid(1, CLOSED);
      Solenoid(2, CLOSED);
      Solenoid(3, CLOSED);
      Solenoid(4, CLOSED);
      Solenoid(5, CLOSED);
      Solenoid(6, CLOSED);
      Solenoid(7, CLOSED);
      Solenoid(8, CLOSED);
      Solenoid(9, CLOSED);
      Solenoid(10, CLOSED);
      Solenoid(11, CLOSED);
      Solenoid(12, CLOSED);
      Solenoid(13, CLOSED);
      Solenoid(14, CLOSED);
      Solenoid(15, CLOSED);

      // Is the device resting?
      if (rests[0] < current_time) {

        // if repeat is off and the mode is the same, we "rest"
        if (repeat_patterns == false && mode == DMXSerial.read(CHANNEL_MODE)) return;

        // We're done resting.  Reset the poofs/rests timestamp.
        poofs[0] = 0;
        rests[0] = 0;

      } else {

        // Rest
        return;

      }

    } else {

      // We're poofing right now
      return;

    }

  } // if poofs[device] != 0

  // make sure we're still in poof all mode  
  if (MODE_FIRE_ALL != constrain(DMXSerial.read(CHANNEL_MODE), 0, 255)) {
    ResetPoofsRests();
    return;
  }

  // We have a new poof!
  // Update the poofs/rests arrays with the new timestamps
  poofs[0] = millis() + poof_ms;
  rests[0] = poofs[0] + rest_ms;

  if (DMXSerial.read(POOFER_START_CHANNEL + 0) >= 254) Solenoid(0, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 1) >= 254) Solenoid(1, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 2) >= 254) Solenoid(2, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 3) >= 254) Solenoid(3, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 4) >= 254) Solenoid(4, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 5) >= 254) Solenoid(5, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 6) >= 254) Solenoid(6, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 7) >= 254) Solenoid(7, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 8) >= 254) Solenoid(8, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 9) >= 254) Solenoid(9, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 10) >= 254) Solenoid(10, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 11) >= 254) Solenoid(11, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 12) >= 254) Solenoid(12, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 13) >= 254) Solenoid(13, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 14) >= 254) Solenoid(14, OPEN);
  if (DMXSerial.read(POOFER_START_CHANNEL + 15) >= 254) Solenoid(15, OPEN);

} // end PoofAll

void RawDMX() {

  int channel;

  for (int device = 0; device < NUMBER_OF_POOFERS; device++) {
    
    // The STARTING DMX channel of the device
    channel = POOFER_START_CHANNEL + device;

    // If enabled is not 255, the device can not poof 
    if (constrain(DMXSerial.read(channel), 0, 255) < 254) {

      // This device is not enabled.  Close the solenoid and 
      // reset the timestamp.  Then move to the next device. 

      // CLOSE SOLENOID
      Solenoid(device, CLOSED);

      // Nothing else to do.  Move to the next device.
      continue;

    } 

    // Get current time
    unsigned long current_time = millis();

    // Are we in the middle of a poof/rest event?
    if (poofs[device] != 0) {

      // This device is in a poof event

      // Are we done with the poof?      
      if (poofs[device] < current_time) {

        // Yes, we're done with the poof

        // CLOSE SOLENOID
        Solenoid(device, CLOSED);

        // Is the device resting?
        if (rests[device] < current_time) {

          // if repeat is off and the mode is the same, we "rest"
          if (repeat_patterns == false && constrain(DMXSerial.read(channel), 0, 255) >= 254) return;

          // We're done resting.  Reset the poofs/rests timestamp.
          poofs[device] = 0;
          rests[device] = 0;

        } else {

          // Rest.  Move to the next device.
          continue;

        }

      } else {

        // We're poofing right now.  Move to the next device.
        continue;

      }

    } // if poofs[device] != 0

    // If the poof_ms value is greater than 0, start a poof event
    if (constrain(DMXSerial.read(channel), 0, 255) >= 254) {

      // We have a new poof!
      // Update the poofs/rests arrays with the new timestamps
      poofs[device] = millis() + poof_ms;
      rests[device] = poofs[device] + rest_ms;

      Solenoid(device, OPEN);

    } // POOF EVENT

  } // END FOR

} // end RawDMX

void ResetPoofsRests() {
  
  PORTA = 0b11111111;
  PORTC = 0b11111111;
  
  for (int device = 0; device < NUMBER_OF_POOFERS; device++) {
    poofs[device] = 0;
    rests[device] = 0;
  }  
}

void Solenoid(int device, int state) {

  switch (state) {

    case OPEN:
    
      // Set the device's PORTA/C bit to 0 to enable the solenoid
      if (device < 8) PORTA &= ~(1 << device);
      else PORTC &= ~(1 << (device - 8));
        
      break;
  
    case CLOSED:
    default:
    
      // Set the device's PORTA/C bit to 1 to close the solenoid
      if (device < 8) PORTA |= (1 << device);
      else PORTC |= (1 << (device - 8));
        
      break;
  
  }

}
