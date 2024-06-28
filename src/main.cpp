#include <Arduino.h>
#include <Wire.h>
#include <Ps3Controller.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define I2C_ISSI_ADDR 0x3C // Replace with your device address

typedef struct {
    uint8_t request_flag;
    uint8_t channel;
    uint8_t value1;
    uint8_t value2;
} IS31FL3236A_update_2_channels_params_t;

IS31FL3236A_update_2_channels_params_t IS31FL3236A_update_2_channels_data;

// Define a struct to hold joystick events
typedef struct {
    enum { FORWARD, DIRECT, STOP,LEFT,RIGHT } action;
} JoystickEvent_t;

QueueHandle_t joystickEventQueue;

void IS31FL3236A_init() {
    delay(1);

    // Shutdown register (00h)
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x0);
    Wire.write(1); // Normal operation
    Wire.endTransmission(1);
    delay(1);

    // LED control register (26h...49h)
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x26);
    
    // PRO and PRODUINO have 3 H-bridges connected to channels 1-6
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.endTransmission(1);

    // PRODUINO has the RGB LED on channels 16, 17, 18
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x26 + 31);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
  
    Wire.endTransmission(1);

    // Global control register (4Ah)
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x4A);
    Wire.write(0); // Normal operation
    Wire.endTransmission(1);

    // PWM registers (01h~24h)
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x01);
    
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.write(1);
    Wire.endTransmission(1);

    // Update register (25h)
    Wire.beginTransmission(I2C_ISSI_ADDR);
    Wire.write(0x25);
    Wire.write(0);
    Wire.endTransmission(1);
}

void IS31FL3236A_update_2_channels(uint8_t channel, uint8_t value1, uint8_t value2) {
    if (channel < 36) {
        Wire.beginTransmission(I2C_ISSI_ADDR);
        Wire.write(channel);
        Wire.write(value1);
        Wire.write(value2);
        Wire.endTransmission();

        Wire.beginTransmission(I2C_ISSI_ADDR);
        Wire.write(0x25);
        Wire.write(0);
        Wire.endTransmission();

        Serial.printf("Updated channels: %d and %d with values: %d, %d\n", channel, channel+1, value1, value2);
    }
}

void Stop() {
    IS31FL3236A_update_2_channels(1, 0, 0);
    IS31FL3236A_update_2_channels(3, 0, 0);
    IS31FL3236A_update_2_channels(32, 0, 0);
    IS31FL3236A_update_2_channels(34, 0, 0);   
    delay(1000); 
}

void Direct() {
    IS31FL3236A_update_2_channels(1, 250, 0);
    IS31FL3236A_update_2_channels(3, 250, 0);
    IS31FL3236A_update_2_channels(32, 250, 0);
    IS31FL3236A_update_2_channels(34, 250, 0);
}

void  Right(){
    IS31FL3236A_update_2_channels(1, 0, 250);
    IS31FL3236A_update_2_channels(3, 250, 0);
    IS31FL3236A_update_2_channels(32, 0, 250);
    IS31FL3236A_update_2_channels(34, 250, 0);
}

void Forward() {
    IS31FL3236A_update_2_channels(1, 0, 250);
    IS31FL3236A_update_2_channels(3, 0, 250);
    IS31FL3236A_update_2_channels(32, 0, 250);
    IS31FL3236A_update_2_channels(34, 0, 250);
}

void  Left(){
    IS31FL3236A_update_2_channels(1, 250, 0);
    IS31FL3236A_update_2_channels(3, 0, 250);
    IS31FL3236A_update_2_channels(32, 250, 0);
    IS31FL3236A_update_2_channels(34, 0, 250);
}
bool cross_presed=false;
void notify() {
    JoystickEvent_t event;
    ////////FORWAD
    if (Ps3.event.button_down.cross) {
        Serial.println("Cross pressed - Forward");
        event.action = JoystickEvent_t::FORWARD;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }

    if (Ps3.event.button_up.cross) {
        Serial.println("Cross released - Stop");
        event.action = JoystickEvent_t::STOP;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }

  /////////DIRECT
    if (Ps3.event.button_down.triangle) {
        Serial.println("Triangle pressed - Direct");
        event.action = JoystickEvent_t::DIRECT;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }

    
    if (Ps3.event.button_up.triangle) {
        Serial.println("Triangle released - Stop");
        event.action = JoystickEvent_t::STOP;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }
    ///LEFT
    
    if (Ps3.event.button_down.square) {
        Serial.println("Square pressed - Left");
        event.action = JoystickEvent_t::LEFT;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }

     if (Ps3.event.button_up.square) {
        Serial.println("Triangle released - Stop");
        event.action = JoystickEvent_t::STOP;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }
    /////RIGHT


    if (Ps3.event.button_down.circle) {
        Serial.println("Circle pressed - Right");
        event.action = JoystickEvent_t::RIGHT;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }


    if (Ps3.event.button_up.circle) {
        Serial.println("Triangle released - Stop");
        event.action = JoystickEvent_t::STOP;
        xQueueSend(joystickEventQueue, &event, portMAX_DELAY);
    }
}

void onConnect() {
    Serial.println("Connected.");
}

void motorControlTask(void *pvParameters) {
    JoystickEvent_t event;
    while (1) {
        if (xQueueReceive(joystickEventQueue, &event, portMAX_DELAY)) {
            switch (event.action) {
                case JoystickEvent_t::FORWARD:
                    Forward();
                    break;
                case JoystickEvent_t::DIRECT:
                    Direct();
                    break;
                case JoystickEvent_t::STOP:
                    Stop();
                    break;
                case JoystickEvent_t::LEFT:
                   Left();
                    break;
                case JoystickEvent_t::RIGHT: 
                   Right() ;
            }
        }
    }
}

void setup() {
    Wire.begin(12, 13); // Initialize I2C
    Serial.begin(115200); // Initialize Serial for debugging

    #if (HARDWARE_BOARD == BOARD_PRODUINO)
        IS31FL3236A_init();
    #endif

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("08:d1:f9:e8:1d:f6");

    Serial.println("Ready.");

    // Create the joystick event queue
    joystickEventQueue = xQueueCreate(2, sizeof(JoystickEvent_t));
    if (joystickEventQueue == NULL) {
        Serial.println("Error creating the queue");
        while (1);
    }

    // Create the motor control task
    xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // The main loop can remain empty since the tasks handle the functionality
   
}
