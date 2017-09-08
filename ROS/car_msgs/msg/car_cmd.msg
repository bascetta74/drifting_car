# car_cmd message
# Commands from radio to car or from control system to car

uint8 STATE_SAFE=0
uint8 STATE_MANUAL=1
uint8 STATE_AUTOMATIC=2
uint8 STATE_HALT=3

float64 speed_ref
float64 steer_ref
uint8   state
