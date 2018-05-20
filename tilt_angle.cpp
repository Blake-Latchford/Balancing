#include "tilt_angle.h"
#include "MPU6050_6Axis_MotionApps20.h"

static MPU6050 mpu;

TiltAngle::TiltAngle()
: current_tilt_angle_in_rad(NAN)
, new_tilt_anlge_available(false)
, state(State::init)
{}

void TiltAngle::setup() {
  pinMode(TILT_ANGLE_INTERRUPT_PIN, INPUT);
}

void TiltAngle::loop( double& current_tilt_angle_in_rad, bool& new_tilt_anlge_available ) {
  switch(state)
  {
    case State::init:
      state_init();
      break;
    case State::warm_up:
      state_warm_up();
      break;
    case State::running:
      state_running();
      break;
  }

  current_tilt_angle_in_rad = get_tilt_angle_in_rad();
  new_tilt_anlge_available = get_and_reset_tilt_angle_available();
}

void TiltAngle::interrupt_handler() {
    mpu_data_ready = true;
}

void TiltAngle::state_init() {
  new_tilt_anlge_available = false;
  current_tilt_angle_in_rad = NAN;
  
  mpu.initialize();

  if(!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    mpu.reset();
    delay(50);
    return;
  }

  int dmp_init_status = mpu.dmpInitialize();
  if( 0 != dmp_init_status ) {
    Serial.print(F("DMP init failed with status:"));
    Serial.println( dmp_init_status );
    mpu.reset();
    delay(50);
    return;
  }
  
  mpu.setXAccelOffset(-4268);
  mpu.setYAccelOffset(-1731);
  mpu.setZAccelOffset(1242);
  mpu.setXGyroOffset(16);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(-33);
  
  mpu.setDMPEnabled(true);
  
  (void)mpu.getIntStatus();

  packet_size = mpu.dmpGetFIFOPacketSize();
  
  mpu.setDMPEnabled(true);

  state = State::warm_up;
  Serial.println("MPU Init Completed");
}

void TiltAngle::state_warm_up() {
  static const int startup_delay_in_ms = 1000;
  static unsigned long timer = 0;
  
  new_tilt_anlge_available = false;
  current_tilt_angle_in_rad = NAN;

  if( waiting_for_data() ) {
    return;
  }

  mpu.getFIFOBytes(fifo_buffer, packet_size);

  if( 0 == timer ){
    timer = millis();
  }
  else if( millis() - timer > startup_delay_in_ms ) {
    Serial.println("state_mpu_warm_up complete.");
    timer = 0;
    state = State::running;
  }
}

void TiltAngle::state_running() {
  Quaternion quant;
  VectorFloat gravity;
  float ypr[3];

  if(waiting_for_data()) {
    new_tilt_anlge_available = false;
    return;
  }

  mpu.getFIFOBytes(fifo_buffer, packet_size);
  mpu.dmpGetQuaternion(&quant, fifo_buffer);
  mpu.dmpGetGravity(&gravity, &quant);
  mpu.dmpGetYawPitchRoll(ypr, &quant, &gravity);
  
  
  new_tilt_anlge_available = true;
  current_tilt_angle_in_rad = ypr[1];
}

bool TiltAngle::waiting_for_data() {
  static int fifo_count = 0;
  int mpu_interrupt_status = 0;
  
  // If the data isn't ready, return without change.
  if( !mpu_data_ready ) {
    return true;
  }
  mpu_data_ready = false;

  mpu_interrupt_status = mpu.getIntStatus();
  fifo_count = mpu.getFIFOCount();
  
  if ((mpu_interrupt_status & 0x10) || fifo_count == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return true;
  }

  if((mpu_interrupt_status & 0x02) == 0x00) {
    return true;
  }

  while (fifo_count < packet_size) {
    fifo_count = mpu.getFIFOCount();
  }

  fifo_count -= packet_size;

  return false;
}

double TiltAngle::get_tilt_angle_in_rad() {
  return current_tilt_angle_in_rad;
}
bool TiltAngle::get_and_reset_tilt_angle_available() {
  bool tmp = new_tilt_anlge_available;
  new_tilt_anlge_available = false;
  return tmp;
}

