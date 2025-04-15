#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "Arduino_BMI270_BMM150.h"
#include "model_data.h"

// Global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;
tflite::AllOpsResolver tflOpsResolver;
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a buffer to hold the model's input/output tensors
constexpr int kTensorArenaSize = 8 * 1024;
uint8_t tensorArena[kTensorArenaSize];

// Constants for data collection
const int NUM_SAMPLES = 119;  // 1 second of data at 119Hz
float accelX[NUM_SAMPLES];
float accelY[NUM_SAMPLES];
float accelZ[NUM_SAMPLES];
float gyroX[NUM_SAMPLES];
float gyroY[NUM_SAMPLES];
float gyroZ[NUM_SAMPLES];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize TensorFlow Lite
  tflModel = tflite::GetModel(model_data);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // Create interpreter
  tflInterpreter = new tflite::MicroInterpreter(
    tflModel, tflOpsResolver, tensorArena, kTensorArenaSize, &tflErrorReporter);

  // Allocate tensors
  if (tflInterpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Failed to allocate tensors!");
    while (1);
  }

  // Get pointers to input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  Serial.println("System initialized!");
  Serial.println("Press 's' to start gesture recognition");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's') {
      recognizeGesture();
    }
  }
}

void recognizeGesture() {
  Serial.println("Recording gesture...");
  digitalWrite(LED_BUILTIN, HIGH);

  // Collect samples
  for (int i = 0; i < NUM_SAMPLES; i++) {
    while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
      delay(1);
    }
    
    IMU.readAcceleration(accelX[i], accelY[i], accelZ[i]);
    IMU.readGyroscope(gyroX[i], gyroY[i], gyroZ[i]);
    
    if (i % 10 == 0) Serial.print(".");
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("\nProcessing...");

  // Extract features
  float features[30];  // 5 features * 6 axes
  int feature_idx = 0;
  
  // Calculate features for each axis
  for (float* data : {accelX, accelY, accelZ, gyroX, gyroY, gyroZ}) {
    // Mean
    float sum = 0;
    float abs_sum = 0;
    float min_val = data[0];
    float max_val = data[0];
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
      sum += data[i];
      abs_sum += abs(data[i]);
      min_val = min(min_val, data[i]);
      max_val = max(max_val, data[i]);
    }
    float mean = sum / NUM_SAMPLES;
    
    // Standard deviation
    float sq_sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      sq_sum += (data[i] - mean) * (data[i] - mean);
    }
    float std_dev = sqrt(sq_sum / NUM_SAMPLES);
    
    // Store features
    features[feature_idx++] = mean;
    features[feature_idx++] = std_dev;
    features[feature_idx++] = min_val;
    features[feature_idx++] = max_val;
    features[feature_idx++] = abs_sum / NUM_SAMPLES;
  }

  // Copy features to input tensor
  for (int i = 0; i < 30; i++) {
    tflInputTensor->data.f[i] = features[i];
  }

  // Run inference
  if (tflInterpreter->Invoke() != kTfLiteOk) {
    Serial.println("Error invoking model");
    return;
  }

  // Get prediction
  float max_prob = 0;
  int predicted_gesture = -1;
  for (int i = 0; i < 4; i++) {
    float prob = tflOutputTensor->data.f[i];
    if (prob > max_prob) {
      max_prob = prob;
      predicted_gesture = i;
    }
  }

  // Print result
  Serial.print("Detected gesture: ");
  Serial.print(GESTURE_LABELS[predicted_gesture]);
  Serial.print(" (confidence: ");
  Serial.print(max_prob * 100);
  Serial.println("%)");
  
  // Visual feedback using RGB LED
  digitalWrite(LEDR, predicted_gesture == 0 ? LOW : HIGH);  // Wave
  digitalWrite(LEDG, predicted_gesture == 1 ? LOW : HIGH);  // Circle
  digitalWrite(LEDB, predicted_gesture == 2 ? LOW : HIGH);  // Shake/Tap
  
  delay(1000);  // Keep LED on for a second
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  
  Serial.println("\nPress 's' to recognize another gesture");
} 