/*
  Gesture Recognition for Arduino Nano 33 BLE Sense
  This program uses a trained neural network to recognize gestures
*/

#include "Arduino_BMI270_BMM150.h"
#include "model_data.h"

// Constants
const int SAMPLE_RATE = 119;  // Hz
const int WINDOW_SIZE = 20;   // Number of samples per window
const int STRIDE = 10;        // Number of samples to move between windows
const int NUM_FEATURES = 36;  // 6 sensors * 6 features per sensor

// Data arrays
float accelX[WINDOW_SIZE];
float accelY[WINDOW_SIZE];
float accelZ[WINDOW_SIZE];
float gyroX[WINDOW_SIZE];
float gyroY[WINDOW_SIZE];
float gyroZ[WINDOW_SIZE];

// Feature vector
float features[NUM_FEATURES];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  Serial.println("Gesture Recognition Ready!");
  Serial.println("Perform a gesture to recognize it");
}

void loop() {
  // Collect a window of data
  collectWindowData();
  
  // Extract features
  extractFeatures();
  
  // Scale features
  scaleFeatures();
  
  // Make prediction
  int prediction = predictGesture();
  
  // Print result
  Serial.print("Detected gesture: ");
  Serial.println(GESTURE_LABELS[prediction]);
  
  // Small delay to prevent too frequent predictions
  delay(500);
}

void collectWindowData() {
  // Collect WINDOW_SIZE samples
  for (int i = 0; i < WINDOW_SIZE; i++) {
    // Wait for new data
    while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
      delay(1);
    }
    
    // Read accelerometer
    IMU.readAcceleration(accelX[i], accelY[i], accelZ[i]);
    
    // Read gyroscope
    IMU.readGyroscope(gyroX[i], gyroY[i], gyroZ[i]);
  }
}

void extractFeatures() {
  int feature_idx = 0;
  
  // Process each sensor
  float* sensors[] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
  
  for (int s = 0; s < 6; s++) {
    float* data = sensors[s];
    
    // Calculate features
    features[feature_idx++] = mean(data, WINDOW_SIZE);
    features[feature_idx++] = stdDev(data, WINDOW_SIZE);
    features[feature_idx++] = max(data, WINDOW_SIZE);
    features[feature_idx++] = min(data, WINDOW_SIZE);
    features[feature_idx++] = percentile(data, WINDOW_SIZE, 25);
    features[feature_idx++] = percentile(data, WINDOW_SIZE, 75);
  }
}

void scaleFeatures() {
  for (int i = 0; i < NUM_FEATURES; i++) {
    features[i] = (features[i] - FEATURE_MEANS[i]) / FEATURE_SCALES[i];
  }
}

int predictGesture() {
  // First layer (fc1)
  float hidden[16] = {0};
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < NUM_FEATURES; j++) {
      hidden[i] += features[j] * fc1_weight[i * NUM_FEATURES + j];
    }
    hidden[i] += fc1_bias[i];
    // ReLU activation
    if (hidden[i] < 0) hidden[i] = 0;
  }
  
  // Second layer (fc2)
  float output[4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 16; j++) {
      output[i] += hidden[j] * fc2_weight[i * 16 + j];
    }
    output[i] += fc2_bias[i];
  }
  
  // Find maximum output (prediction)
  int max_idx = 0;
  float max_val = output[0];
  for (int i = 1; i < 4; i++) {
    if (output[i] > max_val) {
      max_val = output[i];
      max_idx = i;
    }
  }
  
  return max_idx;
}

// Helper functions for feature calculation
float mean(float* data, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

float stdDev(float* data, int size) {
  float m = mean(data, size);
  float sum = 0;
  for (int i = 0; i < size; i++) {
    float diff = data[i] - m;
    sum += diff * diff;
  }
  return sqrt(sum / size);
}

float max(float* data, int size) {
  float max_val = data[0];
  for (int i = 1; i < size; i++) {
    if (data[i] > max_val) max_val = data[i];
  }
  return max_val;
}

float min(float* data, int size) {
  float min_val = data[0];
  for (int i = 1; i < size; i++) {
    if (data[i] < min_val) min_val = data[i];
  }
  return min_val;
}

float percentile(float* data, int size, int p) {
  // Simple implementation - for better accuracy, use proper sorting
  float sorted[size];
  memcpy(sorted, data, size * sizeof(float));
  
  // Bubble sort (simple but not efficient)
  for (int i = 0; i < size-1; i++) {
    for (int j = 0; j < size-i-1; j++) {
      if (sorted[j] > sorted[j+1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
      }
    }
  }
  
  int idx = (p * size) / 100;
  return sorted[idx];
} 