/* Simple image classification on Arduino Nano Sense BLE Rev 2. This generates simple ASCII 'images' then predicts them
 using a basic feedforward neural network. It sends its results over serial.
 Kevin Walker April 2025

Neural Network Architecture:
* Input layer: 64 neurons (8x8 image)
* Hidden layer: 16 neurons
* Output layer: 5 neurons (one for each pattern type)
* Uses sigmoid activation function
* Implements backpropagation for learning

Training Process:
* The network is trained on startup
* Uses 100 epochs of training
* Learning rate of 0.1
* Shows training progress in Serial Monitor

Pattern Classification:
* Generates patterns (horizontal line, vertical line, diagonal, checkerboard)
* Classifies each pattern using the trained network
* Sends both the actual pattern and predicted pattern over BLE
*/

#include <math.h>

// Neural Network parameters
const int INPUT_SIZE = 64;  // 8x8 image
const int HIDDEN_SIZE = 16;  // Middle ground between 8 and 32
const int OUTPUT_SIZE = 5;  // Number of pattern types

// Neural Network weights and biases
float weights1[INPUT_SIZE][HIDDEN_SIZE];
float weights2[HIDDEN_SIZE][OUTPUT_SIZE];
float bias1[HIDDEN_SIZE];
float bias2[OUTPUT_SIZE];

// Momentum terms
float momentum1[INPUT_SIZE][HIDDEN_SIZE] = {0};
float momentum2[HIDDEN_SIZE][OUTPUT_SIZE] = {0};
float momentumBias1[HIDDEN_SIZE] = {0};
float momentumBias2[OUTPUT_SIZE] = {0};
const float MOMENTUM = 0.7;  // Middle ground between 0.5 and 0.9

// Neural Network activations
float hidden[HIDDEN_SIZE];
float output[OUTPUT_SIZE];

// Training parameters
float learningRate = 0.08;  // Middle ground between 0.05 and 0.1
const float LEARNING_RATE_DECAY = 0.997;  // Middle ground between 0.995 and 0.999
const float MIN_LEARNING_RATE = 0.001;
const int EPOCHS = 100;  // Middle ground between 50 and 200

// Pattern types
enum PatternType {
  PATTERN_NONE,
  PATTERN_HORIZONTAL_LINE,
  PATTERN_VERTICAL_LINE,
  PATTERN_DIAGONAL,
  PATTERN_CHECKERBOARD
};

PatternType currentPattern = PATTERN_NONE;

// Simulated "image" parameters
const int IMAGE_WIDTH = 8;
const int IMAGE_HEIGHT = 8;
const int TOTAL_PIXELS = IMAGE_WIDTH * IMAGE_HEIGHT;

// Buffer for simulated image data
float imageBuffer[TOTAL_PIXELS];  // Using float for neural network input

// Success tracking
const int SUCCESS_WINDOW = 100;  // Number of tests to average over
bool successHistory[SUCCESS_WINDOW] = {false};
int historyIndex = 0;
unsigned long totalTests = 0;
float currentSuccessRate = 0.0;
float lastSuccessRate = 0.0;
float improvementRate = 0.0;

// Pattern parameters
const float NOISE_LEVEL = 0.3;  // Increased from 0.2
const float PATTERN_STRENGTH = 0.7;  // Reduced from 0.8
const int MAX_SHIFT = 2;  // Increased from 1
const float FUZZINESS = 0.3;  // Controls how fuzzy the patterns are

// Sigmoid activation function
float sigmoid(float x) {
  return 1.0 / (1.0 + exp(-x));
}

// Leaky ReLU activation function
float leakyReLU(float x) {
  return x > 0 ? x : 0.01 * x;
}

float leakyReLUDerivative(float x) {
  return x > 0 ? 1.0 : 0.01;
}

// Function to initialize neural network weights using Xavier/Glorot initialization
void initializeWeights() {
  // Calculate scaling factors
  float scale1 = sqrt(2.0 / (INPUT_SIZE + HIDDEN_SIZE));
  float scale2 = sqrt(2.0 / (HIDDEN_SIZE + OUTPUT_SIZE));
  
  // Initialize weights with scaled random values
  for (int i = 0; i < INPUT_SIZE; i++) {
    for (int j = 0; j < HIDDEN_SIZE; j++) {
      weights1[i][j] = (random(100) / 100.0 - 0.5) * scale1;
    }
  }
  
  for (int i = 0; i < HIDDEN_SIZE; i++) {
    for (int j = 0; j < OUTPUT_SIZE; j++) {
      weights2[i][j] = (random(100) / 100.0 - 0.5) * scale2;
    }
  }
  
  // Initialize biases to small positive values
  for (int i = 0; i < HIDDEN_SIZE; i++) {
    bias1[i] = 0.01;
  }
  
  for (int i = 0; i < OUTPUT_SIZE; i++) {
    bias2[i] = 0.01;
  }
}

// Forward propagation
void forwardPropagation(float* input) {
  // Hidden layer
  for (int j = 0; j < HIDDEN_SIZE; j++) {
    float sum = bias1[j];
    for (int i = 0; i < INPUT_SIZE; i++) {
      sum += input[i] * weights1[i][j];
    }
    hidden[j] = leakyReLU(sum);
  }
  
  // Output layer
  for (int j = 0; j < OUTPUT_SIZE; j++) {
    float sum = bias2[j];
    for (int i = 0; i < HIDDEN_SIZE; i++) {
      sum += hidden[i] * weights2[i][j];
    }
    output[j] = sigmoid(sum);
  }
}

// Backward propagation with momentum
void backwardPropagation(float* input, int target) {
  // Calculate output layer errors
  float outputErrors[OUTPUT_SIZE];
  for (int i = 0; i < OUTPUT_SIZE; i++) {
    float targetValue = (i == target) ? 1.0 : 0.0;
    outputErrors[i] = (targetValue - output[i]) * output[i] * (1 - output[i]);
  }
  
  // Calculate hidden layer errors
  float hiddenErrors[HIDDEN_SIZE];
  for (int i = 0; i < HIDDEN_SIZE; i++) {
    float sum = 0;
    for (int j = 0; j < OUTPUT_SIZE; j++) {
      sum += outputErrors[j] * weights2[i][j];
    }
    hiddenErrors[i] = sum * leakyReLUDerivative(hidden[i]);
  }
  
  // Update weights and biases with momentum
  for (int i = 0; i < HIDDEN_SIZE; i++) {
    for (int j = 0; j < OUTPUT_SIZE; j++) {
      float delta = learningRate * outputErrors[j] * hidden[i];
      momentum2[i][j] = MOMENTUM * momentum2[i][j] + delta;
      weights2[i][j] += momentum2[i][j];
    }
    float delta = learningRate * outputErrors[i];
    momentumBias2[i] = MOMENTUM * momentumBias2[i] + delta;
    bias2[i] += momentumBias2[i];
  }
  
  for (int i = 0; i < INPUT_SIZE; i++) {
    for (int j = 0; j < HIDDEN_SIZE; j++) {
      float delta = learningRate * hiddenErrors[j] * input[i];
      momentum1[i][j] = MOMENTUM * momentum1[i][j] + delta;
      weights1[i][j] += momentum1[i][j];
    }
  }
  
  for (int i = 0; i < HIDDEN_SIZE; i++) {
    float delta = learningRate * hiddenErrors[i];
    momentumBias1[i] = MOMENTUM * momentumBias1[i] + delta;
    bias1[i] += momentumBias1[i];
  }
  
  // Decay learning rate
  learningRate = max(learningRate * LEARNING_RATE_DECAY, MIN_LEARNING_RATE);
}

// Function to add noise to a pattern
void addNoise(float* pattern, int size) {
  for (int i = 0; i < size; i++) {
    // Add random noise with higher variance
    float noise = (random(100) / 100.0 - 0.5) * NOISE_LEVEL;
    
    // Occasionally add spikes (5% chance)
    if (random(100) < 5) {
      noise += (random(100) / 100.0) * 0.5;
    }
    
    pattern[i] = pattern[i] * PATTERN_STRENGTH + noise;
    
    // Clamp values between 0 and 1
    pattern[i] = max(0.0, min(1.0, pattern[i]));
  }
}

// Function to shift a pattern
void shiftPattern(float* pattern, int width, int height) {
  float temp[TOTAL_PIXELS];
  memcpy(temp, pattern, TOTAL_PIXELS * sizeof(float));
  
  int shiftX = random(2 * MAX_SHIFT + 1) - MAX_SHIFT;
  int shiftY = random(2 * MAX_SHIFT + 1) - MAX_SHIFT;
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int newX = (x + shiftX + width) % width;
      int newY = (y + shiftY + height) % height;
      pattern[y * width + x] = temp[newY * width + newX];
    }
  }
}

// Function to generate different patterns
void generatePattern(PatternType pattern) {
  switch(pattern) {
    case PATTERN_HORIZONTAL_LINE:
      for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
          // Make the line thicker and fuzzier
          float dist = abs(y - IMAGE_HEIGHT/2);
          float value = exp(-dist * dist / (2.0 * FUZZINESS * FUZZINESS));
          
          // Add some vertical variation
          float verticalVar = sin(x * 0.5) * 0.2;
          value = max(0.0, min(1.0, value + verticalVar));
          
          imageBuffer[y * IMAGE_WIDTH + x] = value;
        }
      }
      break;
      
    case PATTERN_VERTICAL_LINE:
      for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
          // Make the line thicker and fuzzier
          float dist = abs(x - IMAGE_WIDTH/2);
          float value = exp(-dist * dist / (2.0 * FUZZINESS * FUZZINESS));
          
          // Add some horizontal variation
          float horizontalVar = sin(y * 0.5) * 0.2;
          value = max(0.0, min(1.0, value + horizontalVar));
          
          imageBuffer[y * IMAGE_WIDTH + x] = value;
        }
      }
      break;
      
    case PATTERN_DIAGONAL:
      for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
          // Make the diagonal thicker and fuzzier
          float dist = abs(x - y);
          float value = exp(-dist * dist / (2.0 * FUZZINESS * FUZZINESS));
          
          // Add some variation along the diagonal
          float diagVar = sin((x + y) * 0.3) * 0.2;
          value = max(0.0, min(1.0, value + diagVar));
          
          imageBuffer[y * IMAGE_WIDTH + x] = value;
        }
      }
      break;
      
    case PATTERN_CHECKERBOARD:
      for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
          // Make checkerboard more complex
          bool isEven = ((x + y) % 2 == 0);
          float value = isEven ? 1.0 : 0.0;
          
          // Add fuzziness based on distance to edges
          float edgeDist = min(min(x, IMAGE_WIDTH-1-x), min(y, IMAGE_HEIGHT-1-y));
          float edgeFuzz = exp(-edgeDist * edgeDist / (2.0 * FUZZINESS * FUZZINESS));
          
          // Add some variation to the pattern
          float patternVar = sin(x * 0.3) * sin(y * 0.3) * 0.2;
          
          // Combine all effects
          value = value * edgeFuzz + patternVar;
          value = max(0.0, min(1.0, value));
          
          imageBuffer[y * IMAGE_WIDTH + x] = value;
        }
      }
      break;
      
    default:
      memset(imageBuffer, 0, TOTAL_PIXELS * sizeof(float));
      break;
  }
  
  // Shift the pattern
  shiftPattern(imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  
  // Add noise
  addNoise(imageBuffer, TOTAL_PIXELS);
}

// Function to print the current pattern with noise
void printPattern() {
  Serial.println("Current Pattern (with noise):");
  for (int y = 0; y < IMAGE_HEIGHT; y++) {
    for (int x = 0; x < IMAGE_WIDTH; x++) {
      float value = imageBuffer[y * IMAGE_WIDTH + x];
      if (value > 0.8) Serial.print("X");
      else if (value > 0.6) Serial.print("x");
      else if (value > 0.4) Serial.print(".");
      else if (value > 0.2) Serial.print(",");
      else Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
}

// Function to train the neural network
void trainNetwork() {
  Serial.println("Training neural network...");
  Serial.print("Initial learning rate: ");
  Serial.println(learningRate, 4);
  
  for (int epoch = 0; epoch < EPOCHS; epoch++) {
    float totalError = 0;
    
    // Train on each pattern type
    for (int pattern = 0; pattern < OUTPUT_SIZE; pattern++) {
      generatePattern((PatternType)pattern);
      forwardPropagation(imageBuffer);
      backwardPropagation(imageBuffer, pattern);
      
      // Calculate error
      float targetValue = 1.0;
      float error = 0;
      for (int i = 0; i < OUTPUT_SIZE; i++) {
        float diff = (i == pattern ? targetValue : 0.0) - output[i];
        error += diff * diff;
      }
      totalError += error;
    }
    
    if (epoch % 10 == 0) {
      Serial.print("Epoch ");
      Serial.print(epoch);
      Serial.print(": Error = ");
      Serial.print(totalError / OUTPUT_SIZE, 4);
      Serial.print(", Learning Rate = ");
      Serial.println(learningRate, 4);
    }
  }
  
  Serial.println("Training complete!");
}

// Function to classify a pattern
PatternType classifyPattern() {
  forwardPropagation(imageBuffer);
  
  // Find the output with highest activation
  int maxIndex = 0;
  float maxValue = output[0];
  
  for (int i = 1; i < OUTPUT_SIZE; i++) {
    if (output[i] > maxValue) {
      maxValue = output[i];
      maxIndex = i;
    }
  }
  
  return (PatternType)maxIndex;
}

// Function to update success rate with sliding window
void updateSuccessRate(bool success) {
  // Update the sliding window
  successHistory[historyIndex] = success;
  historyIndex = (historyIndex + 1) % SUCCESS_WINDOW;
  totalTests++;
  
  // Calculate success rate over the window
  int windowSize = min(totalTests, SUCCESS_WINDOW);
  int successfulInWindow = 0;
  for (int i = 0; i < windowSize; i++) {
    if (successHistory[i]) successfulInWindow++;
  }
  
  // Update rates
  lastSuccessRate = currentSuccessRate;
  currentSuccessRate = (float)successfulInWindow / windowSize * 100.0;
  
  // Calculate improvement if we have enough data
  if (totalTests > SUCCESS_WINDOW) {
    improvementRate = currentSuccessRate - lastSuccessRate;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  Serial.println("Neural Network Pattern Classifier");
  Serial.println("================================");
  
  // Initialize random seed
  randomSeed(analogRead(0));
  
  // Initialize neural network
  initializeWeights();
  
  // Train the network
  trainNetwork();
}

void loop() {
  static unsigned long lastPatternTime = 0;
  const unsigned long PATTERN_INTERVAL = 2000; // Change pattern every 2 seconds
  
  // Change pattern periodically
  if (millis() - lastPatternTime >= PATTERN_INTERVAL) {
    currentPattern = (PatternType)((currentPattern + 1) % (PATTERN_CHECKERBOARD + 1));
    generatePattern(currentPattern);
    
    // Classify the pattern
    PatternType predicted = classifyPattern();
    
    // Check if prediction was correct
    bool success = (predicted == currentPattern);
    updateSuccessRate(success);
    
    // Print the pattern first
    printPattern();
    
    // Then print the classification results
    Serial.println("=== Classification Results ===");
    Serial.print("Actual Pattern: ");
    switch(currentPattern) {
      case PATTERN_HORIZONTAL_LINE: Serial.print("Horizontal Line"); break;
      case PATTERN_VERTICAL_LINE: Serial.print("Vertical Line"); break;
      case PATTERN_DIAGONAL: Serial.print("Diagonal"); break;
      case PATTERN_CHECKERBOARD: Serial.print("Checkerboard"); break;
      default: Serial.print("None"); break;
    }
    
    Serial.print(" | Predicted: ");
    switch(predicted) {
      case PATTERN_HORIZONTAL_LINE: Serial.print("Horizontal Line"); break;
      case PATTERN_VERTICAL_LINE: Serial.print("Vertical Line"); break;
      case PATTERN_DIAGONAL: Serial.print("Diagonal"); break;
      case PATTERN_CHECKERBOARD: Serial.print("Checkerboard"); break;
      default: Serial.print("None"); break;
    }
    
    Serial.print("\nSuccess: ");
    Serial.print(success ? "YES" : "NO");
    Serial.print(" | Success Rate (last ");
    Serial.print(min(totalTests, SUCCESS_WINDOW));
    Serial.print(" tests): ");
    Serial.print(currentSuccessRate, 1);
    Serial.print("%");
    
    if (totalTests > SUCCESS_WINDOW) {
      Serial.print(" | Improvement: ");
      Serial.print(improvementRate, 2);
      Serial.print("%");
    }
    Serial.println("\n=============================\n");
    
    lastPatternTime = millis();
  }
}
