# Arduino Gesture Recognition

This project implements gesture recognition using the Arduino Nano 33 BLE Sense's IMU sensor. It consists of two main components: data collection and gesture recognition.

## Hardware Requirements
- Arduino Nano 33 BLE Sense
- USB cable for programming and power
- Computer with Python 3.x installed

## Software Requirements
- Arduino IDE
- Python packages:
  - numpy
  - pandas
  - scikit-learn
  - torch
  - jax
  - jaxlib

## Project Structure
```
Gesture_Recognition/
├── Gesture_Recognition.ino      # Data collection sketch
├── Recognition/                 # Recognition sketch and model
│   ├── recognize_gestures.ino   # Gesture recognition sketch
│   └── model_data.h            # Trained model parameters
├── train_model.py              # Python script to train the model
├── analyze_gestures.py         # Python script to analyze gesture data
└── *.csv                       # Training data files
```

## Complete Workflow

### 1. Data Collection

1. Open `Gesture_Recognition.ino` in Arduino IDE
2. Install required library:
   - Open Arduino IDE
   - Go to Tools > Manage Libraries
   - Search for "Arduino_BMI270_BMM150"
   - Install the library
3. Upload the sketch to your Arduino
4. Open Serial Monitor (115200 baud)
5. Follow the prompts to record gestures:
   - Press '1' to record gesture 1
   - Press '2' to record gesture 2
   - Press '3' to record gesture 3
   - Press '4' to record gesture 4
6. For each gesture:
   - When prompted, perform the gesture
   - Hold the gesture for about 1-2 seconds
   - The sketch will collect 20 samples
   - Data will be saved to CSV files (e.g., `gesture_1.csv`, `gesture_2.csv`, etc.)

### 2. Data Analysis (Optional)

1. Run the analysis script:
   ```bash
   python analyze_gestures.py
   ```
2. This will:
   - Load all CSV files
   - Calculate basic statistics
   - Generate plots of the gesture data
   - Help you verify if your gestures are distinct enough

### 3. Model Training

1. Make sure you have collected enough data (at least 20 samples per gesture)
2. Install required Python packages:
   ```bash
   pip install numpy pandas scikit-learn torch jax jaxlib
   ```
3. Run the training script:
   ```bash
   python train_model.py
   ```
4. The script will:
   - Load and preprocess the CSV files
   - Split data into training and testing sets
   - Train a neural network model
   - Generate `model_data.h` with the trained parameters
   - Print the test accuracy

### 4. Gesture Recognition

1. Open `Recognition/recognize_gestures.ino` in Arduino IDE
2. Make sure `model_data.h` is in the same directory
3. Upload the sketch to your Arduino
4. Open Serial Monitor (115200 baud)
5. Perform gestures to see them recognized in real-time

## Troubleshooting

### Data Collection Issues
- If the IMU fails to initialize, check the USB connection
- If data collection is too slow, reduce the sample rate in the sketch
- If gestures aren't being recorded, check the Serial Monitor for error messages

### Model Training Issues
- If you get "Not enough samples" error, collect more data
- If accuracy is low, try:
  - Collecting more samples
  - Making gestures more distinct
  - Adjusting the model architecture in `train_model.py`

### Recognition Issues
- If recognition is inaccurate:
  - Make sure you're performing gestures similarly to training
  - Check if the model accuracy was good during training
  - Consider collecting more training data
- If the sketch crashes:
  - Check if `model_data.h` is in the correct directory
  - Verify the Arduino has enough memory

## Tips for Better Results

1. **Data Collection**:
   - Perform each gesture consistently
   - Collect at least 20 samples per gesture
   - Make gestures distinct from each other
   - Hold the gesture for 1-2 seconds when recording

2. **Model Training**:
   - Monitor the test accuracy
   - Aim for at least 80% accuracy
   - If accuracy is low, collect more diverse samples

3. **Gesture Recognition**:
   - Perform gestures at a similar speed to training
   - Hold the gesture for about 1 second
   - Make sure the Arduino is stable when performing gestures

## Example Gestures

The system is currently set up for 4 gestures. Here are some suggested gestures:
1. Up - Move the Arduino upward quickly
2. Down - Move the Arduino downward quickly
3. Left - Move the Arduino to the left quickly
4. Right - Move the Arduino to the right quickly

You can modify these gestures or add new ones by:
1. Collecting new data
2. Retraining the model
3. Updating the gesture labels in both sketches 