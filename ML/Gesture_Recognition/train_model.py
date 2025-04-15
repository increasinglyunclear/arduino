import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from pathlib import Path
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder, StandardScaler

class GestureNet(nn.Module):
    def __init__(self, input_size, num_classes):
        super(GestureNet, self).__init__()
        self.fc1 = nn.Linear(input_size, 16)
        self.fc2 = nn.Linear(16, num_classes)
        self.relu = nn.ReLU()
        
    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.fc2(x)
        return x

def augment_data(features, label, num_augmented=8):
    """Create augmented versions of the feature vector."""
    augmented_features = []
    augmented_labels = []
    
    # Original sample
    augmented_features.append(features)
    augmented_labels.append(label)
    
    # Create augmented versions
    for _ in range(num_augmented):
        # Add random noise
        noise = np.random.normal(0, 0.1, features.shape)
        augmented = features + noise
        
        # Random scaling
        scale = np.random.uniform(0.9, 1.1)
        augmented = augmented * scale
        
        augmented_features.append(augmented)
        augmented_labels.append(label)
    
    return np.array(augmented_features), np.array(augmented_labels)

def create_windows(df, window_size=20, stride=10):
    """Create sliding windows from time series data."""
    windows = []
    for i in range(0, len(df) - window_size + 1, stride):
        window = df.iloc[i:i + window_size]
        features = []
        for axis in ['accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ']:
            # Calculate features for this window
            axis_data = window[axis].astype(float)
            features.extend([
                axis_data.mean(),
                axis_data.std(),
                axis_data.max(),
                axis_data.min(),
                axis_data.quantile(0.25),
                axis_data.quantile(0.75)
            ])
        windows.append(features)
    return np.array(windows)

def load_and_preprocess_data(data_dir, window_size=20, stride=10):
    """Load and preprocess gesture data from CSV files using sliding windows."""
    print("Loading and preprocessing data...")
    
    # Find all CSV files in the directory
    csv_files = list(Path(data_dir).glob('*.csv'))
    print(f"Found CSV files: {csv_files}")
    
    all_features = []
    all_labels = []
    
    for file_path in csv_files:
        print(f"\nProcessing file: {file_path.name}")
        df = pd.read_csv(file_path)
        gesture_name = df['gesture'].iloc[0]
        print(f"Gesture name: {gesture_name}")
        print(f"Data shape: {df.shape}")
        
        # Create windows from the time series data
        windows = create_windows(df, window_size, stride)
        print(f"Created {len(windows)} windows")
        
        # Add features and labels
        all_features.extend(windows)
        all_labels.extend([gesture_name] * len(windows))
    
    # Convert to numpy arrays
    X = np.array(all_features)
    y = np.array(all_labels)
    
    print(f"\nTotal samples: {len(X)}")
    print(f"Feature vector shape: {X.shape}")
    print(f"Unique labels: {np.unique(y)}")
    print(f"Samples per class: {pd.Series(y).value_counts()}")
    
    return X, y

def save_model_for_arduino(model, le, scaler):
    """Save model weights and generate C header file for Arduino."""
    # Save model weights
    torch.save({
        'model_state_dict': model.state_dict(),
        'scaler_mean': scaler.mean_,
        'scaler_scale': scaler.scale_
    }, 'gesture_model.pth')
    
    # Create C header file with model weights and scaling parameters
    with open('model_data.h', 'w') as f:
        f.write("// Automatically generated from Python\n\n")
        f.write("#ifndef MODEL_DATA_H\n")
        f.write("#define MODEL_DATA_H\n\n")
        
        # Write gesture labels
        f.write("const char* GESTURE_LABELS[] = {")
        for label in le.classes_:
            f.write(f'\n  "{label}",')
        f.write("\n};\n\n")
        
        # Write scaling parameters
        f.write("// Feature scaling parameters\n")
        f.write("const float FEATURE_MEANS[] = {")
        for mean in scaler.mean_:
            f.write(f"{mean:.6f}f, ")
        f.write("};\n\n")
        
        f.write("const float FEATURE_SCALES[] = {")
        for scale in scaler.scale_:
            f.write(f"{scale:.6f}f, ")
        f.write("};\n\n")
        
        # Write model weights
        f.write("// Model weights\n")
        for name, param in model.named_parameters():
            if 'weight' in name:
                f.write(f"\n// {name}\n")
                f.write("const float " + name.replace('.', '_') + "[] = {")
                weights = param.detach().numpy().flatten()
                for i, w in enumerate(weights):
                    if i % 8 == 0:
                        f.write('\n  ')
                    f.write(f"{w:.6f}f, ")
                f.write("\n};\n")
            elif 'bias' in name:
                f.write(f"\n// {name}\n")
                f.write("const float " + name.replace('.', '_') + "[] = {")
                biases = param.detach().numpy().flatten()
                for i, b in enumerate(biases):
                    if i % 8 == 0:
                        f.write('\n  ')
                    f.write(f"{b:.6f}f, ")
                f.write("\n};\n")
        
        f.write("\n#endif  // MODEL_DATA_H\n")

def main():
    # Load and preprocess data
    X, y = load_and_preprocess_data('.')
    
    # Encode labels
    le = LabelEncoder()
    y_encoded = le.fit_transform(y)
    
    # Scale features
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y_encoded, test_size=0.2, random_state=42, stratify=y_encoded
    )
    
    # Convert to PyTorch tensors
    X_train = torch.FloatTensor(X_train)
    y_train = torch.LongTensor(y_train)
    X_test = torch.FloatTensor(X_test)
    y_test = torch.LongTensor(y_test)
    
    # Initialize model
    model = GestureNet(input_size=X_train.shape[1], num_classes=len(le.classes_))
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # Training loop
    print("\nTraining model...")
    num_epochs = 100
    batch_size = 4
    
    for epoch in range(num_epochs):
        model.train()
        optimizer.zero_grad()
        outputs = model(X_train)
        loss = criterion(outputs, y_train)
        loss.backward()
        optimizer.step()
        
        if (epoch + 1) % 20 == 0:
            print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')
    
    # Evaluation
    print("\nEvaluating model...")
    model.eval()
    with torch.no_grad():
        outputs = model(X_test)
        _, predicted = torch.max(outputs.data, 1)
        accuracy = (predicted == y_test).sum().item() / y_test.size(0)
        print(f'Test accuracy: {accuracy:.2%}')
        
        print("\nPredictions vs Actuals:")
        for i in range(len(y_test)):
            print(f"Predicted: {le.inverse_transform([predicted[i].item()])[0]}, "
                  f"Actual: {le.inverse_transform([y_test[i].item()])[0]}")
    
    # Save model for Arduino
    print("\nSaving model for Arduino...")
    save_model_for_arduino(model, le, scaler)
    
    # Save PyTorch model
    torch.save({
        'model_state_dict': model.state_dict(),
        'scaler': scaler,
        'label_encoder': le
    }, 'gesture_model.pth')
    print("Model saved as 'gesture_model.pth'")

if __name__ == "__main__":
    main() 