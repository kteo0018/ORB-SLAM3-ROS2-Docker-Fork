#!/usr/bin/env python3
"""
Train a simple CNN classifier for trunk verification
Uses patches extracted from marked trunks
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import cv2
import numpy as np
import os
from pathlib import Path
from sklearn.model_selection import train_test_split

class TrunkDataset(Dataset):
    """Dataset for trunk classification"""
    def __init__(self, positive_dir, negative_dir, transform=None, split='train', train_ratio=0.7, val_ratio=0.15):
        self.transform = transform
        self.images = []
        self.labels = []
        
        # Load positive samples
        pos_files = sorted([f for f in os.listdir(positive_dir) if f.endswith(('.png', '.jpg'))])
        for f in pos_files:
            self.images.append(os.path.join(positive_dir, f))
            self.labels.append(1)  # 1 = trunk
        
        # Load negative samples
        neg_files = sorted([f for f in os.listdir(negative_dir) if f.endswith(('.png', '.jpg'))])
        for f in neg_files:
            self.images.append(os.path.join(negative_dir, f))
            self.labels.append(0)  # 0 = non-trunk
        
        # Split dataset
        indices = np.arange(len(self.images))
        train_indices, temp_indices = train_test_split(
            indices, test_size=(1 - train_ratio), random_state=42, stratify=self.labels
        )
        val_indices, test_indices = train_test_split(
            temp_indices, test_size=(val_ratio / (1 - train_ratio)), random_state=42, stratify=[self.labels[i] for i in temp_indices]
        )
        
        if split == 'train':
            self.indices = train_indices
        elif split == 'val':
            self.indices = val_indices
        else:  # test
            self.indices = test_indices
        
        print(f"{split.capitalize()} set: {len(self.indices)} samples")
    
    def __len__(self):
        return len(self.indices)
    
    def __getitem__(self, idx):
        actual_idx = self.indices[idx]
        img_path = self.images[actual_idx]
        label = self.labels[actual_idx]
        
        # Load image
        image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            image = np.zeros((64, 64), dtype=np.uint8)
        
        # Convert to 3-channel (for pretrained models) or keep grayscale
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        
        # Apply transforms
        if self.transform:
            image = self.transform(image)
        
        return image, label

class TrunkClassifier(nn.Module):
    """Simple CNN for trunk classification"""
    def __init__(self, num_classes=2):
        super(TrunkClassifier, self).__init__()
        self.features = nn.Sequential(
            # First conv block
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            
            # Second conv block
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            
            # Third conv block
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
        )
        
        self.classifier = nn.Sequential(
            nn.Dropout(0.5),
            nn.Linear(128 * 8 * 8, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes)
        )
    
    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

def train_model(model, train_loader, val_loader, num_epochs=50, device='cuda'):
    """Train the model"""
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)
    
    best_val_acc = 0.0
    
    for epoch in range(num_epochs):
        # Training
        model.train()
        train_loss = 0.0
        train_correct = 0
        train_total = 0
        
        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            _, predicted = torch.max(outputs.data, 1)
            train_total += labels.size(0)
            train_correct += (predicted == labels).sum().item()
        
        train_acc = 100 * train_correct / train_total
        
        # Validation
        model.eval()
        val_loss = 0.0
        val_correct = 0
        val_total = 0
        
        with torch.no_grad():
            for images, labels in val_loader:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                loss = criterion(outputs, labels)
                
                val_loss += loss.item()
                _, predicted = torch.max(outputs.data, 1)
                val_total += labels.size(0)
                val_correct += (predicted == labels).sum().item()
        
        val_acc = 100 * val_correct / val_total
        scheduler.step(val_loss)
        
        print(f"Epoch [{epoch+1}/{num_epochs}]")
        print(f"  Train Loss: {train_loss/len(train_loader):.4f}, Train Acc: {train_acc:.2f}%")
        print(f"  Val Loss: {val_loss/len(val_loader):.4f}, Val Acc: {val_acc:.2f}%")
        
        # Save best model
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), 'trunk_classifier_best.pth')
            print(f"  ✓ Saved best model (Val Acc: {val_acc:.2f}%)")
        print()
    
    return model

def main():
    """Main function"""
    # Check if dataset exists
    positive_dir = "cnn_dataset/positive"
    negative_dir = "cnn_dataset/negative"
    
    if not os.path.exists(positive_dir) or not os.path.exists(negative_dir):
        print("Error: CNN dataset not found!")
        print("Please run prepare_cnn_dataset.py first")
        return
    
    # Data transforms
    train_transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.RandomHorizontalFlip(),
        transforms.RandomRotation(10),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    val_transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    # Create datasets
    train_dataset = TrunkDataset(positive_dir, negative_dir, transform=train_transform, split='train')
    val_dataset = TrunkDataset(positive_dir, negative_dir, transform=val_transform, split='val')
    test_dataset = TrunkDataset(positive_dir, negative_dir, transform=val_transform, split='test')
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False, num_workers=4)
    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False, num_workers=4)
    
    # Create model
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    model = TrunkClassifier(num_classes=2).to(device)
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Train model
    print("\n" + "="*80)
    print("Training Trunk Classifier")
    print("="*80)
    model = train_model(model, train_loader, val_loader, num_epochs=50, device=device)
    
    # Test model
    print("="*80)
    print("Testing on Test Set")
    print("="*80)
    model.load_state_dict(torch.load('trunk_classifier_best.pth'))
    model.eval()
    
    test_correct = 0
    test_total = 0
    
    with torch.no_grad():
        for images, labels in test_loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = torch.max(outputs.data, 1)
            test_total += labels.size(0)
            test_correct += (predicted == labels).sum().item()
    
    test_acc = 100 * test_correct / test_total
    print(f"Test Accuracy: {test_acc:.2f}%")
    
    # Export to ONNX for C++ integration
    print("\n" + "="*80)
    print("Exporting to ONNX")
    print("="*80)
    model.eval()
    dummy_input = torch.randn(1, 3, 64, 64).to(device)
    onnx_path = "trunk_classifier.onnx"
    
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
    )
    
    print(f"✓ Model exported to {onnx_path}")
    print("\nNext steps:")
    print("1. Integrate ONNX Runtime in ORB-SLAM3 C++ code")
    print("2. Use classifier to verify HoughCircles detections")
    print("3. See IMPROVEMENT_STRATEGIES.md for integration details")

if __name__ == "__main__":
    main()

