#!/usr/bin/env python3
"""
Train ML Classifier for Leap Motion Hand Movement Classification
Tests multiple models and saves the best performer
"""

import pandas as pd
import numpy as np
import joblib
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
from datetime import datetime
import os

class HandMovementClassifier:
    def __init__(self):
        """Initialize the classifier trainer"""
        self.models = {}
        self.scaler = StandardScaler()
        self.best_model = None
        self.best_model_name = None
        self.feature_columns = None
        
    def load_data(self, csv_path: str):
        """Load the training features CSV"""
        print(f"Loading data from: {csv_path}")
        df = pd.read_csv(csv_path)
        
        print(f"Dataset shape: {df.shape}")
        print(f"Label distribution:")
        print(df['label'].value_counts())
        
        # Separate features and labels
        self.feature_columns = [col for col in df.columns 
                               if col not in ['label', 'source_file']]
        
        X = df[self.feature_columns].values
        y = df['label'].values
        
        print(f"\nFeatures: {len(self.feature_columns)}")
        print(f"Samples: {len(X)}")
        
        return X, y
    
    def prepare_data(self, X, y, test_size=0.2, val_size=0.1):
        """
        Split data into train, validation, and test sets
        Scale features using StandardScaler
        """
        print(f"\nSplitting data:")
        print(f"  Train: {(1-test_size-val_size)*100:.0f}%")
        print(f"  Validation: {val_size*100:.0f}%")
        print(f"  Test: {test_size*100:.0f}%")
        
        # First split: separate test set
        X_temp, X_test, y_temp, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )
        
        # Second split: separate validation from train
        val_ratio = val_size / (1 - test_size)
        X_train, X_val, y_train, y_val = train_test_split(
            X_temp, y_temp, test_size=val_ratio, random_state=42, stratify=y_temp
        )
        
        print(f"\nActual split sizes:")
        print(f"  Train: {len(X_train)} samples")
        print(f"  Validation: {len(X_val)} samples")
        print(f"  Test: {len(X_test)} samples")
        
        # Scale features
        print("\nScaling features...")
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_val_scaled = self.scaler.transform(X_val)
        X_test_scaled = self.scaler.transform(X_test)
        
        return X_train_scaled, X_val_scaled, X_test_scaled, y_train, y_val, y_test
    
    def train_models(self, X_train, y_train):
        """Train multiple ML models"""
        print("\n" + "="*60)
        print("TRAINING MODELS")
        print("="*60)
        
        # Define models to test
        models_to_train = {
            'Random Forest': RandomForestClassifier(
                n_estimators=100,
                max_depth=20,
                min_samples_split=5,
                random_state=42,
                n_jobs=-1
            ),
            'Gradient Boosting': GradientBoostingClassifier(
                n_estimators=100,
                max_depth=5,
                learning_rate=0.1,
                random_state=42
            ),
            'Logistic Regression': LogisticRegression(
                max_iter=1000,
                random_state=42,
                n_jobs=-1
            )
        }
        
        # Train each model
        for name, model in models_to_train.items():
            print(f"\nTraining {name}...")
            model.fit(X_train, y_train)
            self.models[name] = model
            
            # Cross-validation score
            cv_scores = cross_val_score(model, X_train, y_train, cv=5, n_jobs=-1)
            print(f"  Cross-validation accuracy: {cv_scores.mean():.4f} (+/- {cv_scores.std():.4f})")
    
    def evaluate_models(self, X_val, y_val):
        """Evaluate all trained models on validation set"""
        print("\n" + "="*60)
        print("VALIDATION RESULTS")
        print("="*60)
        
        results = {}
        
        for name, model in self.models.items():
            print(f"\n{name}:")
            y_pred = model.predict(X_val)
            accuracy = accuracy_score(y_val, y_pred)
            results[name] = accuracy
            
            print(f"  Accuracy: {accuracy:.4f}")
            print(f"\n  Classification Report:")
            report = classification_report(y_val, y_pred)
            for line in report.split('\n'):
                print(f"    {line}")
        
        # Find best model
        self.best_model_name = max(results, key=results.get)
        self.best_model = self.models[self.best_model_name]
        
        print("\n" + "="*60)
        print(f"BEST MODEL: {self.best_model_name}")
        print(f"Validation Accuracy: {results[self.best_model_name]:.4f}")
        print("="*60)
        
        return results
    
    def final_test(self, X_test, y_test):
        """Test the best model on held-out test set"""
        print("\n" + "="*60)
        print("FINAL TEST SET EVALUATION")
        print("="*60)
        
        y_pred = self.best_model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        
        print(f"\nModel: {self.best_model_name}")
        print(f"Test Accuracy: {accuracy:.4f}")
        
        print(f"\nClassification Report:")
        print(classification_report(y_test, y_pred))
        
        print(f"\nConfusion Matrix:")
        cm = confusion_matrix(y_test, y_pred)
        print(cm)
        print("\n  [TN  FP]")
        print("  [FN  TP]")
        
        # Calculate additional metrics
        tn, fp, fn, tp = cm.ravel()
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
        print(f"\nDetailed Metrics (Intentional class):")
        print(f"  Precision: {precision:.4f} (% of predicted intentional that are correct)")
        print(f"  Recall: {recall:.4f} (% of actual intentional that are detected)")
        print(f"  F1-Score: {f1:.4f}")
        print(f"  False Positive Rate: {fp/(fp+tn):.4f} (unintentional classified as intentional)")
        print(f"  False Negative Rate: {fn/(fn+tp):.4f} (intentional classified as unintentional)")
        
        return accuracy
    
    def show_feature_importance(self):
        """Show feature importance for tree-based models"""
        if hasattr(self.best_model, 'feature_importances_'):
            print("\n" + "="*60)
            print("FEATURE IMPORTANCE")
            print("="*60)
            
            importances = self.best_model.feature_importances_
            indices = np.argsort(importances)[::-1]
            
            print("\nTop 10 most important features:")
            for i in range(min(10, len(indices))):
                idx = indices[i]
                print(f"  {i+1:2d}. {self.feature_columns[idx]:25s}: {importances[idx]:.4f}")
    
    def save_model(self, output_dir="."):
        """Save the trained model and scaler"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        model_filename = f"hand_classifier_{timestamp}.pkl"
        scaler_filename = f"feature_scaler_{timestamp}.pkl"
        
        model_path = os.path.join(output_dir, model_filename)
        scaler_path = os.path.join(output_dir, scaler_filename)
        
        # Save model
        joblib.dump(self.best_model, model_path)
        print(f"\n✅ Model saved to: {model_path}")
        
        # Save scaler
        joblib.dump(self.scaler, scaler_path)
        print(f"✅ Scaler saved to: {scaler_path}")
        
        # Save feature columns for reference
        feature_info = {
            'feature_columns': self.feature_columns,
            'model_name': self.best_model_name
        }
        info_path = os.path.join(output_dir, f"model_info_{timestamp}.pkl")
        joblib.dump(feature_info, info_path)
        print(f"✅ Model info saved to: {info_path}")
        
        return model_path, scaler_path

def main():
    print("""
╔════════════════════════════════════════════════════════════════╗
║              HAND MOVEMENT CLASSIFIER TRAINING                 ║
║         Train ML model for intentional vs unintentional        ║
╚════════════════════════════════════════════════════════════════╝
""")
    
    # Configuration
    DATA_FILE = "training_features.csv"
    
    # Initialize classifier
    classifier = HandMovementClassifier()
    
    # Load data
    X, y = classifier.load_data(DATA_FILE)
    
    # Prepare data (split and scale)
    X_train, X_val, X_test, y_train, y_val, y_test = classifier.prepare_data(X, y)
    
    # Train models
    classifier.train_models(X_train, y_train)
    
    # Evaluate on validation set
    classifier.evaluate_models(X_val, y_val)
    
    # Show feature importance
    classifier.show_feature_importance()
    
    # Final test on held-out test set
    final_accuracy = classifier.final_test(X_test, y_test)
    
    # Save the best model
    model_path, scaler_path = classifier.save_model()
    
    print("\n" + "="*60)
    print("TRAINING COMPLETE!")
    print("="*60)
    print(f"Final test accuracy: {final_accuracy:.4f}")
    print(f"\nNext steps:")
    print("  1. Review the metrics above")
    print("  2. If accuracy is good (>85%), proceed to integration")
    print("  3. If accuracy is low, consider collecting more data or tuning features")
    print(f"\nModel files ready for deployment:")
    print(f"  - {model_path}")
    print(f"  - {scaler_path}")

if __name__ == "__main__":
    main()