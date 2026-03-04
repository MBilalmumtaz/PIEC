#!/usr/bin/env python3
import torch
import numpy as np
import sys
from pathlib import Path

# Add path to your module
sys.path.append('/home/amjad/PIEC_2d/src/piec_pinn_surrogate/piec_pinn_surrogate')
try:
    from pinn_model import PhysicsInformedPINN
    print("✅ Successfully imported PhysicsInformedPINN")
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)

def inspect_checkpoint():
    """Inspect what's in the checkpoint file"""
    
    model_path = '/home/amjad/PIEC_2d/src/piec_pinn_surrogate/models/pinn_physics.pt'
    print(f"\n🔍 Inspecting checkpoint: {model_path}")
    
    try:
        checkpoint = torch.load(model_path, map_location='cpu')
        print(f"\n📦 Checkpoint type: {type(checkpoint)}")
        
        if isinstance(checkpoint, dict):
            print(f"\n📋 Keys in checkpoint: {list(checkpoint.keys())}")
            
            # Check each key's type and shape
            for key, value in checkpoint.items():
                if isinstance(value, torch.Tensor):
                    print(f"  - {key}: torch.Tensor, shape={value.shape}")
                elif isinstance(value, dict):
                    print(f"  - {key}: dict, keys={list(value.keys())}")
                else:
                    print(f"  - {key}: {type(value)}")
            
            # Found model state under 'model_state'
            if 'model_state' in checkpoint:
                print(f"\n✅ Found model state dict under key: 'model_state'")
                state_dict = checkpoint['model_state']
                print(f"  State dict keys (first 5): {list(state_dict.keys())[:5]}")
            
            return checkpoint
        
    except Exception as e:
        print(f"❌ Error loading checkpoint: {e}")
        return None

def test_model_variations(model, device, scaler=None):
    """Test if model responds to different inputs"""
    
    # Test different scenarios with proper scaling
    test_cases = [
        # [x, y, yaw, velocity, omega, slope, roughness, obstacle_density, clearance, terrain_type]
        ("Clear path", [0, 0, 0, 1.0, 0, 0, 0.1, 0.0, 2.0, 0]),
        ("Near obstacle", [0, 0, 0, 1.0, 0, 0, 0.1, 0.8, 0.2, 0]),
        ("Steep slope", [0, 0, 0, 1.0, 0, 0.5, 0.2, 0.0, 1.0, 0]),
        ("Rough terrain", [0, 0, 0, 1.0, 0, 0, 0.8, 0.0, 1.0, 1]),
        ("Dense obstacles", [0, 0, 0, 1.0, 0, 0, 0.2, 0.9, 0.1, 0]),
        ("High speed", [0, 0, 0, 2.0, 0, 0, 0.1, 0.0, 2.0, 0]),
        ("Turning", [0, 0, 0, 1.0, 0.5, 0, 0.1, 0.0, 2.0, 0]),
        ("Worst case", [0, 0, 0, 2.0, 0.5, 0.5, 0.8, 0.9, 0.1, 1]),
    ]
    
    print("\n" + "="*70)
    print("Testing PINN Model Response to Different Scenarios")
    print("="*70)
    
    model.eval()
    all_outputs = []
    
    for name, features in test_cases:
        # Convert to tensor
        input_tensor = torch.FloatTensor(features).unsqueeze(0).to(device)
        
        # Apply scaling if available
        if scaler is not None and isinstance(scaler, dict):
            if 'mean' in scaler and 'std' in scaler:
                mean = torch.FloatTensor(scaler['mean']).to(device)
                std = torch.FloatTensor(scaler['std']).to(device)
                std = torch.where(std < 1e-6, torch.ones_like(std), std)
                input_tensor = (input_tensor - mean) / std
        
        # Run inference
        with torch.no_grad():
            output = model(input_tensor)
        
        # Handle output format
        if isinstance(output, tuple):
            energy = output[0][0, 0].item() if isinstance(output[0], torch.Tensor) else output[0]
            raw_stability = output[1][0, 0].item() if isinstance(output[1], torch.Tensor) else output[1]
            stability = 1.0 / (1.0 + np.exp(-raw_stability))  # Sigmoid
        elif isinstance(output, torch.Tensor):
            if output.shape[-1] >= 2:
                energy = output[0, 0].item()
                raw_stability = output[0, 1].item()
                stability = 1.0 / (1.0 + np.exp(-raw_stability))  # Sigmoid
            else:
                energy = output[0, 0].item()
                stability = 0.5
        else:
            energy = float(output[0]) if hasattr(output, '__getitem__') else 0
            stability = 0.5
        
        # Apply inverse scaling for energy if available
        if scaler is not None and isinstance(scaler, dict):
            if 'Y_mean' in scaler and 'Y_std' in scaler:
                Y_mean = scaler['Y_mean'][0] if isinstance(scaler['Y_mean'], (list, np.ndarray)) else scaler['Y_mean']
                Y_std = scaler['Y_std'][0] if isinstance(scaler['Y_std'], (list, np.ndarray)) else scaler['Y_std']
                energy = energy * Y_std + Y_mean
        
        all_outputs.append((energy, stability))
        
        print(f"\n{name}:")
        print(f"  Features: obs_dens={features[7]:.1f}, clearance={features[8]:.1f}, slope={features[5]:.1f}")
        print(f"  Energy: {energy:.2f} J")
        print(f"  Stability: {stability:.3f}")
    
    # Check variation
    energies = [e for e, _ in all_outputs]
    stabilities = [s for _, s in all_outputs]
    
    print("\n" + "="*70)
    print("📊 Summary Statistics:")
    print(f"Energy range: {min(energies):.2f} - {max(energies):.2f} J (Δ={max(energies)-min(energies):.2f})")
    print(f"Stability range: {min(stabilities):.3f} - {max(stabilities):.3f} (Δ={max(stabilities)-min(stabilities):.3f})")
    
    if max(energies) - min(energies) < 10:
        print("\n⚠️ WARNING: Very little energy variation! Model may not be responding to inputs")
    if max(stabilities) - min(stabilities) < 0.1:
        print("⚠️ WARNING: Very little stability variation! Model may not be responding to inputs")
    
    # Provide recommendations
    print("\n🔧 Recommendations:")
    if max(energies) < 100:
        print("  - Energy values are low (<100J). Consider retraining with better scaling")
    if max(stabilities) < 0.8:
        print("  - Stability values capped low. Check sigmoid output")

def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Inspect checkpoint first
    checkpoint = inspect_checkpoint()
    if checkpoint is None:
        print("❌ Failed to load checkpoint")
        return
    
    # Create model
    print("\n🔧 Creating model...")
    input_dim = checkpoint.get('input_dim', 10)
    hidden_dim = checkpoint.get('hidden_dim', 128)
    model = PhysicsInformedPINN(input_dim=input_dim, hidden_dim=hidden_dim)
    
    # Load model state from 'model_state' key
    try:
        if 'model_state' in checkpoint:
            model.load_state_dict(checkpoint['model_state'])
            print("✅ Loaded model from 'model_state' key")
        else:
            print("❌ No 'model_state' key found in checkpoint")
            return
        
        model.to(device)
        print("✅ Model loaded successfully!")
        
        # Get scaler if available
        scaler = checkpoint.get('scaler', None)
        if scaler:
            print("✅ Scaler loaded from checkpoint")
            if 'Y_mean' in scaler:
                print(f"  Y_mean: {scaler['Y_mean']}")
                print(f"  Y_std: {scaler['Y_std']}")
        
        # Test model
        test_model_variations(model, device, scaler)
        
        # Print training info
        print("\n📚 Training Info:")
        print(f"  Epochs: {checkpoint.get('epochs', 'N/A')}")
        print(f"  Train Loss: {checkpoint.get('train_loss', 'N/A'):.6f}")
        print(f"  Val Loss: {checkpoint.get('val_loss', 'N/A'):.6f}")
        print(f"  Physics Weight: {checkpoint.get('physics_weight', 'N/A')}")
        
    except Exception as e:
        print(f"❌ Error loading model: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
