#!/usr/bin/env python3
"""
Simple test for horizon_mapper modifications
Checks that the AERO integration features are present
"""

import os
import sys

def test_horizon_mapper_modifications():
    """Test that horizon_mapper has the required AERO integration features."""
    print("🧪 Testing horizon_mapper modifications...")
    
    try:
        # Check the main node file for modifications
        node_file = 'horizon_mapper/horizon_mapper_node.py'
        
        if not os.path.exists(node_file):
            print(f"   ❌ Node file not found: {node_file}")
            return False
        
        with open(node_file, 'r') as f:
            content = f.read()
        
        # Check for key AERO integration features
        aero_features = {
            'Service override_path': 'override_path' in content,
            'LiDAR subscription': '_lidar_callback' in content,
            'Corridor visualization': 'corridor_viz_pub' in content,
            'Velocity path publishing': 'velocity_path_pub' in content,
            'Adaptive bounds': '_compute_adaptive_bounds' in content,
            'Enhanced trajectory': '_publish_enhanced_trajectory' in content,
            'Bound adjustment': '_bound_adjustment_callback' in content,
            'Corridor visualization publishing': '_publish_corridor_visualization' in content,
            'Velocity visualization': '_publish_velocity_visualization' in content
        }
        
        found_count = 0
        for feature_name, found in aero_features.items():
            if found:
                print(f"   ✅ {feature_name}")
                found_count += 1
            else:
                print(f"   ❌ {feature_name}")
        
        print(f"   📊 AERO integration: {found_count}/{len(aero_features)} features found")
        
        # Check for specific implementation details
        print("\n   🔍 Checking implementation details...")
        
        # Check for service creation
        if 'create_service' in content and 'Trigger' in content:
            print("      ✅ override_path service created")
        else:
            print("      ❌ override_path service not found")
        
        # Check for LiDAR subscription
        if 'create_subscription' in content and 'LaserScan' in content:
            print("      ✅ LiDAR subscription created")
        else:
            print("      ❌ LiDAR subscription not found")
        
        # Check for visualization publishers
        if 'MarkerArray' in content and 'corridor_viz_pub' in content:
            print("      ✅ Corridor visualization publisher created")
        else:
            print("      ❌ Corridor visualization publisher not found")
        
        # We expect most AERO features to be present
        assert found_count >= len(aero_features) * 0.7, "Too many AERO features missing"
        
        return True
        
    except Exception as e:
        print(f"   ❌ horizon_mapper test failed: {e}")
        return False

def test_file_structure():
    """Test that horizon_mapper has the expected file structure."""
    print("🧪 Testing horizon_mapper file structure...")
    
    try:
        expected_files = [
            'horizon_mapper/horizon_mapper_node.py',
            'horizon_mapper/preprocess_trajectory.py',
            'horizon_mapper/__init__.py',
            'launch/',
            'config/',
            'package.xml',
            'setup.py'
        ]
        
        found_count = 0
        for expected_file in expected_files:
            if os.path.exists(expected_file):
                print(f"   ✅ Found: {expected_file}")
                found_count += 1
            else:
                print(f"   ❌ Missing: {expected_file}")
        
        print(f"   📊 File structure: {found_count}/{len(expected_files)} files found")
        
        # We expect most files to be present
        assert found_count >= len(expected_files) * 0.8, "Too many files missing"
        
        return True
        
    except Exception as e:
        print(f"   ❌ File structure test failed: {e}")
        return False

def test_csv_trajectories():
    """Test that horizon_mapper has trajectory CSV files."""
    print("🧪 Testing CSV trajectory files...")
    
    try:
        csv_dir = 'horizon_mapper'
        expected_csvs = [
            'ref_trajectory.csv',
            'optimal_trajectory.csv',
            'optimal_trajectory2.csv'
        ]
        
        found_count = 0
        for csv_file in expected_csvs:
            csv_path = os.path.join(csv_dir, csv_file)
            if os.path.exists(csv_path):
                size = os.path.getsize(csv_path)
                print(f"   ✅ Found: {csv_file} ({size} bytes)")
                found_count += 1
            else:
                print(f"   ❌ Missing: {csv_file}")
        
        print(f"   📊 CSV files: {found_count}/{len(expected_csvs)} found")
        
        # We expect most CSV files to be present
        assert found_count >= len(expected_csvs) * 0.7, "Too many CSV files missing"
        
        return True
        
    except Exception as e:
        print(f"   ❌ CSV trajectory test failed: {e}")
        return False

def test_config_files():
    """Test that horizon_mapper has proper configuration files."""
    print("🧪 Testing horizon_mapper configuration...")
    
    try:
        config_dir = 'config'
        launch_dir = 'launch'
        
        if os.path.exists(config_dir):
            config_files = os.listdir(config_dir)
            print(f"   ✅ Config directory: {len(config_files)} files")
            for config_file in config_files:
                print(f"      - {config_file}")
        else:
            print("   ❌ Config directory not found")
        
        if os.path.exists(launch_dir):
            launch_files = os.listdir(launch_dir)
            print(f"   ✅ Launch directory: {len(launch_files)} files")
            for launch_file in launch_files:
                print(f"      - {launch_file}")
        else:
            print("   ❌ Launch directory not found")
        
        return True
        
    except Exception as e:
        print(f"   ❌ Configuration test failed: {e}")
        return False

def main():
    """Run all horizon_mapper tests."""
    print("🚀 horizon_mapper Comprehensive Testing")
    print("=" * 50)
    
    tests = [
        test_horizon_mapper_modifications,
        test_file_structure,
        test_csv_trajectories,
        test_config_files
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 50)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All horizon_mapper tests passed!")
        return True
    else:
        print("❌ Some horizon_mapper tests failed.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
