#!/usr/bin/env python3
"""
Test Inverse Kinematics for Quadruped
Place in: ros2_ws/src/project/scripts/test_ik.py

Usage: python3 test_ik.py
"""

import numpy as np


class QuadrupedLegIK:
    """Inverse Kinematics solver for 3DOF leg"""
    
    def __init__(self, l1=0.057, l2=0.130, l3=0.138):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.reach_max = l2 + l3
        self.reach_min = abs(l2 - l3)
    
    def solve(self, x, y, z):
        """Solve IK for foot position (x, y, z)"""
        L = np.sqrt(x**2 + z**2)
        theta1 = np.arctan2(z, x)
        
        L_prime = L - self.l1
        D = np.sqrt(L_prime**2 + y**2)
        
        if D > self.reach_max or D < self.reach_min:
            return None
        
        cos_theta3 = (D**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        theta3 = -np.arccos(cos_theta3)
        
        alpha = np.arctan2(y, L_prime)
        cos_beta = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        theta2 = alpha + beta
        
        return (theta1, theta2, theta3)
    
    def forward_kinematics(self, theta1, theta2, theta3):
        """Calculate foot position from joint angles"""
        x1 = self.l1 * np.cos(theta1)
        z1 = self.l1 * np.sin(theta1)
        
        x2 = x1 + self.l2 * np.cos(theta1) * np.cos(theta2)
        y2 = self.l2 * np.sin(theta2)
        z2 = z1 + self.l2 * np.sin(theta1) * np.cos(theta2)
        
        x = x2 + self.l3 * np.cos(theta1) * np.cos(theta2 + theta3)
        y = y2 + self.l3 * np.sin(theta2 + theta3)
        z = z2 + self.l3 * np.sin(theta1) * np.cos(theta2 + theta3)
        
        return (x, y, z)


def test_position(ik, name, x, y, z):
    """Test IK solution for a position"""
    print(f'\n📍 {name}')
    print(f'Target: ({x:.3f}, {y:.3f}, {z:.3f}) m')
    
    angles = ik.solve(x, y, z)
    
    if angles:
        deg = tuple(np.degrees(a) for a in angles)
        print(f'✓ Angles (rad): θ1={angles[0]:.3f}, θ2={angles[1]:.3f}, θ3={angles[2]:.3f}')
        print(f'✓ Angles (deg): θ1={deg[0]:.1f}°, θ2={deg[1]:.1f}°, θ3={deg[2]:.1f}°')
        
        # Verify
        fk = ik.forward_kinematics(*angles)
        error = np.sqrt((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-z)**2)
        print(f'✓ FK check: ({fk[0]:.4f}, {fk[1]:.4f}, {fk[2]:.4f}), error={error*1000:.3f}mm')
        return True
    else:
        print('❌ Unreachable!')
        return False


def main():
    print('='*60)
    print('QUADRUPED INVERSE KINEMATICS TESTER')
    print('='*60)
    
    ik = QuadrupedLegIK(l1=0.057, l2=0.130, l3=0.138)
    
    print(f'Link lengths: l1={ik.l1}m, l2={ik.l2}m, l3={ik.l3}m')
    print(f'Workspace: {ik.reach_min:.3f}m to {ik.reach_max:.3f}m')
    print('='*60)
    
    # Test cases
    test_position(ik, 'Neutral standing', 0.15, -0.20, 0.05)
    test_position(ik, 'Extended', 0.25, -0.10, 0.00)
    test_position(ik, 'Retracted', 0.10, -0.15, 0.03)
    test_position(ik, 'Lifted leg', 0.12, -0.08, 0.03)
    test_position(ik, 'Forward reach', 0.20, -0.18, 0.04)
    test_position(ik, 'Lateral reach', 0.15, -0.20, 0.10)
    test_position(ik, 'Unreachable', 0.40, -0.15, 0.10)
    
    print('\n' + '='*60)
    print('Testing complete!')
    print('='*60)


if __name__ == '__main__':
    main()