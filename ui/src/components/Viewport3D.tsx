import React, { useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import * as THREE from 'three';

interface Viewport3DProps {
  rodPositionMm: number;
  penetrationDepthMm: number;
  spindleRpm: number;
  spindleTorqueNm: number;
  wobN: number;
}

const MechanicalRig: React.FC<Viewport3DProps> = ({
  rodPositionMm,
  penetrationDepthMm,
  spindleRpm,
  spindleTorqueNm,
  wobN,
}) => {
  const spindleRef = useRef<THREE.Group>(null);
  const contactRingRef = useRef<THREE.Mesh>(null);

  // Scaled coordinates in 3D scene (1 meter in model = 10 units in Three.js)
  const rodOffset = (rodPositionMm / 1000.0) * 8.0;

  useFrame((_, delta) => {
    if (spindleRef.current) {
      // Rotate cutter proportional to spindle RPM
      const omega = (spindleRpm * 2 * Math.PI) / 60;
      spindleRef.current.rotation.x += omega * delta * 0.1;
    }

    if (contactRingRef.current) {
      // Pulse contact ring scale and intensity with WOB force
      const intensity = Math.min(1.0, Math.max(0.2, wobN / 1200.0));
      contactRingRef.current.visible = penetrationDepthMm > 0;
      const mat = contactRingRef.current.material as THREE.MeshBasicMaterial;
      if (mat) {
        mat.opacity = intensity;
      }
    }
  });

  return (
    <group position={[-2, 0, 0]}>
      {/* 1. Fixed Hydraulic Pusher Cylinder Housing */}
      <mesh position={[-3, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.7, 0.7, 4.0, 32]} />
        <meshStandardMaterial color="#475569" metalness={0.7} roughness={0.3} />
      </mesh>
      {/* Pusher mounting flange */}
      <mesh position={[-1.0, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.85, 0.85, 0.3, 32]} />
        <meshStandardMaterial color="#334155" metalness={0.8} roughness={0.2} />
      </mesh>

      {/* 2. Moving Piston Rod & Milling Spindle Head */}
      <group position={[rodOffset, 0, 0]}>
        {/* Polished Chrome Hydraulic Rod */}
        <mesh position={[-1.0, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.3, 0.3, 3.5, 32]} />
          <meshStandardMaterial color="#e2e8f0" metalness={0.95} roughness={0.1} />
        </mesh>

        {/* Spindle PMSM Housing */}
        <mesh position={[1.2, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.6, 0.6, 1.8, 32]} />
          <meshStandardMaterial color="#2563eb" metalness={0.6} roughness={0.4} />
        </mesh>

        {/* Tool Holder Collet */}
        <mesh position={[2.2, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.35, 0.25, 0.5, 32]} />
          <meshStandardMaterial color="#1e293b" metalness={0.9} roughness={0.2} />
        </mesh>

        {/* Rotating Cylindrical Milling Tool */}
        <group ref={spindleRef} position={[2.6, 0, 0]}>
          <mesh rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.18, 0.18, 0.8, 16]} />
            <meshStandardMaterial color="#94a3b8" metalness={0.9} roughness={0.2} />
          </mesh>
          {/* Flute spiral indicators */}
          <mesh position={[0, 0.19, 0]}>
            <boxGeometry args={[0.7, 0.02, 0.04]} />
            <meshStandardMaterial color="#0f172a" />
          </mesh>
          <mesh position={[0, -0.19, 0]}>
            <boxGeometry args={[0.7, 0.02, 0.04]} />
            <meshStandardMaterial color="#0f172a" />
          </mesh>
        </group>

        {/* Dynamic Contact Stress Indicator Ring */}
        <mesh ref={contactRingRef} position={[3.0, 0, 0]} rotation={[0, Math.PI / 2, 0]}>
          <ringGeometry args={[0.2, 0.35, 32]} />
          <meshBasicMaterial
            color={spindleTorqueNm > 6.0 ? "#ef4444" : "#f59e0b"}
            side={THREE.DoubleSide}
            transparent
            opacity={0.8}
          />
        </mesh>
      </group>

      {/* 3. Inconel 718 Spherical Target Workpiece */}
      <group position={[3.8, 0, 0]}>
        <mesh>
          <sphereGeometry args={[1.2, 64, 64]} />
          <meshStandardMaterial
            color="#64748b"
            metalness={0.85}
            roughness={0.25}
          />
        </mesh>
        {/* Dynamic Crater visualization */}
        {penetrationDepthMm > 0 && (
          <mesh position={[-1.15, 0, 0]} rotation={[0, Math.PI / 2, 0]}>
            <circleGeometry args={[Math.min(0.8, 0.18 + penetrationDepthMm * 0.06), 32]} />
            <meshStandardMaterial color="#334155" roughness={0.9} />
          </mesh>
        )}
        {/* Target Fixture Base */}
        <mesh position={[0, -1.5, 0]}>
          <boxGeometry args={[2.8, 0.6, 2.8]} />
          <meshStandardMaterial color="#cbd5e1" roughness={0.6} />
        </mesh>
      </group>
    </group>
  );
};

export const Viewport3D: React.FC<Viewport3DProps> = (props) => {
  return (
    <div className="w-full h-full bg-slate-900 rounded relative overflow-hidden border border-border shadow-inner">
      <div className="absolute top-3 left-3 z-10 bg-slate-900/80 backdrop-blur border border-slate-700/60 rounded px-2.5 py-1 text-[10px] font-mono text-slate-300 flex items-center gap-2">
        <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse" />
        <span>3D KINEMATICS RIG</span>
      </div>

      <Canvas camera={{ position: [0, 2.5, 7.5], fov: 40 }}>
        <ambientLight intensity={0.7} />
        <directionalLight position={[10, 15, 10]} intensity={1.5} castShadow />
        <directionalLight position={[-10, -5, -10]} intensity={0.4} />

        <MechanicalRig {...props} />

        <Grid
          renderOrder={-1}
          position={[0, -1.8, 0]}
          infiniteGrid
          cellSize={0.5}
          cellThickness={0.6}
          cellColor="#334155"
          sectionSize={2.0}
          sectionThickness={1.2}
          sectionColor="#475569"
          fadeDistance={25}
        />
        <OrbitControls makeDefault minDistance={3} maxDistance={15} maxPolarAngle={Math.PI / 2 + 0.1} />
      </Canvas>
    </div>
  );
};
