import argparse
import os
import sys
import time
import struct
import csv
import gc
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

TEST_LQR_ONLY = False

# Proportional Gains
K1 = float(-4.4681 * 0.25)
K2 = float(-1.6210 * 0.50)
K3 = float(-39.5037 * 0.75)
K4 = float(-5.0402 * 0.60)
K5 = float(-0.4387 * 0.50)  # The 5th state gain (Anticipatory Brake)

#Serial Communication

class HardwareInterface:
    """Communicate with Arduino running Furuta firmware."""
    
    BAUD_RATE = 500000
    PACKET_SIZE = 16  # 4 floats
    FULL_PACKET_SIZE = 18  # sync word (2) + data (16)
    SYNC_WORD = b'\xcd\xab'
    
    def __init__(self, port: str):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = self.BAUD_RATE
        self.ser.timeout = 0  # CRITICAL: Non-blocking mode for zero latency
        
        # MAC FIX: Force hardware lines low BEFORE opening
        self.ser.dtr = False 
        self.ser.rts = False 
        self.ser.open()
        
        # MAC FIX: Trigger clean reset
        time.sleep(float(0.1))
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(float(2.0))  # Wait for bootloader
        self.ser.reset_input_buffer()
    
    def get_sensor_data(self) -> tuple:
        """
        Ultra-low latency read: Bypasses macOS in_waiting bugs.
        Uses non-blocking bulk reads to grab data in a single rapid OS call.
        Blocks perfectly until the Arduino's 10ms hardware timer fires.
        """
        raw_buffer = b''
        while True:
            chunk = self.ser.read(1024)
            if chunk:
                raw_buffer += chunk
                last_sync_idx = raw_buffer.rfind(self.SYNC_WORD)
                
                if last_sync_idx != -1 and (len(raw_buffer) - last_sync_idx) >= self.FULL_PACKET_SIZE:
                    start = last_sync_idx + 2
                    data = raw_buffer[start : start + self.PACKET_SIZE]
                    try:
                        return struct.unpack('<ffff', data)
                    except:
                        pass # If unpacking fails due to corruption, just wait for next packet
            pass # Zero-latency CPU polling
    
    def send_voltage(self, voltage: float):
        """Send voltage command to motor and force immediate USB transfer."""
        self.ser.write(struct.pack('f', float(voltage)))
        self.ser.flush() # MAC FIX: Forces OS to send data immediately
    
    def close(self):
        self.send_voltage(float(0.0))
        self.ser.close()


#Reward & State Processing

def compute_reward(pend_pos: float, arm_pos: float, pend_vel: float, arm_vel: float, voltage: float, prev_voltage: float = 0.0) -> float:
    """Compute reward (optimized for hardware fine-tuning)."""
    r = (float(20.0)  * -(float(pend_pos) ** float(2))   # theta — pendulum angle (most important)
       + float(5.0)   * -(float(pend_vel) ** float(2))   # theta_dot — damping term
       + float(1.0)   * -(float(arm_pos) ** float(2))    # alpha — arm angle (baseline)
       + float(1.0)   * -(float(arm_vel) ** float(2))    # alpha_dot — arm velocity (baseline)
       + float(0.5)   * -(float(voltage) ** float(2))    # control effort
       + float(2.0)   * -((float(voltage) - float(prev_voltage)) ** float(2))  # smoothness penalty
       + float(1.0))                                     # alive bonus

    return float(r)

def apply_deadzone_model(voltage: float, deadzone: float = float(0.5), threshold: float = float(0.5), cap: float = float(12.0)) -> float:
    voltage = float(voltage)
    deadzone = float(deadzone)
    threshold = float(threshold)
    cap = float(cap)
    if voltage > threshold:
        voltage += deadzone
    elif voltage < -threshold:
        voltage -= deadzone
    else:
        voltage = float(0.0)

    return float(np.clip(float(voltage), float(-cap), float(cap)))

def state_to_obs(
    pend_pos: float,
    arm_pos: float,
    pend_vel: float,
    arm_vel: float,
    last_voltage: float,
    voltage_cap: float,
    include_prev_voltage: bool,
) -> np.ndarray:
    obs = [
        float(np.cos(float(pend_pos))), float(np.sin(float(pend_pos))),
        float(np.cos(float(arm_pos))), float(np.sin(float(arm_pos))),
        float(pend_vel) / float(15.0),
        float(arm_vel) / float(25.0),
    ]

    if include_prev_voltage:
        obs.append(float(np.clip(float(last_voltage) / float(max(float(voltage_cap), float(1e-6))), float(-1.0), float(1.0))))

    return np.array(obs, dtype=np.float32)


#Main

def main():
    parser = argparse.ArgumentParser(description="Hardware fine-tuning")
    
    parser.add_argument('--port', default="/dev/cu.usbmodem1401", required=False, help='Serial port (e.g., /dev/cu.usbmodem1401)')
    parser.add_argument('--vcap', type=float, default=float(12.0), help='Voltage cap for Proportional safety net')
    parser.add_argument('--rl-vcap', type=float, default=float(12.0), help='Voltage cap for RL (defines action semantics, keep FIXED across training)')
    parser.add_argument('--rl-voltage-limit', type=float, default=None, help='Hard clip on applied RL voltage (adjustable for curriculum, defaults to --rl-vcap)')
    parser.add_argument('--safe-angle', type=float, default=float(0.40), help='Pendulum safety cutoff (~23°)')
    parser.add_argument('--resume-angle', type=float, default=float(np.radians(4)), help='Angle to resume control (°)')
    parser.add_argument('--rl-angle', type=float, default=float(4.0), help='RL control boundary in degrees (SAC controls within, proportional catches outside)')
    parser.add_argument('--boundary-penalty', type=float, default=float(-50.0), help='One-time reward penalty when SAC lets pendulum escape --rl-angle')
    parser.add_argument('--warmup-steps', type=int, default=50, help='Steps to send 0V while settling')
    
    parser.add_argument('--total-steps', type=int, default=0)
    parser.add_argument('--gradient-steps', type=int, default=50, help='Updates performed DURING FALLEN STATE')
    parser.add_argument('--online-gradient-steps', type=int, default=1, help='SAC updates during normal hardware control')
    parser.add_argument('--train-every', type=int, default=25, help='Run online SAC updates every N collected transitions')
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--buffer-size', type=int, default=200000)
    parser.add_argument(
        '--include-prev-voltage',
        action=argparse.BooleanOptionalAction,
        default=None,
        help='Append previous applied voltage to the RL observation',
    )
    
    parser.add_argument('--model', default=None)
    parser.add_argument('--resume', default=None)
    parser.add_argument('--log', default='hardware_log.csv')
    parser.add_argument('--save-every', type=int, default=1000)
    
    args = parser.parse_args()
    if args.include_prev_voltage is None:
        args.include_prev_voltage = True

    if args.rl_voltage_limit is None:
        args.rl_voltage_limit = args.rl_vcap

    args.rl_angle_rad = float(np.radians(float(args.rl_angle)))

    device = torch.device("cpu")
    
    print("\n" + "="*70)
    print("Furuta Hardware Fine-Tuning — SAC + proportional")
    print("="*70)
    print(f"  Port: {args.port}")
    print(f"  Proportional V-Cap: {args.vcap:.1f} V")
    print(f"  RL V-Cap (fixed): {args.rl_vcap:.1f} V")
    print(f"  RL Voltage Limit: {args.rl_voltage_limit:.1f} V")
    print(f"  RL Angle Boundary: {args.rl_angle:.1f}° ({float(np.degrees(args.rl_angle_rad)):.2f}°)")
    print(f"  Boundary Penalty: {args.boundary_penalty:.1f}")
    print(f"  Previous voltage in obs: {'yes' if args.include_prev_voltage else 'no'}")
    
    os.makedirs('models', exist_ok=True)
    log_file = open(args.log, 'w', newline='')
    log_writer = csv.writer(log_file)
    log_writer.writerow(['step', 'pend_rad', 'arm_rad', 'pend_vel', 'arm_vel', 'voltage', 'reward', 'late', 'controller'])
    
    print("\nConnecting to hardware...")
    try:
        hw = HardwareInterface(args.port)
        print("✓ Connected")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    

    print("\nInitializing SAC model...")

    class HardwareShapeEnv(gym.Env):
        def __init__(self, include_prev_voltage: bool):
            obs_dim = 7 if include_prev_voltage else 6
            self.observation_space = spaces.Box(
                low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32
            )
            self.action_space = spaces.Box(
                low=-1.0, high=1.0, shape=(1,), dtype=np.float32
            )
        def reset(self, **kwargs): return np.zeros(self.observation_space.shape, dtype=np.float32), {}
        def step(self, action): return np.zeros(self.observation_space.shape, dtype=np.float32), 0.0, False, False, {}
    
    _dummy_env = DummyVecEnv([lambda: HardwareShapeEnv(args.include_prev_voltage)])
    if args.resume:
        model = SAC.load(args.resume.replace('.zip', ''), env=_dummy_env, device=device)
    elif args.model:
        model = SAC.load(args.model, env=_dummy_env, device=device)
    else:
        model = SAC(
        "MlpPolicy",
        _dummy_env,
        learning_rate=float(1e-3),
        buffer_size=args.buffer_size,
        batch_size=args.batch_size,
        use_sde=True,
        sde_sample_freq=100,
        device=device,
        verbose=0
        )

    from stable_baselines3.common.logger import configure
    model.set_logger(configure(folder=None, format_strings=[]))
    model.batch_size = args.batch_size
    
    print("✓ Model ready\n")
    
    step = 0
    late_count = 0
    ep_reward = float(0.0)
    last_voltage = float(0.0)   # voltage applied in the previous step
    prev_voltage = float(0.0)   # voltage applied two steps ago (for smoothness penalty)
    last_obs = None
    last_action = None
    transition_count = 0
    fallen = False
    last_controller_was_sac = False
    
    # Disable Garbage Collection to prevent loop stuttering
    gc.disable()
    
    try:
        input("\n  >>> Hold pendulum UPRIGHT and press Enter to start <<<\n")
        
        for _ in range(args.warmup_steps):
            hw.send_voltage(float(0.0))
            time.sleep(float(0.01))

        t_start = time.perf_counter()

        while args.total_steps == 0 or step < args.total_steps:
            sensor_data = hw.get_sensor_data()
            if sensor_data is None: continue
            
            t_compute_start = time.perf_counter()
            pend_pos, arm_pos, pend_vel, arm_vel = float(sensor_data[0]), float(sensor_data[1]), float(sensor_data[2]), float(sensor_data[3])
            pend_pos = float(((float(pend_pos) + float(np.pi)) % (float(2.0) * float(np.pi))) - float(np.pi))
            obs = state_to_obs(
                pend_pos,
                arm_pos,
                pend_vel,
                arm_vel,
                last_voltage,
                args.rl_vcap,
                args.include_prev_voltage,
            )
            reward = float(compute_reward(float(pend_pos), float(arm_pos), float(pend_vel), float(arm_vel), float(last_voltage), float(prev_voltage)))
            done = abs(float(pend_pos)) > float(args.safe_angle)

            # Boundary-crossing penalty: fire once when SAC loses control
            if last_controller_was_sac and abs(float(pend_pos)) > float(args.rl_angle_rad):
                reward += float(args.boundary_penalty)

            ep_reward += reward

            # Store the transition caused by the previous applied voltage.
            if last_obs is not None and last_action is not None and not TEST_LQR_ONLY:
                model.replay_buffer.add(
                    obs=last_obs,
                    next_obs=obs,
                    action=last_action,
                    reward=np.array([reward]),
                    done=np.array([done]),
                    infos=[{}],
                )
                transition_count += 1

                should_train_online = (
                    args.online_gradient_steps > 0
                    and args.train_every > 0
                    and transition_count % args.train_every == 0
                    and model.replay_buffer.size() > args.batch_size
                )
                if should_train_online:
                    model.train(gradient_steps=args.online_gradient_steps, batch_size=args.batch_size)
            
            # 3. Safety Cutoff & TRAIN-ON-FAIL HACK
            if done:
                if not fallen:
                    fallen = True
                    hw.send_voltage(float(0.0))
                    last_voltage = float(0.0)
                    prev_voltage = float(0.0)  # CRITICAL: reset so penalty doesn't spike on resume
                    last_obs = None
                    last_action = None
                    print(f"{step:>6d} | FALLEN — motor stopped.")
                    
                    if not TEST_LQR_ONLY and model.replay_buffer.size() > args.batch_size:
                        print(f"         [PyTorch Training... doing {args.gradient_steps} updates while safe]")
                        model.train(gradient_steps=args.gradient_steps, batch_size=args.batch_size)
                        print("Hiiii")
                        
                hw.send_voltage(float(0.0))
                continue
            
            # 4. Recovery Routine
            if fallen:
                if abs(float(pend_pos)) > float(args.resume_angle):
                    hw.send_voltage(float(0.0))
                    continue
                print(f"{step:>6d} | ✓ RECOVERED (resuming)")
                fallen = False
                last_obs = None 
                last_action = None
                
                if not TEST_LQR_ONLY and getattr(model, 'use_sde', False):
                    model.policy.reset_noise(1)
                    
                continue
            
            # 5. Controller Selection
            active_controller = "Proportional"
            
            if TEST_LQR_ONLY:
                #Just proportional
                u = -(float(K1) * float(arm_pos) + float(K2) * float(arm_vel) + float(K3) * float(pend_pos) + float(K4) * float(pend_vel) + float(K5) * float(last_voltage))
                voltage = float(np.clip(float(u), float(-args.vcap), float(args.vcap)))
                action = np.array([float(voltage) / float(args.vcap)]) 
            else:
                # PHASE 2: Hybrid Safety Envelope
                if abs(float(pend_pos)) < float(args.rl_angle_rad):

                    # SAC Control
                    active_controller = "SAC"
                    action, _ = model.predict(obs, deterministic=True) 
                    
                    voltage = float(action[0]) * float(args.rl_vcap) if isinstance(action, np.ndarray) else float(action) * float(args.rl_vcap)
                    voltage = float(np.clip(float(voltage), float(-args.rl_voltage_limit), float(args.rl_voltage_limit)))  # curriculum clip
                    voltage = float(apply_deadzone_model(float(voltage)))
                else:
                    # Proportional
                    active_controller = "Proportional_Catch"
                    u = -(float(K1) * float(arm_pos) + float(K2) * float(arm_vel) + float(K3) * float(pend_pos) + float(K4) * float(pend_vel) + float(K5) * float(last_voltage))
                    
                    voltage = float(np.clip(float(u), float(-args.vcap), float(args.vcap)))
                    voltage = float(apply_deadzone_model(float(voltage)))
                    action = np.array([float(np.clip(float(voltage) / float(args.rl_vcap), float(-1.0), float(1.0)))])

            # Track which controller was active for boundary penalty
            last_controller_was_sac = (active_controller == "SAC")
            
            # 6. Send voltage
            hw.send_voltage(float(voltage))
            prev_voltage = last_voltage   # shift: t-1 becomes t-2 for next step's smoothness penalty
            last_voltage = float(voltage)
            last_obs = obs
            last_action = np.array([float(action[0])], dtype=np.float32) if isinstance(action, np.ndarray) else np.array([float(action)], dtype=np.float32)
            
            # 8. Check Latency Metric
            t_compute_elapsed = time.perf_counter() - t_compute_start
            if t_compute_elapsed > float(0.003):
                late_count += 1
                
            log_writer.writerow([step, float(pend_pos), float(arm_pos), float(pend_vel), float(arm_vel), float(voltage), float(reward), late_count, active_controller])
            
            if step % 10 == 0:
                print(f"Step {step:6d} | θ: {float(np.degrees(float(pend_pos))):+6.1f}° | theta_dot: {float(np.degrees(float(pend_vel))):+6.1f}° | alpha: {float(np.degrees(float(arm_pos))):+6.1f}° | alpha_dot: {float(np.degrees(float(arm_vel))):+6.1f}° | V: {float(voltage):+6.2f} | late: {late_count} | ctrl: {active_controller}")
            
            if args.save_every > 0 and (step + 1) % args.save_every == 0:
                model.save(f"models/hardware_ckpt_{step+1:06d}.zip")
            
            step += 1
    
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        hw.close()
        log_file.close()
        gc.enable()
        if not TEST_LQR_ONLY:
            model.save("models/hardware_final")
            model.save("models/hardware_ckpt_latest")

if __name__ == "__main__":
    main()