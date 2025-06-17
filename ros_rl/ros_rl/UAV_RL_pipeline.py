#!/home/cirl/ros2_rl_env/bin/python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import subprocess
import time
import matplotlib.pyplot as plt


class UAVGymEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.step_count = 0
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=np.array([-10.0, -10.0]),
                                            high=np.array([10.0, 10.0]),
                                            dtype=np.float32)

        rclpy.init(args=None)
        self.node = rclpy.create_node('uav_gym_node')

        self.cmd_pub = self.node.create_publisher(Twist, '/x3/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/x3/odometry', self.odom_callback, 10)

        self.goal = np.array([10.0, 0.0])
        self.position = np.array([0.0, 0.0])
        self.last_odom_received = False

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.last_odom_received = True

    def reset(self, seed=None, options=None):
        self.step_count = 0
        print("Reset!!!")
        super().reset(seed=seed)
        self.reset_gazebo_world()
        time.sleep(1.0)
        self.send_velocity(0.0, 0.0)

        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if np.linalg.norm(self.position) < 1.0:
                break

        self.last_odom_received = False
        return self.position.copy(), {}

    def reset_gazebo_world(self):
        result = subprocess.run(
            [
                "gz", "service", "-s", "/world/empty/control",
                "--reqtype", "gz.msgs.WorldControl",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000",
                "--req", "reset: {all: true}"
            ],
            capture_output=True,
            text=True
        )
        print("Reset result:", result.stdout)
        if result.returncode != 0:
            print("Reset error:", result.stderr)

    def step(self, action):
        self.step_count += 1

        # # Velocity actions
        # if action == 0:
        #     self.send_velocity(0.5, 0.0)   # +x
        # elif action == 1:
        #     self.send_velocity(-0.5, 0.0)  # -x
        # elif action == 2:
        #     self.send_velocity(0.0, 0.5)   # +y
        # elif action == 3:
        #     self.send_velocity(0.0, -0.5)  # -y

        if action == 0:
            self.send_velocity(5.0, 0.0)   # +x
        elif action == 1:
            self.send_velocity(0.0, 5.0)   # +y
        elif action == 2:
            self.send_velocity(0.0, -5.0)  # -y

        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Check for boundary violation
        out_of_bounds = (
            self.position[0] < 0.0 or self.position[0] > 10.0 or
            self.position[1] < -5.0 or self.position[1] > 5.0
        )
        if out_of_bounds:
            print("Out of bounds! Episode terminated.")
            return self.position.copy(), -2000.0, True, False, {}

        # Calculate distance to goal
        dist = np.linalg.norm(self.goal - self.position)

        # Reward logic
        if dist < 0.2:
            reward = 10.0  # Goal reached
            done = True
        else:
            reward = -0.1 * dist  # Penalize distance
            done = False

        # Optional: End if too many steps
        if self.step_count >= 200:
            done = True

        return self.position.copy(), reward, done, False, {}

    def send_velocity(self, x_vel, y_vel):
        twist = Twist()
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        twist.linear.z = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    env = UAVGymEnv()
    rewards = []

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], label="Episode Reward")
    ax.set_xlabel('Episode')
    ax.set_ylabel('Reward')
    ax.set_title('UAV RL Reward per Episode')
    ax.legend()

    for episode in range(1000):
        print(f"\n=== Episode {episode + 1} ===")
        obs, _ = env.reset()
        done = False
        total_reward = 0.0
        while not done:
            action = env.action_space.sample()
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward
            print(f"obs={obs}, reward={reward:.2f}, done={done}")

        rewards.append(total_reward)
        line.set_xdata(np.arange(len(rewards)))
        line.set_ydata(rewards)
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    plt.ioff()
    plt.show()
    env.close()


if __name__ == '__main__':
    main()