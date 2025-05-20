    def plan_and_send_arm_trajectory(self):
        pose = self.current_pose.pose
        target_se3 = SE3(pose.position.x, pose.position.y, pose.position.z)
        q0 = self.current_joint_state if self.current_joint_state is not None else self.q0
        N = 50
        Ts = self.robot.fkine(q0).interp(target_se3, N)
        q_traj = []
        q_curr = q0
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warn("IK non trovata per uno degli step. Interrompo la traiettoria.")
                break
        if not q_traj:
            self.get_logger().error("Nessuna soluzione IK trovata. Comando non inviato.")
            return

        # --- Split arm e torso ---

        # Torso trajectory
        torso_traj = JointTrajectory()
        torso_traj.joint_names = ['torso_lift_joint']

        # Arm trajectory
        arm_traj = JointTrajectory()
        arm_traj.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            # q dev'essere di lunghezza 8: [torso, arm_1,..., arm_7]
            if len(q) == 8:
                q = q.tolist()
            else:
                self.get_logger().warn("Soluzione IK non di lunghezza 8! Skipping step.")
                continue

            # Torso
            torso_point = JointTrajectoryPoint()
            torso_point.positions = [q[0]]      # solo torso
            torso_point.time_from_start.sec = int(i * 0.1)
            torso_traj.points.append(torso_point)

            # Arm
            arm_point = JointTrajectoryPoint()
            arm_point.positions = q[1:8]       # solo arm
            arm_point.time_from_start.sec = int(i * 0.1)
            arm_traj.points.append(arm_point)

        # Pubblica separatamente
        self.torso_pub.publish(torso_traj)
        self.arm_pub.publish(arm_traj)
        self.get_logger().info("Inviata traiettoria separata per torso e braccio.")
