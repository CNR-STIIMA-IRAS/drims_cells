import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive


class AddTableNode(Node):
    def __init__(self):
        super().__init__(
            'table_scene_publisher_node',
            automatically_declare_parameters_from_overrides=True
        )

        def get_param(name: str, default_val):
            if self.has_parameter(name):
                return self.get_parameter(name).value
            return self.declare_parameter(name, default_val).value

        # Main table dimensions & position
        L = float(get_param('table_length', 1.0))
        W = float(get_param('table_width', 1.5))
        T = float(get_param('table_thickness', 0.05))
        leg_T = float(get_param('leg_thickness', 0.05))
        leg_H = float(get_param('leg_height', 0.74))
        pos_x = float(get_param('table_pos_x', 0.85))
        pos_y = float(get_param('table_pos_y', 0.0))
        pos_z = float(get_param('table_pos_z', 0.0))

        # Feature toggles & custom geometry parameters
        enable_cam = bool(get_param('enable_camera_structure', True))
        enable_pc = bool(get_param('enable_pc_box', False))
        pc_L = float(get_param('pc_box_length', 0.35))
        pc_W = float(get_param('pc_box_width', 0.45))
        pc_H = float(get_param('pc_box_height', 0.40))
        pc_x = float(get_param('pc_box_pos_x', 0.0))
        pc_y = float(get_param('pc_box_pos_y', 0.50))
        pc_z = float(get_param('pc_box_pos_z', 0.225))

        enable_barriers = bool(get_param('enable_safety_barriers', False))
        barrier_T = float(get_param('barrier_thickness', 0.02))
        barrier_H = float(get_param('barrier_height', 0.80))
        barrier_A = float(get_param('barrier_alpha', 0.3))

        # Barrier wall lengths (side walls run along X, end walls run along Y).
        # Default to the table's own footprint if not explicitly overridden.
        barrier_side_L = float(get_param('barrier_side_length', L))
        barrier_end_L = float(get_param('barrier_end_length', W + 2.0 * barrier_T))

        # Barrier center position wrt base_footprint (defaults to pos_x, pos_y, and calculated table top Z if omitted)
        default_barrier_z = pos_z + leg_H + T + barrier_H / 2.0
        barrier_x = float(get_param('barrier_pos_x', pos_x))
        barrier_y = float(get_param('barrier_pos_y', pos_y))
        barrier_z = float(get_param('barrier_pos_z', default_barrier_z))

        # Table top Z level
        table_top_z = pos_z + leg_H + T

        # Individual barrier side toggles (+Y, -Y, +X, -X)
        b_pos_y = bool(get_param('enable_barrier_side_pos_y', True))
        b_neg_y = bool(get_param('enable_barrier_side_neg_y', True))
        b_pos_x = bool(get_param('enable_barrier_end_pos_x', True))
        b_neg_x = bool(get_param('enable_barrier_end_neg_x', True))

        # Calculated default min-corner positions (min_x, min_y, min_z) for each barrier
        def_b1_x, def_b1_y, def_b1_z = pos_x - barrier_side_L / 2.0, pos_y + W / 2.0, table_top_z
        def_b2_x, def_b2_y, def_b2_z = pos_x - barrier_side_L / 2.0, pos_y - W / 2.0 - barrier_T, table_top_z
        def_b3_x, def_b3_y, def_b3_z = pos_x + L / 2.0, pos_y - barrier_end_L / 2.0, table_top_z
        def_b4_x, def_b4_y, def_b4_z = pos_x - L / 2.0 - barrier_T, pos_y - barrier_end_L / 2.0, table_top_z

        # Individual wall min corner X, Y, Z parameters wrt base_footprint (Wall 1: +Y, Wall 2: -Y, Wall 3: +X, Wall 4: -X)
        b1_x = float(get_param('barrier_1_pos_x', def_b1_x))
        b1_y = float(get_param('barrier_1_pos_y', def_b1_y))
        b1_z = float(get_param('barrier_1_pos_z', def_b1_z))

        b2_x = float(get_param('barrier_2_pos_x', def_b2_x))
        b2_y = float(get_param('barrier_2_pos_y', def_b2_y))
        b2_z = float(get_param('barrier_2_pos_z', def_b2_z))

        b3_x = float(get_param('barrier_3_pos_x', def_b3_x))
        b3_y = float(get_param('barrier_3_pos_y', def_b3_y))
        b3_z = float(get_param('barrier_3_pos_z', def_b3_z))

        b4_x = float(get_param('barrier_4_pos_x', def_b4_x))
        b4_y = float(get_param('barrier_4_pos_y', def_b4_y))
        b4_z = float(get_param('barrier_4_pos_z', def_b4_z))

        self.get_logger().info(
            f"Loaded Table Params -> L={L}, W={W}, H={leg_H}, pos=({pos_x}, {pos_y}, {pos_z}) | "
            f"enable_cam={enable_cam}, enable_pc={enable_pc}, enable_barriers={enable_barriers} (pos=({barrier_x}, {barrier_y}, {barrier_z}))"
        )



        scene = PlanningScene()
        scene.is_diff = True



        # -------------------------------
        # Table top (brown)
        table_top = CollisionObject()
        table_top.id = "table_top"
        table_top.header.frame_id = "base_footprint"

        top_primitive = SolidPrimitive()
        top_primitive.type = SolidPrimitive.BOX
        top_primitive.dimensions = [L, W, T]

        top_pose = PoseStamped()
        top_pose.header.frame_id = "base_footprint"
        top_pose.pose.position.x = pos_x
        top_pose.pose.position.y = pos_y
        top_pose.pose.position.z = pos_z + leg_H + T / 2.0
        top_pose.pose.orientation.w = 1.0

        table_top.primitives.append(top_primitive)
        table_top.primitive_poses.append(top_pose.pose)
        table_top.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table_top)

        top_color = ObjectColor()
        top_color.id = "table_top"
        top_color.color.r = 0.55  # brown
        top_color.color.g = 0.27
        top_color.color.b = 0.07
        top_color.color.a = 1.0
        scene.object_colors.append(top_color)

        # -------------------------------
        # Optional: Camera structure (base, vertical, horizontal bars)
        if enable_cam:
            camera_base = CollisionObject()
            camera_base.id = "camera_base"
            camera_base.header.frame_id = "table_top"

            camera_base_primitive = SolidPrimitive()
            camera_base_primitive.type = SolidPrimitive.BOX
            camera_base_primitive.dimensions = [L, W / 8, 1.5 * T]

            camera_base_pose = PoseStamped()
            camera_base_pose.header.frame_id = "table_top"
            camera_base_pose.pose.position.x = 0.0
            camera_base_pose.pose.position.y = -W / 2 + W / 20
            camera_base_pose.pose.position.z = T
            camera_base_pose.pose.orientation.w = 1.0

            camera_base.primitives.append(camera_base_primitive)
            camera_base.primitive_poses.append(camera_base_pose.pose)
            camera_base.operation = CollisionObject.ADD
            scene.world.collision_objects.append(camera_base)

            camera_base_color = ObjectColor()
            camera_base_color.id = "camera_base"
            camera_base_color.color.r = 0.5
            camera_base_color.color.g = 0.5
            camera_base_color.color.b = 0.5
            camera_base_color.color.a = 1.0
            scene.object_colors.append(camera_base_color)

            camera_vertical_L = W / 8
            camera_vertical_H = 0.60
            camera_vertical = CollisionObject()
            camera_vertical.id = "camera_vertical"
            camera_vertical.header.frame_id = "camera_base"

            camera_vertical_primitive = SolidPrimitive()
            camera_vertical_primitive.type = SolidPrimitive.BOX
            camera_vertical_primitive.dimensions = [camera_vertical_L, camera_vertical_L, camera_vertical_H]

            camera_vertical_pose = PoseStamped()
            camera_vertical_pose.header.frame_id = "camera_base"
            camera_vertical_pose.pose.position.x = -0.37
            camera_vertical_pose.pose.position.y = 0.0
            camera_vertical_pose.pose.position.z = T / 2 + camera_vertical_H / 2.0
            camera_vertical_pose.pose.orientation.w = 1.0

            camera_vertical.primitives.append(camera_vertical_primitive)
            camera_vertical.primitive_poses.append(camera_vertical_pose.pose)
            camera_vertical.operation = CollisionObject.ADD
            scene.world.collision_objects.append(camera_vertical)

            camera_vertical_color = ObjectColor()
            camera_vertical_color.id = "camera_vertical"
            camera_vertical_color.color.r = 0.5
            camera_vertical_color.color.g = 0.5
            camera_vertical_color.color.b = 0.5
            camera_vertical_color.color.a = 1.0
            scene.object_colors.append(camera_vertical_color)

            camera_horizontal_L = W / 8
            camera_horizontal_H = 0.8
            camera_horizontal = CollisionObject()
            camera_horizontal.id = "camera_horizontal"
            camera_horizontal.header.frame_id = "camera_vertical"

            camera_horizontal_primitive = SolidPrimitive()
            camera_horizontal_primitive.type = SolidPrimitive.BOX
            camera_horizontal_primitive.dimensions = [camera_horizontal_L, camera_horizontal_H, camera_horizontal_L]

            camera_horizontal_pose = PoseStamped()
            camera_horizontal_pose.header.frame_id = "camera_vertical"
            camera_horizontal_pose.pose.position.x = 0.0
            camera_horizontal_pose.pose.position.y = camera_horizontal_H / 2 - camera_vertical_L / 2
            camera_horizontal_pose.pose.position.z = camera_vertical_H / 2 + camera_horizontal_L / 2
            camera_horizontal_pose.pose.orientation.w = 1.0

            camera_horizontal.primitives.append(camera_horizontal_primitive)
            camera_horizontal.primitive_poses.append(camera_horizontal_pose.pose)
            camera_horizontal.operation = CollisionObject.ADD
            scene.world.collision_objects.append(camera_horizontal)

            camera_horizontal_color = ObjectColor()
            camera_horizontal_color.id = "camera_horizontal"
            camera_horizontal_color.color.r = 0.5
            camera_horizontal_color.color.g = 0.5
            camera_horizontal_color.color.b = 0.5
            camera_horizontal_color.color.a = 1.0
            scene.object_colors.append(camera_horizontal_color)

        # -------------------------------
        # Optional: Lateral PC Box (restricted space occupied by PC)
        if enable_pc:
            pc_box = CollisionObject()
            pc_box.id = "pc_box"
            pc_box.header.frame_id = "table_top"

            pc_primitive = SolidPrimitive()
            pc_primitive.type = SolidPrimitive.BOX
            pc_primitive.dimensions = [pc_L, pc_W, pc_H]

            pc_pose = PoseStamped()
            pc_pose.header.frame_id = "table_top"
            pc_pose.pose.position.x = pc_x
            pc_pose.pose.position.y = pc_y
            pc_pose.pose.position.z = T / 2.0 + pc_H / 2.0
            pc_pose.pose.orientation.w = 1.0

            pc_box.primitives.append(pc_primitive)
            pc_box.primitive_poses.append(pc_pose.pose)
            pc_box.operation = CollisionObject.ADD
            scene.world.collision_objects.append(pc_box)

            pc_color = ObjectColor()
            pc_color.id = "pc_box"
            pc_color.color.r = 0.25  # Dark blue-grey
            pc_color.color.g = 0.35
            pc_color.color.b = 0.45
            pc_color.color.a = 1.0
            scene.object_colors.append(pc_color)

        # -------------------------------
        # Table legs (gray)
        leg_offsets = [
            (L / 2 - leg_T / 2, W / 2 - leg_T / 2),
            (L / 2 - leg_T / 2, -W / 2 + leg_T / 2),
            (-L / 2 + leg_T / 2, W / 2 - leg_T / 2),
            (-L / 2 + leg_T / 2, -W / 2 + leg_T / 2),
        ]

        for i, (dx, dy) in enumerate(leg_offsets):
            leg = CollisionObject()
            leg.id = f"table_leg_{i + 1}"
            leg.header.frame_id = "base_footprint"

            leg_primitive = SolidPrimitive()
            leg_primitive.type = SolidPrimitive.BOX
            leg_primitive.dimensions = [leg_T, leg_T, leg_H]

            leg_pose = PoseStamped()
            leg_pose.header.frame_id = "base_footprint"
            leg_pose.pose.position.x = pos_x + dx
            leg_pose.pose.position.y = pos_y + dy
            leg_pose.pose.position.z = pos_z + leg_H / 2.0
            leg_pose.pose.orientation.w = 1.0

            leg.primitives.append(leg_primitive)
            leg.primitive_poses.append(leg_pose.pose)
            leg.operation = CollisionObject.ADD
            scene.world.collision_objects.append(leg)

            leg_color = ObjectColor()
            leg_color.id = leg.id
            leg_color.color.r = 0.5
            leg_color.color.g = 0.5
            leg_color.color.b = 0.5
            leg_color.color.a = 1.0
            scene.object_colors.append(leg_color)

        # -------------------------------
        # Optional: Safety barriers (yellow semi-transparent virtual walls enclosing the table volume)
        barrier_configs = [
            ("safety_barrier_1", b_pos_y, [barrier_side_L, barrier_T, barrier_H], (b1_x, b1_y, b1_z)),
            ("safety_barrier_2", b_neg_y, [barrier_side_L, barrier_T, barrier_H], (b2_x, b2_y, b2_z)),
            ("safety_barrier_3", b_pos_x, [barrier_T, barrier_end_L, barrier_H], (b3_x, b3_y, b3_z)),
            ("safety_barrier_4", b_neg_x, [barrier_T, barrier_end_L, barrier_H], (b4_x, b4_y, b4_z)),
        ]

        for bid, is_enabled, dims, (bx, by, bz) in barrier_configs:
            if enable_barriers and is_enabled:
                barrier = CollisionObject()
                barrier.id = bid
                barrier.header.frame_id = "base_footprint"

                barrier_primitive = SolidPrimitive()
                barrier_primitive.type = SolidPrimitive.BOX
                barrier_primitive.dimensions = dims

                barrier_pose = PoseStamped()
                barrier_pose.header.frame_id = "base_footprint"
                barrier_pose.pose.position.x = bx + dims[0] / 2.0
                barrier_pose.pose.position.y = by + dims[1] / 2.0
                barrier_pose.pose.position.z = bz + dims[2] / 2.0
                barrier_pose.pose.orientation.w = 1.0

                barrier.primitives.append(barrier_primitive)
                barrier.primitive_poses.append(barrier_pose.pose)
                barrier.operation = CollisionObject.ADD
                scene.world.collision_objects.append(barrier)

                barrier_color = ObjectColor()
                barrier_color.id = barrier.id
                barrier_color.color.r = 1.0  # yellow
                barrier_color.color.g = 1.0
                barrier_color.color.b = 0.0
                barrier_color.color.a = barrier_A  # semi-transparent
                scene.object_colors.append(barrier_color)



        # -------------------------------
        # Apply scene via service
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        self.cli.wait_for_service()
        self.get_logger().info("/apply_planning_scene service available")

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.future = self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = AddTableNode()
    rclpy.spin_until_future_complete(node, node.future)
    try:
        res = node.future.result()
        if res and res.success:
            node.get_logger().info("Table scene successfully updated in MoveIt planning scene!")
        else:
            node.get_logger().error("Failed to apply planning scene to MoveIt.")
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

