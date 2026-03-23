from marvin_real_control.marvin_moveit_bridge_demo import MarvinMoveItBridgeDemo


class DummyPoint:
    def __init__(self, positions):
        self.positions = positions


def test_snapshot_trajectory_extracts_target_arm_joints():
    demo = MarvinMoveItBridgeDemo.__new__(MarvinMoveItBridgeDemo)
    demo.joint_names = [f"Joint{i}_L" for i in range(1, 8)]

    joint_names = ["Joint1_L", "Joint2_L", "Joint3_L", "OtherJoint"]
    points = [
        DummyPoint([0.1, 0.2, 0.3, 9.9]),
        DummyPoint([1.1, 1.2, 1.3, 8.8]),
    ]

    snapshot = demo._snapshot_trajectory(
        source="test",
        joint_names=joint_names,
        points=points,
    )

    assert snapshot is not None
    assert snapshot["source"] == "test"
    assert snapshot["point_count"] == 2
    assert snapshot["joint_names"] == ["Joint1_L", "Joint2_L", "Joint3_L"]
    assert snapshot["first_point"]["Joint1_L"] == 0.1
    assert snapshot["last_point"]["Joint3_L"] == 1.3
