"""Enable PhysX drag-and-grab for rapid debugging."""

import omni.physx as physx


def enable_mouse_grab(stiffness: float = 150.0, damping: float = 5.0) -> None:
    interface = physx.get_physx_interface()
    interface.set_setting_bool("physxEnableDirectMouseControl", True)
    interface.set_setting_bool("physxEnableArticulationJointDrives", True)
    interface.set_setting_float("physxMouseSpringStiffness", stiffness)
    interface.set_setting_float("physxMouseSpringDamping", damping)
