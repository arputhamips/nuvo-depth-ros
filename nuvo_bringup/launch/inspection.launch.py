from nuvo_bringup.camera_launch import camera_launch_description


def generate_launch_description():
    return camera_launch_description('nuvo/inspection', 'inspection.yaml', inspection=True)
