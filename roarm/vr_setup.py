import nestbox

nestbox.create_coordinate_system('vr')
nestbox.create_coordinate_system('roarm')

nestbox.add_measurement(
    feature='nestbox:feature/hand/lefthand/root-pose/position',
    cs='vr',
    mean=[2.0, 3.0, 4.0],
    covariance=[[0.001, 0, 0],
                [0, 0.001, 0],
                [0, 0, 0.001]])

nestbox.start_alignment()
