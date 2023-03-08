from realsensescan.eye_in_hand import helpers, eye_in_hand

# CameraControl.capture_rgb_camera("data/images/230223-Robot_Images-D455-9X6")
# eye_in_hand.calibrate_camera("data/images/Calibration_Images-D455",
#                              "data/boards/charuco.json",
#                              "data/cameras/D455-Jan26")
# eye_in_hand.generate_charuco_board("data/boards/charuco4.json", "data/boards/charuco_board4.png")

eye_in_hand.preprocess_calibration_program("../data/robot_files/HECalib_T_ROB1.mod",
                                           "../data/images/230223-Robot_Images-D455-9X6",
                                           "../data/robot_poses/230223-Robot_Poses-D455-9X6.csv")

calibration_model = eye_in_hand.preprocess_calibration_data("../data/robot_poses/230223-Robot_Poses-D455-9X6.csv",
                                                            "../data/images/230223-Robot_Images-D455-9X6",
                                                            "../data/cameras/D455-Jan26.json",
                                                            "../data/boards/charuco4.json")

calib = eye_in_hand.calibrate(calibration_model)

helpers.write_hand_eye_transform(*calib)

print("Rotation Matrix:\n{}".format(calib[0]))
print("Translation Vector:\n{}".format(calib[1]))
