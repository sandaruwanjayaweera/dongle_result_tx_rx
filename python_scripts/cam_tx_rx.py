import serial
import struct
import time

def main(args=None):

	BLE_ARRAY_MAX 		= 227
	CAM_LEN 			= 65 		# length of CAM datafields
	OFFSET 				= 3
	ser 				= serial.Serial('/dev/ttyTHS0', 115200)
	bc_ref_pos_lattitude 	= 1
	uart_flag 				= 1

	while True:
#_____________________________________(Tx)___________________________________________

		ser.reset_output_buffer()
		uart_len 			= CAM_LEN + 2; 	# CAM length + uart_flag + uart_flag_2
		if(uart_flag == 255):
			uart_flag = 0
		else:
			uart_flag += 1
		uart_flag_2 		= 255 - uart_flag

		pdu_proto_version 	= 1
		pdu_message_id 		= 2
		pdu_src_station_id 	= 1

		bc_station_type 					= 4
		# bc_ref_pos_lattitude 				= 5
		bc_ref_pos_longitude 				= 6
		bc_ref_pos_conf_ellipse_semi_major 	= 7
		bc_ref_pos_conf_ellipse_semi_minor 	= 8
		bc_ref_pos_altitude_heading 		= 9
		bc_ref_pos_altitude_val 			= 10
		bc_ref_pos_altitude_conf 			= 11

		hf_heading_val 						= 12
		hf_heading_conf  					= 13
		hf_vertical_heading 				= 14
		hf_speed_val 						= 15
		hf_speed_conf 						= 16
		hf_drive_dir_heading_val 			= 17
		hf_drive_dir_heading_conf 			= 18
		hf_drive_dir_vertical_heading 		= 19
		hf_v_len_val 						= 20
		hf_v_len_conf 						= 21
		hf_v_width 							= 22
		hf_v_height 						= 23
		hf_long_acc_val 					= 24
		hf_long_acc_conf 					= 25
		hf_curv_val 						= 26
		hf_curv_conf 						= 27
		hf_curv_cal_mod 					= 28
		hf_yaw_r_val 						= 29
		hf_yaw_r_conf 						= 30
		hf_lat_acc_val 						= 31
		hf_lat_acc_conf 					= 32
		hf_vertical_acc_val 				= 33
		hf_vertical_acc_conf 				= 34

		uav_safetyarearadius 				= 35
		uav_pathhistory_len 				= 36

		string 				= b''

		string += struct.pack('!B',uart_flag)
		string += struct.pack('!B',uart_flag_2)
		string += struct.pack('!B',pdu_proto_version)
		string += struct.pack('!B',pdu_message_id)
		string += struct.pack('!I',pdu_src_station_id)
		string += struct.pack('!B',bc_station_type)
		string += struct.pack('!I',bc_ref_pos_lattitude)
		string += struct.pack('!I',bc_ref_pos_longitude)
		string += struct.pack('!H',bc_ref_pos_conf_ellipse_semi_major)
		string += struct.pack('!H',bc_ref_pos_conf_ellipse_semi_minor)
		string += struct.pack('!H',bc_ref_pos_altitude_heading)
		string += struct.pack('!I',bc_ref_pos_altitude_val)
		string += struct.pack('!B',bc_ref_pos_altitude_conf) 				# 28


		string += struct.pack('!H',hf_heading_val) 							# 30
		string += struct.pack('!B',hf_heading_conf) 						# 31
		string += struct.pack('!H',hf_vertical_heading) 					# 33
		string += struct.pack('!H',hf_speed_val) 							# 35

		string += struct.pack('c',b'\r')
		result 				= ser.write(string)

		string += struct.pack('!B',hf_speed_conf)
		string += struct.pack('!H',hf_drive_dir_heading_val)
		string += struct.pack('!B',hf_drive_dir_heading_conf)
		string += struct.pack('!H',hf_drive_dir_vertical_heading)
		string += struct.pack('!H',hf_v_len_val)
		string += struct.pack('!B',hf_v_len_conf)
		string += struct.pack('!H',hf_v_width)
		string += struct.pack('!H',hf_v_height)
		string += struct.pack('!H',hf_long_acc_val)
		string += struct.pack('!B',hf_long_acc_conf)
		string += struct.pack('!H',hf_curv_val)
		string += struct.pack('!B',hf_curv_conf)
		string += struct.pack('!B',hf_curv_cal_mod)
		string += struct.pack('!H',hf_yaw_r_val)
		string += struct.pack('!B',hf_yaw_r_conf)
		string += struct.pack('!H',hf_lat_acc_val)
		string += struct.pack('!B',hf_lat_acc_conf)
		string += struct.pack('!H',hf_vertical_acc_val)
		string += struct.pack('!B',hf_vertical_acc_conf) 					# 64

		string += struct.pack('!H',uav_safetyarearadius)
		string += struct.pack('!B',uav_pathhistory_len) 					# 67

		# string += struct.pack('c',b'\r')
		# # print(string)
		# result 				= ser.write(string)
		# print(result)
		bc_ref_pos_lattitude 	+= 1
		time.sleep(0.5)
#_____________________________________(Rx)___________________________________________

		if(ser.inWaiting() > 0):
			r_pdu_proto_version 	= int.from_bytes(ser.read(), "big")
			r_pdu_message_id 		= int.from_bytes(ser.read(), "big")
			if(r_pdu_proto_version == 1 and r_pdu_message_id == 2):
				r_pdu_src_station_id 	= int.from_bytes(ser.read(4), "big")					# 6

				r_bc_station_type 						= int.from_bytes(ser.read(), "big")
				r_bc_ref_pos_lattitude 					= int.from_bytes(ser.read(4), "big")
				r_bc_ref_pos_longitude 					= int.from_bytes(ser.read(4), "big")
				r_bc_ref_pos_conf_ellipse_semi_major 	= int.from_bytes(ser.read(2), "big")
				r_bc_ref_pos_conf_ellipse_semi_minor 	= int.from_bytes(ser.read(2), "big")
				r_bc_ref_pos_altitude_heading 			= int.from_bytes(ser.read(2), "big")
				r_bc_ref_pos_altitude_val 				= int.from_bytes(ser.read(4), "big")
				r_bc_ref_pos_altitude_conf 				= int.from_bytes(ser.read(), "big") 	# 26

				r_hf_heading_val 						= int.from_bytes(ser.read(2), "big")
				r_hf_heading_conf  						= int.from_bytes(ser.read(), "big")
				r_hf_vertical_heading 					= int.from_bytes(ser.read(2), "big")
				r_hf_speed_val 							= int.from_bytes(ser.read(2), "big")
				r_hf_speed_conf 						= int.from_bytes(ser.read(), "big")
				r_hf_drive_dir_heading_val 				= int.from_bytes(ser.read(2), "big")
				r_hf_drive_dir_heading_conf 			= int.from_bytes(ser.read(), "big")
				r_hf_drive_dir_vertical_heading 		= int.from_bytes(ser.read(2), "big")
				r_hf_v_len_val 							= int.from_bytes(ser.read(2), "big")
				r_hf_v_len_conf 						= int.from_bytes(ser.read(), "big")
				r_hf_v_width 							= int.from_bytes(ser.read(2), "big")
				r_hf_v_height 							= int.from_bytes(ser.read(2), "big")
				r_hf_long_acc_val 						= int.from_bytes(ser.read(2), "big")
				r_hf_long_acc_conf 						= int.from_bytes(ser.read(), "big")
				r_hf_curv_val 							= int.from_bytes(ser.read(2), "big")
				r_hf_curv_conf 							= int.from_bytes(ser.read(), "big")
				r_hf_curv_cal_mod 						= int.from_bytes(ser.read(), "big")
				r_hf_yaw_r_val 							= int.from_bytes(ser.read(2), "big")
				r_hf_yaw_r_conf 						= int.from_bytes(ser.read(), "big")
				r_hf_lat_acc_val 						= int.from_bytes(ser.read(2), "big")
				r_hf_lat_acc_conf 						= int.from_bytes(ser.read(), "big")
				r_hf_vertical_acc_val 					= int.from_bytes(ser.read(2), "big")
				r_hf_vertical_acc_conf 					= int.from_bytes(ser.read(), "big")

				r_uav_safetyarearadius 					= int.from_bytes(ser.read(2), "big")
				r_uav_pathhistory_len 					= int.from_bytes(ser.read(), "big") 	# 65

				# ser.read(BLE_ARRAY_MAX - (CAM_LEN + OFFSET))

				print('______________________________________ \n \
					pdu_proto_version - %d, \n \
					pdu_message_id - %d, \n \
					pdu_src_station_id - %d, \n \
					bc_station_type - %d, \n \
					bc_ref_pos_lattitude - %d, \n \
					bc_ref_pos_longitude - %d, \n \
					bc_ref_pos_conf_ellipse_semi_major - %d, \n \
					bc_ref_pos_conf_ellipse_semi_minor - %d, \n \
					bc_ref_pos_altitude_heading - %d, \n \
					bc_ref_pos_altitude_val - %d, \n \
					bc_ref_pos_altitude_conf - %d \n \
					\n \
					hf_heading_val - %d, \n \
					hf_heading_conf - %d, \n \
					hf_vertical_heading - %d, \n \
					hf_speed_val - %d, \n \
					hf_speed_conf - %d, \n \
					hf_drive_dir_heading_val - %d, \n \
					hf_drive_dir_heading_conf - %d, \n \
					hf_drive_dir_vertical_heading - %d, \n \
					hf_v_len_val - %d, \n \
					hf_v_len_conf - %d, \n \
					hf_v_width - %d \n \
					hf_v_height - %d, \n \
					hf_long_acc_val - %d, \n \
					hf_long_acc_conf - %d, \n \
					hf_curv_val - %d, \n \
					hf_curv_conf - %d, \n \
					hf_curv_cal_mod - %d, \n \
					hf_yaw_r_val - %d, \n \
					hf_yaw_r_conf - %d, \n \
					hf_lat_acc_val - %d, \n \
					hf_lat_acc_conf - %d, \n \
					hf_vertical_acc_val - %d \n \
					hf_vertical_acc_conf - %d \n \
					\n \
					uav_safetyarearadius - %d, \n \
					uav_pathhistory_len - %d, \n \
					'
						%(	r_pdu_proto_version,
							r_pdu_message_id,
							r_pdu_src_station_id,
							r_bc_station_type,
							r_bc_ref_pos_lattitude,
							r_bc_ref_pos_longitude,
							r_bc_ref_pos_conf_ellipse_semi_major,
							r_bc_ref_pos_conf_ellipse_semi_minor,
							r_bc_ref_pos_altitude_heading,
							r_bc_ref_pos_altitude_val,
							r_bc_ref_pos_altitude_conf,
							r_hf_heading_val,
							r_hf_heading_conf,
							r_hf_vertical_heading,
							r_hf_speed_val,
							r_hf_speed_conf,
							r_hf_drive_dir_heading_val,
							r_hf_drive_dir_heading_conf,
							r_hf_drive_dir_vertical_heading,
							r_hf_v_len_val,
							r_hf_v_len_conf,
							r_hf_v_width,
							r_hf_v_height,
							r_hf_long_acc_val,
							r_hf_long_acc_conf,
							r_hf_curv_val,
							r_hf_curv_conf,
							r_hf_curv_cal_mod,
							r_hf_yaw_r_val,
							r_hf_yaw_r_conf,
							r_hf_lat_acc_val,
							r_hf_lat_acc_conf,
							r_hf_vertical_acc_val,
							r_hf_vertical_acc_conf,
							r_uav_safetyarearadius,
							r_uav_pathhistory_len
						))
				time.sleep(0.05)
				# print(ser.read(BLE_ARRAY_MAX - OFFSET))
			else:
				print(ser.readline())
				time.sleep(0.05)
		else:
			# print('No data bc_ref_pos_longitude %d', bc_ref_pos_longitude)
			time.sleep(0.05)
			continue

	ser.close()

if __name__ == '__main__':
	main()

