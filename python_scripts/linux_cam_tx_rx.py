import serial
import struct
import time

def main(args=None):

	BLE_ARRAY_MAX 		= 227
	CAM_LEN 			= 65
	OFFSET 				= 3
	ser 				= serial.Serial('/dev/ttyTHS0', 115200)
	bc_ref_pos_lattitude 	= 1
	pkt_no 				= 0;

	status 					= 0
	read_mode 				= 0
	data 					= bytearray(CAM_LEN+1)
	crc_cal 				= 0
	index 					= 0
	index_2					= 0

	cur_time 				= time.time();
	while True:
#_____________________________________(Tx)___________________________________________

		if((time.time() - cur_time) >=  1):
			cur_time			= time.time()
			ser.reset_output_buffer()

			uart_len 			= 71

			pdu_proto_version 	= 1
			pdu_message_id 		= 2
			pdu_src_station_id 	= 2

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

			string += struct.pack('!B',0xff)
			string += struct.pack('!B',0x00)
			string += struct.pack('!B',0xff)
			string += struct.pack('!B',uart_len)

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
			string += struct.pack('!B',bc_ref_pos_altitude_conf)

			string += struct.pack('!H',hf_heading_val)
			string += struct.pack('!B',hf_heading_conf)
			string += struct.pack('!H',hf_vertical_heading)
			string += struct.pack('!H',hf_speed_val)
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
			string += struct.pack('!B',hf_vertical_acc_conf)

			string += struct.pack('!H',uav_safetyarearadius)
			string += struct.pack('!B',uav_pathhistory_len)

			string += struct.pack('!B',0xff)
			string += struct.pack('!B',0xff)

			result 				= ser.write(string)

			bc_ref_pos_lattitude 	+= 1
			pkt_no 					+= 1
#_____________________________________(Rx)___________________________________________
		
		while True:
			if read_mode == 0:
				if(ser.inWaiting() > 0):
					databyte 	= int.from_bytes(ser.read(), "big")
				else:
					break
			else:
				databyte 		= data[index_2]
				index_2 		+= 1
				if index_2 >= (CAM_LEN+1):
					read_mode 	= 0
					index_2 	= 0
				else:
					read_mode	= 1

			if status == 5:
				end_flag 		= databyte
				status 			= 0
				if(end_flag == 255 and crc == crc_cal):
					read_mode 	= 0

					pkt_len 								= data[0]

					r_pdu_proto_version 					= data[1]
					r_pdu_message_id 						= data[2]
					r_pdu_src_station_id 					= (data[3]<<24) + (data[4]<<16) + (data[5]<<8) + data[6]

					r_bc_station_type 						= data[7]
					r_bc_ref_pos_lattitude 					= (data[8]<<24) + (data[9]<<16) + (data[10]<<8) + data[11]
					r_bc_ref_pos_longitude 					= (data[12]<<24) + (data[13]<<16) + (data[14]<<8) + data[15]
					r_bc_ref_pos_conf_ellipse_semi_major 	= (data[16]<<8) + data[17]
					r_bc_ref_pos_conf_ellipse_semi_minor 	= (data[18]<<8) + data[19]
					r_bc_ref_pos_altitude_heading 			= (data[20]<<8) + data[21]
					r_bc_ref_pos_altitude_val 				= (data[22]<<24) + (data[23]<<16) + (data[24]<<8) + data[25]
					r_bc_ref_pos_altitude_conf 				= data[26]

					r_hf_heading_val 						= (data[27]<<8) + data[28]
					r_hf_heading_conf  						= data[29]
					r_hf_vertical_heading 					= (data[30]<<8) + data[31]
					r_hf_speed_val 							= (data[32]<<8) + data[33]
					r_hf_speed_conf 						= data[34]
					r_hf_drive_dir_heading_val 				= (data[35]<<8) + data[36]
					r_hf_drive_dir_heading_conf 			= data[37]
					r_hf_drive_dir_vertical_heading 		= (data[38]<<8) + data[39]
					r_hf_v_len_val 							= (data[40]<<8) + data[41]
					r_hf_v_len_conf 						= data[42]
					r_hf_v_width 							= (data[43]<<8) + data[44]
					r_hf_v_height 							= (data[45]<<8) + data[46]
					r_hf_long_acc_val 						= (data[47]<<8) + data[48]
					r_hf_long_acc_conf 						= data[49]

					r_hf_curv_val 							= (data[50]<<8) + data[51]
					r_hf_curv_conf 							= data[52]
					r_hf_curv_cal_mod 						= data[53]
					r_hf_yaw_r_val 							= (data[54]<<8) + data[55]
					r_hf_yaw_r_conf 						= data[56]
					r_hf_lat_acc_val 						= (data[57]<<8) + data[58]
					r_hf_lat_acc_conf 						= data[59]
					r_hf_vertical_acc_val 					= (data[60]<<8) + data[61]
					r_hf_vertical_acc_conf 					= data[62]

					r_uav_safetyarearadius 					= (data[63]<<8) + data[64]
					r_uav_pathhistory_len 					= data[65]

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

				else:
					read_mode 	= 1
				break

			if status == 4:
				crc 			= databyte
				status 			= 5

			if status == 3:
				data[index] = databyte
				crc_cal 	= crc_cal ^ data[index]
				index 		+= 1
				if index >= (CAM_LEN+1):
					index 	= 0
					status 	= 4

			if status == 2 and databyte == 255:
				status 		= 3

			if status == 1 and databyte == 0:
				status 		= 2

			if status == 0 and databyte == 255:
				crc_cal 	= 0
				status 		= 1

	ser.close()

if __name__ == '__main__':
	main()

