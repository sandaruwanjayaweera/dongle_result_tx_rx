uint8_t 	pdu_proto_version 				= 1;
uint8_t  	pdu_message_id 					  = 2;
union 	pdu_src_station_id_t{
	uint32_t bit_32;
	uint8_t bit_8[4];
} pdu_src_station_id 						= { .bit_32 = 100 };

uint8_t 	bc_station_type 					= 0;
union bc_ref_pos_lattitude_t{
	uint32_t bit_32;
	uint8_t bit_8[4];
} bc_ref_pos_lattitude 							= { .bit_32 = 0xaabbccdd };
union bc_ref_pos_longitude_t{
 uint32_t bit_32;
 uint8_t bit_8[4];
} bc_ref_pos_longitude 							= { .bit_32 = 599 };
union bc_ref_pos_conf_ellipse_semi_major_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} bc_ref_pos_conf_ellipse_semi_major 			= { .bit_16 = 599 };
union bc_ref_pos_conf_ellipse_semi_minor_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} bc_ref_pos_conf_ellipse_semi_minor 			= { .bit_16 = 0 };
union bc_ref_pos_altitude_heading_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} bc_ref_pos_altitude_heading 					= { .bit_16 = 0 };
union bc_ref_pos_altitude_val_t{
 uint32_t bit_32;
 uint8_t bit_8[4];
} bc_ref_pos_altitude_val 						= { .bit_32 = 1999 };
uint8_t bc_ref_pos_altitude_conf 				= 0;

union hf_heading_val_t{
 uint32_t bit_16;
 uint8_t bit_8[2];
} hf_heading_val 								= { .bit_16 = 1999 };
uint8_t hf_heading_conf 						= 0;
union hf_vertical_heading_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_vertical_heading 					        = { .bit_16 = 0 };
union hf_speed_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_speed_val 					                = { .bit_16 = 0 };
uint8_t hf_speed_conf 						    = 0;
union hf_drive_dir_heading_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_drive_dir_heading_val 					    = { .bit_16 = 0 };
uint8_t hf_drive_dir_heading_conf 				= 0;
union hf_drive_dir_vertical_heading_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_drive_dir_vertical_heading 				= { .bit_16 = 0 };
union hf_v_len_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_v_len_val 					                = { .bit_16 = 0 };
uint8_t hf_v_len_conf 						    = 0;
union hf_v_width_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_v_width 					                = { .bit_16 = 0 };
union hf_v_height_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_v_height 					                = { .bit_16 = 0 };
union hf_long_acc_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_long_acc_val 					            = { .bit_16 = 0 };
uint8_t hf_long_acc_conf 						= 0;
union hf_curv_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_curv_val 					                = { .bit_16 = 0 };
uint8_t hf_curv_conf 						    = 0;
uint8_t hf_curv_cal_mod 						= 0;
union hf_yaw_r_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_yaw_r_val 					                = { .bit_16 = 0 };
uint8_t hf_yaw_r_conf 						    = 0;
union hf_lat_acc_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_lat_acc_val 					            = { .bit_16 = 0 };
uint8_t hf_lat_acc_conf 						= 0;
union hf_vertical_acc_val_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} hf_vertical_acc_val 					        = { .bit_16 = 0 };
uint8_t hf_vertical_acc_conf 					= 0;

union uav_safetyarearadius_t{
 uint16_t bit_16;
 uint8_t bit_8[2];
} uav_safetyarearadius 					        = { .bit_16 = 0 };
uint8_t uav_pathhistory_len 					= 0;
